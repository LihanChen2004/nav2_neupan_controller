// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_neupan_controller/python_bridge.hpp"

#include <dlfcn.h>

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "numpy/arrayobject.h"

namespace nav2_neupan_controller
{

// ─────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────

PythonBridge::PythonBridge(const rclcpp::Logger & logger) : logger_(logger) {}

PythonBridge::~PythonBridge() { cleanup(); }

bool PythonBridge::initialize(const std::string & config_path, const std::string & dune_model_path)
{
  if (initialized_) return true;

  if (config_path.empty()) {
    RCLCPP_ERROR(logger_, "neupan_config_path is empty");
    return false;
  }

  RCLCPP_INFO(logger_, "Initializing Python interpreter for NeuPAN");

  if (!Py_IsInitialized()) {
    void * lib = dlopen("/usr/lib/x86_64-linux-gnu/libpython3.12.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!lib) {
      RCLCPP_DEBUG(logger_, "Could not pre-load Python shared library: %s", dlerror());
    }
    Py_Initialize();
    if (!Py_IsInitialized()) {
      RCLCPP_ERROR(logger_, "Failed to initialize Python interpreter");
      return false;
    }
    PyEval_SaveThread();  // Release GIL so worker threads can use PyGILState_Ensure
  }

  GILGuard gil;

  if (!initNumpy()) {
    RCLCPP_ERROR(logger_, "Failed to initialize NumPy C API");
    return false;
  }

  PyObject * module = PyImport_ImportModule("neupan.neupan");
  if (!module) {
    RCLCPP_ERROR(logger_, "Failed to import neupan.neupan");
    PyErr_Print();
    return false;
  }

  PyObject * cls = PyObject_GetAttrString(module, "neupan");
  Py_DECREF(module);
  if (!cls) {
    RCLCPP_ERROR(logger_, "Cannot find neupan class in module");
    PyErr_Print();
    return false;
  }

  PyObject * init_fn = PyObject_GetAttrString(cls, "init_from_yaml");
  Py_DECREF(cls);
  if (!init_fn || !PyCallable_Check(init_fn)) {
    RCLCPP_ERROR(logger_, "Cannot find init_from_yaml class method");
    PyErr_Print();
    Py_XDECREF(init_fn);
    return false;
  }

  PyObject * config_arg = PyUnicode_FromString(config_path.c_str());
  PyObject * args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, config_arg);  // steals ref

  PyObject * kwargs = PyDict_New();
  if (!dune_model_path.empty()) {
    PyObject * pan_dict = PyDict_New();
    PyObject * ckpt = PyUnicode_FromString(dune_model_path.c_str());
    PyDict_SetItemString(pan_dict, "dune_checkpoint", ckpt);
    Py_DECREF(ckpt);
    PyDict_SetItemString(kwargs, "pan", pan_dict);
    Py_DECREF(pan_dict);
  }

  neupan_instance_ = PyObject_Call(init_fn, args, kwargs);
  Py_DECREF(args);
  Py_DECREF(kwargs);
  Py_DECREF(init_fn);

  if (!neupan_instance_) {
    RCLCPP_ERROR(logger_, "Failed to create NeuPAN instance");
    PyErr_Print();
    return false;
  }

  initialized_ = true;
  loadRobotInfo();
  RCLCPP_INFO(logger_, "Python / NeuPAN initialized successfully");
  return true;
}

void PythonBridge::cleanup()
{
  if (!initialized_ && !neupan_instance_) return;
  if (!Py_IsInitialized()) {
    neupan_instance_ = nullptr;
    initialized_ = false;
    return;
  }
  GILGuard gil;
  Py_XDECREF(neupan_instance_);
  neupan_instance_ = nullptr;
  initialized_ = false;
}

bool PythonBridge::initNumpy()
{
  if (_import_array() >= 0) return true;
  PyErr_Print();
  PyErr_Clear();
  return false;
}

bool PythonBridge::loadRobotInfo()
{
  PyObject * robot = PyObject_GetAttrString(neupan_instance_, "robot");
  if (!robot) {
    PyErr_Clear();
    return false;
  }

  auto get_str = [&](const char * attr, std::string & out) {
    PyObject * o = PyObject_GetAttrString(robot, attr);
    if (o && PyUnicode_Check(o)) out = PyUnicode_AsUTF8(o);
    Py_XDECREF(o);
    PyErr_Clear();
  };
  auto get_dbl = [&](const char * attr, double & out) {
    PyObject * o = PyObject_GetAttrString(robot, attr);
    if (o && PyFloat_Check(o)) out = PyFloat_AsDouble(o);
    Py_XDECREF(o);
    PyErr_Clear();
  };

  get_str("shape", robot_info_.shape);
  get_str("kinematics", robot_info_.kinematics);
  get_dbl("length", robot_info_.length);
  get_dbl("width", robot_info_.width);
  get_dbl("wheelbase", robot_info_.wheelbase);
  Py_DECREF(robot);

  RCLCPP_INFO(
    logger_, "Robot: shape=%s kinematics=%s length=%.3f width=%.3f wheelbase=%.3f",
    robot_info_.shape.c_str(), robot_info_.kinematics.c_str(), robot_info_.length,
    robot_info_.width, robot_info_.wheelbase);
  return true;
}

// ─────────────────────────────────────────────────────────────
//  NeuPAN API calls
// ─────────────────────────────────────────────────────────────

bool PythonBridge::callForward(
  const std::array<double, 3> & robot_state,
  const std::vector<std::pair<double, double>> & obstacles, const std::string & robot_type,
  PlannerOutput & output)
{
  if (!initialized_ || !neupan_instance_) return false;
  GILGuard gil;

  // Build state (3, 1)
  npy_intp rdims[2] = {3, 1};
  PyObject * state_arr = PyArray_SimpleNew(2, rdims, NPY_DOUBLE);
  if (!state_arr) {
    RCLCPP_ERROR(logger_, "Failed to create robot state array");
    PyErr_Print();
    return false;
  }
  double * sd = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(state_arr)));
  sd[0] = robot_state[0];
  sd[1] = robot_state[1];
  sd[2] = robot_state[2];

  // Build obstacles (2, N) or None
  PyObject * obs_arr = nullptr;
  if (!obstacles.empty()) {
    npy_intp odims[2] = {2, static_cast<npy_intp>(obstacles.size())};
    obs_arr = PyArray_SimpleNew(2, odims, NPY_DOUBLE);
    if (!obs_arr) {
      RCLCPP_ERROR(logger_, "Failed to create obstacle array");
      PyErr_Print();
      Py_DECREF(state_arr);
      return false;
    }
    double * od = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(obs_arr)));
    const size_t n_obs = obstacles.size();
    for (size_t i = 0; i < n_obs; ++i) {
      od[i] = obstacles[i].first;           // x row
      od[n_obs + i] = obstacles[i].second;  // y row
    }
  } else {
    obs_arr = Py_None;
    Py_INCREF(obs_arr);
  }

  PyObject * fwd = PyObject_GetAttrString(neupan_instance_, "forward");
  if (!fwd || !PyCallable_Check(fwd)) {
    RCLCPP_ERROR(logger_, "Cannot find forward() on NeuPAN instance");
    Py_XDECREF(fwd);
    Py_DECREF(state_arr);
    Py_DECREF(obs_arr);
    return false;
  }

  PyObject * args = PyTuple_New(3);
  PyTuple_SetItem(args, 0, state_arr);  // steals ref
  PyTuple_SetItem(args, 1, obs_arr);    // steals ref
  Py_INCREF(Py_None);
  PyTuple_SetItem(args, 2, Py_None);  // velocities = None

  PyObject * result = PyObject_CallObject(fwd, args);
  Py_DECREF(args);
  Py_DECREF(fwd);

  if (!result) {
    RCLCPP_ERROR(logger_, "NeuPAN forward() raised an exception");
    PyErr_Print();
    return false;
  }

  if (!PyTuple_Check(result) || PyTuple_Size(result) != 2) {
    RCLCPP_ERROR(logger_, "Unexpected return format from forward()");
    Py_DECREF(result);
    return false;
  }

  PyObject * action = PyTuple_GetItem(result, 0);  // borrowed
  PyObject * info = PyTuple_GetItem(result, 1);    // borrowed

  PyObject * stop_obj = PyDict_GetItemString(info, "stop");
  PyObject * arrive_obj = PyDict_GetItemString(info, "arrive");
  output.stop = stop_obj && PyObject_IsTrue(stop_obj);
  output.arrive = arrive_obj && PyObject_IsTrue(arrive_obj);

  if (output.stop) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 500, "NeuPAN stopped — collision threshold reached");
  }
  if (output.arrive) {
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 100, "NeuPAN arrived at target");
  }

  if (!output.stop && !output.arrive) {
    decodeAction(action, robot_type, output.cmd_vel);
  }

  extractVisualizationData(info, output);
  Py_DECREF(result);
  return true;
}

bool PythonBridge::callReset()
{
  if (!initialized_ || !neupan_instance_) return false;
  GILGuard gil;

  PyObject * fn = PyObject_GetAttrString(neupan_instance_, "reset");
  if (!fn || !PyCallable_Check(fn)) {
    Py_XDECREF(fn);
    PyErr_Clear();
    return false;
  }
  PyObject * r = PyObject_CallObject(fn, nullptr);
  Py_DECREF(fn);
  if (!r) {
    RCLCPP_WARN(logger_, "NeuPAN reset() failed");
    PyErr_Print();
    return false;
  }
  Py_DECREF(r);
  RCLCPP_DEBUG(logger_, "NeuPAN internal state reset");
  return true;
}

bool PythonBridge::setInitialPath(const std::vector<NeuPANWaypoint> & path)
{
  if (!initialized_ || !neupan_instance_) return false;
  GILGuard gil;

  PyObject * wp_list = PyList_New(0);
  if (!wp_list) {
    PyErr_Print();
    return false;
  }

  for (const auto & wp : path) {
    npy_intp dims[2] = {4, 1};
    PyObject * pwp = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    if (!pwp) {
      PyErr_Print();
      Py_DECREF(wp_list);
      return false;
    }
    double * d = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(pwp)));
    d[0] = wp.x;
    d[1] = wp.y;
    d[2] = wp.theta;
    d[3] = wp.gear;

    if (PyList_Append(wp_list, pwp) != 0) {
      PyErr_Print();
      Py_DECREF(pwp);
      Py_DECREF(wp_list);
      return false;
    }
    Py_DECREF(pwp);
  }

  PyObject * fn = PyObject_GetAttrString(neupan_instance_, "set_initial_path");
  if (!fn || !PyCallable_Check(fn)) {
    RCLCPP_ERROR(logger_, "Cannot find set_initial_path()");
    Py_XDECREF(fn);
    Py_DECREF(wp_list);
    return false;
  }

  PyObject * args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, wp_list);  // steals ref
  PyObject * r = PyObject_CallObject(fn, args);
  Py_DECREF(args);
  Py_DECREF(fn);

  if (!r) {
    RCLCPP_ERROR(logger_, "set_initial_path() failed");
    PyErr_Print();
    return false;
  }
  Py_DECREF(r);
  return true;
}

bool PythonBridge::updatePathFromGoal(
  const std::array<double, 3> & start, const std::array<double, 3> & goal)
{
  if (!initialized_ || !neupan_instance_) return false;
  GILGuard gil;

  PyObject * start_arr = makeStateArray(start[0], start[1], start[2]);
  PyObject * goal_arr = makeStateArray(goal[0], goal[1], goal[2]);
  if (!start_arr || !goal_arr) {
    Py_XDECREF(start_arr);
    Py_XDECREF(goal_arr);
    PyErr_Print();
    return false;
  }

  PyObject * fn = PyObject_GetAttrString(neupan_instance_, "update_initial_path_from_goal");
  if (!fn || !PyCallable_Check(fn)) {
    Py_XDECREF(fn);
    Py_DECREF(start_arr);
    Py_DECREF(goal_arr);
    PyErr_Clear();
    return false;
  }

  PyObject * args = PyTuple_New(2);
  PyTuple_SetItem(args, 0, start_arr);  // steals ref
  PyTuple_SetItem(args, 1, goal_arr);   // steals ref
  PyObject * r = PyObject_CallObject(fn, args);
  Py_DECREF(args);
  Py_DECREF(fn);

  if (!r) {
    RCLCPP_WARN(logger_, "update_initial_path_from_goal() failed");
    PyErr_Clear();
    return false;
  }
  Py_DECREF(r);
  return true;
}

// ─────────────────────────────────────────────────────────────
//  Private helpers — all called under the GIL
// ─────────────────────────────────────────────────────────────

void PythonBridge::decodeAction(
  PyObject * action, const std::string & robot_type, geometry_msgs::msg::Twist & cmd_vel)
{
  if (!PyArray_Check(action)) {
    RCLCPP_ERROR(logger_, "NeuPAN action is not a numpy array");
    return;
  }
  PyArrayObject * arr =
    reinterpret_cast<PyArrayObject *>(PyArray_FROM_OTF(action, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY));
  if (!arr) {
    PyErr_Clear();
    return;
  }

  const double * a = static_cast<const double *>(PyArray_DATA(arr));

  if (robot_type == "omni") {
    cmd_vel.linear.x = a[0];
    cmd_vel.linear.y = a[1];
    cmd_vel.angular.z = 0.0;
  } else if (robot_type == "acker") {
    const double v = a[0];
    const double steering = a[1];
    const double wb = robot_info_.wheelbase;
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = (std::abs(v) > 1e-4) ? (v / wb) * std::tan(steering) : 0.0;
  } else {
    cmd_vel.linear.x = a[0];
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = a[1];
  }

  Py_DECREF(arr);
}

void PythonBridge::extractVisualizationData(PyObject * info_dict, PlannerOutput & output)
{
  PyObject * opt = PyDict_GetItemString(info_dict, "opt_state_list");
  if (opt) output.opt_states = extractStateList(opt);

  PyObject * ref = PyDict_GetItemString(info_dict, "ref_state_list");
  if (ref) output.ref_states = extractStateList(ref);

  // initial_path may be a list or a 2-D numpy array
  PyObject * ipath = PyObject_GetAttrString(neupan_instance_, "initial_path");
  if (ipath && ipath != Py_None) {
    if (PyList_Check(ipath)) {
      output.initial_path = extractStateList(ipath);
    } else if (PyArray_Check(ipath)) {
      output.initial_path = extractStateArray(ipath);
    }
    Py_DECREF(ipath);
  } else {
    Py_XDECREF(ipath);
    PyErr_Clear();
  }

  PyObject * dune = PyObject_GetAttrString(neupan_instance_, "dune_points");
  if (dune && dune != Py_None) {
    output.dune_pts = extractPointCloud(dune);
    Py_DECREF(dune);
  } else {
    Py_XDECREF(dune);
    PyErr_Clear();
  }

  PyObject * nrmp = PyObject_GetAttrString(neupan_instance_, "nrmp_points");
  if (nrmp && nrmp != Py_None) {
    output.nrmp_pts = extractPointCloud(nrmp);
    Py_DECREF(nrmp);
  } else {
    Py_XDECREF(nrmp);
    PyErr_Clear();
  }
}

PyObject * PythonBridge::makeStateArray(double x, double y, double theta)
{
  npy_intp dims[2] = {3, 1};
  PyObject * arr = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
  if (!arr) return nullptr;
  double * d = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
  d[0] = x;
  d[1] = y;
  d[2] = theta;
  return arr;
}

// Extract list of (3,1) or (4,1) numpy arrays → vector of [x,y,θ]
std::vector<std::array<double, 3>> PythonBridge::extractStateList(PyObject * py_list)
{
  std::vector<std::array<double, 3>> result;
  if (!py_list || !PyList_Check(py_list)) return result;

  const Py_ssize_t n = PyList_Size(py_list);
  result.reserve(n);
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject * item = PyList_GetItem(py_list, i);  // borrowed
    if (!item || !PyArray_Check(item)) continue;
    PyArrayObject * arr =
      reinterpret_cast<PyArrayObject *>(PyArray_FROM_OTF(item, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY));
    if (!arr) {
      PyErr_Clear();
      continue;
    }
    const double * d = static_cast<const double *>(PyArray_DATA(arr));
    result.push_back({d[0], d[1], d[2]});
    Py_DECREF(arr);
  }
  return result;
}

// Extract C-contiguous (3, N) numpy array → vector of [x,y,θ]
std::vector<std::array<double, 3>> PythonBridge::extractStateArray(PyObject * py_array)
{
  std::vector<std::array<double, 3>> result;
  if (!py_array || py_array == Py_None || !PyArray_Check(py_array)) return result;

  PyArrayObject * arr =
    reinterpret_cast<PyArrayObject *>(PyArray_FROM_OTF(py_array, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY));
  if (!arr) {
    PyErr_Clear();
    return result;
  }

  if (PyArray_NDIM(arr) == 2 && PyArray_DIM(arr, 0) >= 3) {
    const npy_intp num_cols = PyArray_DIM(arr, 1);
    const double * d = static_cast<const double *>(PyArray_DATA(arr));
    result.reserve(num_cols);
    for (npy_intp col = 0; col < num_cols; ++col) {
      result.push_back({d[0 * num_cols + col], d[1 * num_cols + col], d[2 * num_cols + col]});
    }
  }
  Py_DECREF(arr);
  return result;
}

// Extract C-contiguous (2, N) numpy array → vector of [x,y] pairs
std::vector<std::pair<double, double>> PythonBridge::extractPointCloud(PyObject * py_array)
{
  std::vector<std::pair<double, double>> result;
  if (!py_array || py_array == Py_None || !PyArray_Check(py_array)) return result;

  PyArrayObject * arr =
    reinterpret_cast<PyArrayObject *>(PyArray_FROM_OTF(py_array, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY));
  if (!arr) {
    PyErr_Clear();
    return result;
  }

  if (PyArray_NDIM(arr) == 2 && PyArray_DIM(arr, 0) >= 2) {
    const npy_intp num_pts = PyArray_DIM(arr, 1);
    const double * d = static_cast<const double *>(PyArray_DATA(arr));
    result.reserve(num_pts);
    for (npy_intp i = 0; i < num_pts; ++i) {
      result.emplace_back(d[i], d[num_pts + i]);
    }
  }
  Py_DECREF(arr);
  return result;
}

}  // namespace nav2_neupan_controller
