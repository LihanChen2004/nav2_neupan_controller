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

#ifndef NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_
#define NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "Python.h"
#include "nav2_neupan_controller/neupan_types.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_neupan_controller
{

// All Python C API interaction is funneled through this class.
// Every public method acquires the GIL internally; callers need not manage it.
class PythonBridge
{
public:
  explicit PythonBridge(const rclcpp::Logger & logger);
  ~PythonBridge();

  // Initialise Python interpreter and create the NeuPAN instance.
  bool initialize(const std::string & config_path, const std::string & dune_model_path);
  void cleanup();

  bool isInitialized() const { return initialized_; }
  const RobotInfo & robotInfo() const { return robot_info_; }

  // Call neupan.forward(state, obstacles, None).
  // Returns false only on hard failure; stop/arrive flags are inside PlannerOutput.
  bool callForward(
    const std::array<double, 3> & robot_state,
    const std::vector<std::pair<double, double>> & obstacles, const std::string & robot_type,
    PlannerOutput & output);

  // Call neupan.reset() — invoke on every new plan.
  bool callReset();

  // Convert Nav2 path to NeuPAN waypoint list and call set_initial_path().
  bool setInitialPath(const std::vector<NeuPANWaypoint> & path);

  // Call neupan.update_initial_path_from_goal(start, goal) — recover from early arrive.
  bool updatePathFromGoal(const std::array<double, 3> & start, const std::array<double, 3> & goal);

private:
  // RAII GIL guard — acquire on construction, release on destruction.
  struct GILGuard
  {
    PyGILState_STATE state;
    GILGuard() { state = PyGILState_Ensure(); }
    ~GILGuard() { PyGILState_Release(state); }
  };

  bool initNumpy();
  bool loadRobotInfo();

  void decodeAction(
    PyObject * action, const std::string & robot_type, geometry_msgs::msg::Twist & cmd_vel);
  void extractVisualizationData(PyObject * info_dict, PlannerOutput & output);

  // Data extraction helpers — must be called under the GIL.
  static std::vector<std::array<double, 3>> extractStateList(PyObject * py_list);
  static std::vector<std::array<double, 3>> extractStateArray(PyObject * py_array);
  static std::vector<std::pair<double, double>> extractPointCloud(PyObject * py_array);
  static PyObject * makeStateArray(double x, double y, double theta);

  rclcpp::Logger logger_;
  PyObject * neupan_instance_{nullptr};
  bool initialized_{false};
  RobotInfo robot_info_;
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_
