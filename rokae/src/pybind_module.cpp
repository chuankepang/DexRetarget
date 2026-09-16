#include "rokae_driver.hpp"

#include <cstring>
#include <stdexcept>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using anydex::rokae_bridge::Diagnostics;
using anydex::rokae_bridge::Pose;
using anydex::rokae_bridge::RealtimeConfig;
using anydex::rokae_bridge::RealtimeCore;
using anydex::rokae_bridge::RobotState;
using anydex::rokae_bridge::RokaeDriver;

namespace {

Pose pose_from_numpy(const py::array_t<double, py::array::c_style | py::array::forcecast>& value) {
  if (value.ndim() != 2 || value.shape(0) != 4 || value.shape(1) != 4) {
    throw std::invalid_argument("pose must be a 4x4 float64 matrix");
  }
  Pose pose{};
  std::memcpy(pose.data(), value.data(), sizeof(double) * pose.size());
  return pose;
}

py::array_t<double> numpy_from_pose(const Pose& pose) {
  py::array_t<double> result({4, 4});
  std::memcpy(result.mutable_data(), pose.data(), sizeof(double) * pose.size());
  return result;
}

py::dict diagnostics_dict(const Diagnostics& value) {
  py::dict result;
  result["connected"] = value.connected;
  result["running"] = value.realtime_running;
  result["watchdog_state"] = value.watchdog_state;
  result["callback_count"] = value.callback_count;
  result["accepted"] = value.accepted;
  result["rejected_sequence"] = value.rejected_sequence;
  result["rejected_source_time"] = value.rejected_source_time;
  result["rejected_source_age"] = value.rejected_source_age;
  result["rejected_target_delta"] = value.rejected_target_delta;
  result["latest_sequence"] = value.latest_sequence;
  result["latest_command_age"] = value.latest_command_age;
  result["linear_velocity"] = value.linear_velocity;
  result["angular_velocity"] = value.angular_velocity;
  result["linear_acceleration"] = value.linear_acceleration;
  result["angular_acceleration"] = value.angular_acceleration;
  result["callback_error"] = value.callback_error.empty()
                                 ? py::none()
                                 : py::cast(value.callback_error);
  return result;
}

py::dict state_dict(const RobotState& value) {
  py::dict result;
  result["connected"] = value.connected;
  result["realtime_running"] = value.realtime_running;
  result["sdk_version"] = value.sdk_version;
  result["controller_version"] = value.controller_version;
  result["robot_type"] = value.robot_type;
  result["joint_count"] = value.joint_count;
  result["power_state"] = value.power_state;
  result["operate_mode"] = value.operate_mode;
  result["operation_state"] = value.operation_state;
  result["joint_position"] = value.joint_position;
  result["joint_velocity"] = value.joint_velocity;
  result["tcp_pose"] = numpy_from_pose(value.tcp_pose);
  result["timestamp"] = value.timestamp;
  return result;
}

}  // namespace

PYBIND11_MODULE(_rokae_cpp, module) {
  module.doc() = "DexRetarget bridge for ROKAE xCoreSDK-CPP v0.3.4";
  module.attr("SDK_VERSION") = "0.3.4";

  py::class_<RealtimeConfig>(module, "RealtimeConfig")
      .def(py::init<>())
      .def_readwrite("nrt_online_speed_scale", &RealtimeConfig::nrt_online_speed_scale)
      .def_readwrite("translation_cutoff_hz", &RealtimeConfig::translation_cutoff_hz)
      .def_readwrite("rotation_cutoff_hz", &RealtimeConfig::rotation_cutoff_hz)
      .def_readwrite("translation_deadband", &RealtimeConfig::translation_deadband)
      .def_readwrite("rotation_deadband", &RealtimeConfig::rotation_deadband)
      .def_readwrite("max_translation_speed", &RealtimeConfig::max_translation_speed)
      .def_readwrite("max_angular_speed", &RealtimeConfig::max_angular_speed)
      .def_readwrite("max_translation_acceleration", &RealtimeConfig::max_translation_acceleration)
      .def_readwrite("max_angular_acceleration", &RealtimeConfig::max_angular_acceleration)
      .def_readwrite("max_translation_jerk", &RealtimeConfig::max_translation_jerk)
      .def_readwrite("max_angular_jerk", &RealtimeConfig::max_angular_jerk)
      .def_readwrite("max_target_translation_delta", &RealtimeConfig::max_target_translation_delta)
      .def_readwrite("max_target_rotation_delta", &RealtimeConfig::max_target_rotation_delta)
      .def_readwrite("workspace_min", &RealtimeConfig::workspace_min)
      .def_readwrite("workspace_max", &RealtimeConfig::workspace_max)
      .def_readwrite("hold_timeout", &RealtimeConfig::hold_timeout)
      .def_readwrite("stop_timeout", &RealtimeConfig::stop_timeout)
      .def_readwrite("max_source_age", &RealtimeConfig::max_source_age)
      .def_readwrite("future_tolerance", &RealtimeConfig::future_tolerance)
      .def_readwrite("controller_rate_limit", &RealtimeConfig::controller_rate_limit)
      .def_readwrite("controller_filter_cutoff_hz", &RealtimeConfig::controller_filter_cutoff_hz)
      .def_readwrite("rt_network_tolerance", &RealtimeConfig::rt_network_tolerance)
      .def("validate", &RealtimeConfig::validate);

  py::class_<RealtimeCore>(module, "RealtimeCore")
      .def(py::init<const RealtimeConfig&>(), py::arg("config") = RealtimeConfig{})
      .def("reset", [](RealtimeCore& self, const py::array_t<double>& pose,
                       double timestamp) {
        self.reset(pose_from_numpy(pose), timestamp);
      })
      .def("publish", [](RealtimeCore& self, const py::array_t<double>& pose,
                         double source_timestamp, std::int64_t sequence,
                         double receive_timestamp) {
        return self.publish(pose_from_numpy(pose), source_timestamp, sequence,
                            receive_timestamp);
      })
      .def("step", [](RealtimeCore& self, double timestamp) {
        return numpy_from_pose(self.step(timestamp));
      })
      .def("hold", &RealtimeCore::hold)
      .def("request_stop", &RealtimeCore::request_stop)
      .def("command_pose", [](const RealtimeCore& self) {
        return numpy_from_pose(self.command_pose());
      })
      .def("diagnostics", [](const RealtimeCore& self, double timestamp) {
        return diagnostics_dict(self.diagnostics(timestamp));
      })
      .def_property_readonly("should_finish", &RealtimeCore::should_finish);

  py::class_<RokaeDriver>(module, "RokaeDriver")
      .def(py::init<std::string, std::string, std::string,
                    const RealtimeConfig&>(),
           py::arg("robot_ip"), py::arg("local_ip"),
           py::arg("robot_type") = "xmate-er-pro-7",
           py::arg("realtime_config") = RealtimeConfig{})
      .def("connect", [](RokaeDriver& self) {
        py::gil_scoped_release release; self.connect();
      })
      .def("disconnect", [](RokaeDriver& self) {
        py::gil_scoped_release release; self.disconnect();
      })
      .def("get_state", [](RokaeDriver& self) {
        RobotState value;
        { py::gil_scoped_release release; value = self.get_state(); }
        return state_dict(value);
      })
      .def("get_joint_positions", [](RokaeDriver& self) {
        std::vector<double> value;
        { py::gil_scoped_release release; value = self.get_joint_positions(); }
        return value;
      })
      .def("get_tcp_pose", [](RokaeDriver& self) {
        Pose value;
        { py::gil_scoped_release release; value = self.get_tcp_pose(); }
        return numpy_from_pose(value);
      })
      .def("get_base_frame", [](RokaeDriver& self) {
        Pose value;
        { py::gil_scoped_release release; value = self.get_base_frame(); }
        return numpy_from_pose(value);
      })
      .def("calculate_fk", [](RokaeDriver& self,
                               const std::vector<double>& joints) {
        Pose value;
        { py::gil_scoped_release release; value = self.calculate_fk(joints); }
        return numpy_from_pose(value);
      }, py::arg("joints"))
      .def("preview_nrt_cartesian_target", [](RokaeDriver& self,
                                               const py::array_t<double>& target) {
        Pose value;
        const Pose pose = pose_from_numpy(target);
        { py::gil_scoped_release release;
          value = self.preview_nrt_cartesian_target(pose); }
        return numpy_from_pose(value);
      }, py::arg("base_T_tcp_target"))
      .def("move_joint", [](RokaeDriver& self, const std::vector<double>& target,
                            int speed, double timeout, double max_joint_delta,
                            bool power_on) {
        py::gil_scoped_release release;
        self.move_joint(target, speed, timeout, max_joint_delta, power_on);
      }, py::arg("target"), py::arg("speed") = 50,
         py::arg("timeout") = 60.0, py::arg("max_joint_delta") = 0.25,
         py::arg("power_on") = true)
      .def("move_cartesian", [](RokaeDriver& self,
                                const py::array_t<double>& target, int speed,
                                double timeout, double max_translation_delta,
                                double max_rotation_delta, bool power_on) {
        const Pose pose = pose_from_numpy(target);
        py::gil_scoped_release release;
        self.move_cartesian(pose, speed, timeout, max_translation_delta,
                            max_rotation_delta, power_on);
      }, py::arg("target"), py::arg("speed") = 20,
         py::arg("timeout") = 60.0,
         py::arg("max_translation_delta") = 0.05,
         py::arg("max_rotation_delta") = 0.20,
         py::arg("power_on") = true)
      .def("move_to_init", [](RokaeDriver& self,
                              const std::vector<double>& target, int speed,
                              double timeout, double max_joint_delta,
                              bool power_on) {
        py::gil_scoped_release release;
        self.move_to_init(target, speed, timeout, max_joint_delta, power_on);
      }, py::arg("target"), py::arg("speed") = 50,
         py::arg("timeout") = 60.0, py::arg("max_joint_delta") = 1.2,
         py::arg("power_on") = true)
      .def("start_realtime_cartesian", [](RokaeDriver& self, bool power_on) {
        py::gil_scoped_release release; self.start_realtime_cartesian(power_on);
      }, py::arg("power_on") = true)
      .def("set_target_pose", [](RokaeDriver& self,
                                 const py::array_t<double>& target,
                                 double source_timestamp,
                                 std::int64_t sequence) {
        return self.set_target_pose(pose_from_numpy(target), source_timestamp,
                                    sequence);
      }, py::arg("target"), py::arg("source_timestamp"), py::arg("sequence"))
      .def("hold", &RokaeDriver::hold)
      .def("stop_realtime", [](RokaeDriver& self) {
        py::gil_scoped_release release; self.stop_realtime();
      })
      .def("stop", [](RokaeDriver& self) {
        py::gil_scoped_release release; self.stop();
      })
      .def("get_command_pose", [](const RokaeDriver& self) {
        return numpy_from_pose(self.get_command_pose());
      })
      .def("diagnostics", [](const RokaeDriver& self) {
        return diagnostics_dict(self.diagnostics());
      });

  module.def("monotonic_time", &RokaeDriver::monotonic_time);
}
