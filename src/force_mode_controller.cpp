#include <ur_modern_driver/force_mode_controller.h>

namespace force_mode_controllers {

bool ForceModeController::init(hardware_interface::RobotHW * hw, ros::NodeHandle & n) {
		hardware_interface::PositionJointInterface * p = hw->get<hardware_interface::PositionJointInterface>();
		hardware_interface::ForceModeInterface * f     = hw->get<hardware_interface::ForceModeInterface>();

		// TODO: Fetch resources from interfaces and perform rest of initialization...
		ROS_WARN_STREAM("Initialized force mode controller.");
}

// Call pluginlib macro to export the class.
PLUGINLIB_EXPORT_CLASS(force_mode_controllers::ForceModeController, controller_interface::ControllerBase)

} // namespace

