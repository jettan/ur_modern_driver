#ifndef FORCE_MODE_CONTROLLER_H
#define FORCE_MODE_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ur_modern_driver/force_mode_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>

#include <boost/shared_ptr.hpp>

namespace force_mode_controllers {

/// \brief Custom interface to handle joint_position commands on the UR while in force mode.
class ForceModeController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::ForceModeInterface> {
public:
	ForceModeController();

	virtual bool init(hardware_interface::RobotHW * hw, ros::NodeHandle & n);

	virtual void starting(const ros::Time & time);
	virtual void update(const ros::Time & time, const ros::Duration & period);
	virtual void stopping(const ros::Time & time);

	std::vector<hardware_interface::JointHandle> joints_;
	std::vector<hardware_interface::ForceModeHandle> entries_;
	realtime_tools::RealtimeBuffer<std::tuple<int,double>> command_buffer_;
};

} // namespace

#endif
