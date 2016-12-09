#ifndef FORCE_MODE_CONTROLLER_H
#define FORCE_MODE_CONTROLLER_H

#include <cassert>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/node_handle.h>
#include <urdf/model.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>

#include <hardware_interface/joint_command_interface.h>
#include <ur_modern_driver/force_mode_interface.h>

namespace force_mode_controllers {

/// \brief Custom interface to handle joint_position commands on the UR while in force mode.
class ForceModeController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::ForceModeInterface> {
public:
	/// Constructor.
	ForceModeController();

	virtual bool init(hardware_interface::RobotHW * hw, ros::NodeHandle & root_nh, ros::NodeHandle & controller_nh);

	virtual void starting(const ros::Time & time);
	virtual void stopping(const ros::Time & /*time*/);
	virtual void update(const ros::Time & time, const ros::Duration & period);

private:
	typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> StatePublisher;
	typedef boost::scoped_ptr<StatePublisher>                                               StatePublisherPtr;
	typedef trajectory_msgs::JointTrajectory::ConstPtr                                      JointTrajectoryConstPtr;

	/// The node handle of this controller.
	ros::NodeHandle controller_nh_;

	/// Subscriber to receive an individual position command to reset the position of the robot.
	ros::Subscriber position_command_subscriber_;

	/// Controller state publisher.
	StatePublisher state_publisher_;

	/// The period to publish the state of this controller.
	ros::Duration state_publisher_period_;

	/// The name of this controller.
	std::string name_;

	/// Names of the controlled joints.
	std::vector<std::string> joint_names_;

	// FIXME: Read resource names from parameter server.
	/// Names of the force mode resources.
	std::vector<std::string> force_mode_resources_ = {"x", "y", "z", "roll", "pitch", "yaw"};

	/// Handles to the controlled joints.
	std::vector<hardware_interface::JointHandle> joints_;

	/// Handles to the controlled positional/rotational axes for force mode.
	std::vector<hardware_interface::ForceModeHandle> entries_;

	/// The command buffer.
	realtime_tools::RealtimeBuffer<std::tuple<int,double>> command_buffer_;

	/// Callback for when the position command subscriber receives a new command.
	void positionCommandCB(const JointTrajectoryConstPtr & msg);
};

} // namespace

#endif
