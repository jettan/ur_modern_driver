#include <pluginlib/class_list_macros.h>
#include <ur_modern_driver/force_mode_controller.h>

namespace force_mode_controllers {

// Internal helper functions copied from joint_trajectory_controller.
namespace internal {

std::vector<std::string> getStrings(const ros::NodeHandle & nh, const std::string & param_name) {
	using namespace XmlRpc;

	XmlRpcValue xml_array;
	if (!nh.getParam(param_name, xml_array)) {
		ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
		return std::vector<std::string>();
	}
	if (xml_array.getType() != XmlRpcValue::TypeArray) {
		ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
			nh.getNamespace() << ").");
		return std::vector<std::string>();
	}

	std::vector<std::string> out;
	for (int i = 0; i < xml_array.size(); ++i) {
		if (xml_array[i].getType() != XmlRpcValue::TypeString) {
			ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
				nh.getNamespace() << ").");
			return std::vector<std::string>();
		}
		out.push_back(static_cast<std::string>(xml_array[i]));
	}
	return out;
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle & nh, const std::string & param_name) {
	urdf::ModelSharedPtr urdf(new urdf::Model);

	std::string urdf_str;
	// Check for robot_description in proper namespace
	if (nh.getParam(param_name, urdf_str)) {
		if (!urdf->initString(urdf_str)) {
			ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
				nh.getNamespace() << ").");
			return urdf::ModelSharedPtr();
		}
	} else if (!urdf->initParam("robot_description")) {
		ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
		return urdf::ModelSharedPtr();
	}
	return urdf;
}

std::vector<urdf::JointConstSharedPtr> getUrdfJoints(const urdf::Model & urdf, const std::vector<std::string> & joint_names) {
	std::vector<urdf::JointConstSharedPtr> out;
	for (unsigned int i = 0; i < joint_names.size(); ++i) {
		urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_names[i]);
		if (urdf_joint) {
			out.push_back(urdf_joint);
		} else {
			ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
			return std::vector<urdf::JointConstSharedPtr>();
		}
	}
	return out;
}

std::string getLeafNamespace(const ros::NodeHandle & nh) {
	const std::string complete_ns = nh.getNamespace();
	std::size_t id   = complete_ns.find_last_of("/");
	return complete_ns.substr(id + 1);
}

} // internal namespace

ForceModeController::ForceModeController() : distribution_(-1.0, 1.0), steps_per_trial_(5), controller_step_length_(125) {}

bool ForceModeController::init(hardware_interface::RobotHW * hw, ros::NodeHandle & root_nh, ros::NodeHandle & controller_nh) {
	// Cache the controller node handle.
	controller_nh_ = controller_nh;

	// Obtain the given name of the controller.
	name_ = internal::getLeafNamespace(controller_nh_);

	// Initialize joint names and number of joints.
	joint_names_ = internal::getStrings(controller_nh_, "joints");
	if (joint_names_.empty()) {
		return false;
	}
	const unsigned int n_joints = joint_names_.size();

	urdf::ModelSharedPtr urdf = internal::getUrdf(root_nh, "robot_description");
	if (!urdf) {
		return false;
	}

	std::vector<urdf::JointConstSharedPtr> urdf_joints = internal::getUrdfJoints(*urdf, joint_names_);
	if (urdf_joints.empty()) {
		return false;
	}
	assert(n_joints == urdf_joints.size());

	joints_.resize(n_joints);
	entries_.resize(6);
	position_command_.resize(n_joints);
	force_command_.resize(6);
	compliance_command_.resize(6);

	position_command_active_ = false;

	// Obtain pointers to the wanted interfaces.
	hardware_interface::PositionJointInterface * p = hw->get<hardware_interface::PositionJointInterface>();
	hardware_interface::ForceModeInterface * f     = hw->get<hardware_interface::ForceModeInterface>();

	// Claim joint handles.
	for (unsigned int i = 0; i < n_joints; ++i) {
		try {
			joints_[i] = p->getHandle(joint_names_[i]);
		} catch (...) {
			ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in 'PositionJointInterface'.");
			return false;
		}
	}

	// Claim force mode handles.
	for (unsigned int i = 0; i < 6; ++i) {
		try {
			entries_[i] = f->getHandle(force_mode_resources_[i]);
		} catch (...) {
			ROS_ERROR_STREAM_NAMED(name_, "Could not find force mode resource '" << force_mode_resources_[i] << "' in 'ForceModeInterface'.");
			return false;
		}
	}

	// Subscribe on ~command topic.
	position_command_subscriber_ = controller_nh_.subscribe("command", 1, &ForceModeController::positionCommandCB, this);
	action_command_subscriber_ = controller_nh_.subscribe("action", 125, &ForceModeController::actionCommandCB, this);

	return true;
}

void ForceModeController::starting(const ros::Time & time) {
	// Initialize the realtime buffer.
	TimeData time_data;
	time_data.time = time;
	time_data.uptime = ros::Time(0.0);
	time_buffer_.initRT(time_data);

	controller_counter_ = 0;
	step_counter_       = 0;

	// Semantic zero: set the position command to the current position for each joint.
	for (unsigned int i = 0; i < joints_.size(); ++i) {
		position_command_[i] = joints_[i].getPosition();
		joints_[i].setCommand(position_command_[i]);
	}

	// Set all forces/compliance to 0.
	for (unsigned int i = 0; i < entries_.size(); ++i) {
		compliance_command_[i] = 0;
		force_command_[i]      = 0.0;
		entries_[i].setCommand(compliance_command_[i], force_command_[i]);
	}
}

void ForceModeController::update(const ros::Time & time, const ros::Duration & period) {
	// Update the real time buffer.
	TimeData time_data;
	time_data.time   = time;
	time_data.period = period;
	time_data.uptime = time_buffer_.readFromRT()->uptime + period;
	time_buffer_.writeFromNonRT(time_data);

	// Get the state of the robot.
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = time;

	joint_state.name.resize(joint_names_.size());
	joint_state.position.resize(joints_.size());
	joint_state.velocity.resize(joints_.size());

	joint_state.name = joint_names_;

	for (unsigned int i = 0; i < joints_.size(); ++i) {
		joint_state.position[i] = joints_[i].getPosition();
		joint_state.velocity[i] = joints_[i].getVelocity();
	}

	// Bump the counter.
	controller_counter_ = (controller_counter_+ 1) % controller_step_length_;

	// Only perform controller step if the counter equals 0 to downsample controller updates.
	if (!controller_counter_) {
		updateControllers(time, compliance_command_, force_command_);
	}

	// Make the robot do the action.
	for (unsigned int i = 0; i < joints_.size(); ++i) {
		if (position_command_active_) {
			joints_[i].setCommand(position_command_[i]);
		} else {
			joints_[i].setCommand(joints_[i].getPosition());
		}
	}

	for (unsigned int i = 0; i < entries_.size(); ++i) {
		entries_[i].setCommand(compliance_command_[i], force_command_[i]);
	}
}

void ForceModeController::stopping(const ros::Time & /*time*/) {}

// FIXME: Check for joint names/ordering etc.
void ForceModeController::positionCommandCB(const JointTrajectoryConstPtr & msg) {
	ROS_INFO_STREAM("Received position command.");
	position_command_active_ = true;
	for (unsigned int i = 0; i < joints_.size(); ++i) {
		position_command_[i] = msg->points[0].positions[i];
	}
}

// Update forces/compliance on receiving.
void ForceModeController::actionCommandCB(const ForceModeActionConstPtr & msg) {
	ROS_INFO_STREAM("Updating force mode action to id: " << msg->id);
	for (unsigned int i = 0; i < 6; ++i) {
		compliance_command_[i] = msg->compliances[i];
		force_command_[i]      = msg->forces[i];
	}
}

} // namespace

PLUGINLIB_EXPORT_CLASS(force_mode_controllers::ForceModeController, controller_interface::ControllerBase)
