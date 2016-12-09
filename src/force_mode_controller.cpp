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

ForceModeController::ForceModeController() {}

bool ForceModeController::init(hardware_interface::RobotHW * hw, ros::NodeHandle & root_nh, ros::NodeHandle & controller_nh) {
	// Cache the controller node handle.
	controller_nh_ = controller_nh;

	// Obtain the given name of the controller.
	name_ = internal::getLeafNamespace(controller_nh_);

	// Obtain the publishing rate of the controller's state.
	double state_publish_rate = 125.0;
	controller_nh_.getParam("state_publish_rate", state_publish_rate);
	ROS_INFO_STREAM("Controller state will be published at " << state_publish_rate << "Hz.");
	state_publisher_period_  = ros::Duration(1.0 / state_publish_rate);

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
		}
	}
	return true;
}

void ForceModeController::starting(const ros::Time & time) {
	// Semantic zero: set the position command to the current position for each joint.
	for (unsigned int i = 0; i < joints_.size(); ++i) {
		joints_[i].setCommand(joints_[i].getPosition());
	}

	// Set all forces/compliance to 0.
	for (unsigned int i = 0; i < entries_.size(); ++i) {
		entries_[i].setCommand(0, 0);
	}
}

void ForceModeController::update(const ros::Time & time, const ros::Duration & period) {
	for (unsigned int i = 0; i < joints_.size(); ++i) {
		joints_[i].setCommand(joints_[i].getPosition());
	}

	for (unsigned int i = 0; i < entries_.size(); ++i) {
		entries_[i].setCommand(0, 0);
	}
}

void ForceModeController::stopping(const ros::Time & /*time*/) {}

} // namespace

PLUGINLIB_EXPORT_CLASS(force_mode_controllers::ForceModeController, controller_interface::ControllerBase)
