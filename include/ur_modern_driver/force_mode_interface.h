#ifndef FORCE_MODE_INTERFACE_H
#define FORCE_MODE_INTERFACE_H

#include <string>
#include <cassert>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface {

class ForceModeHandle {
public:
	ForceModeHandle() : name_(), cmd_compliance_(0), cmd_force_(0) {}

	ForceModeHandle(const std::string & name, int * compliance, double * force) :
					name_(name),
					cmd_compliance_(compliance),
					cmd_force_(force)
	{}

	void setCommand(int cmd_compliance, double cmd_force) {
		setCommandCompliance(cmd_compliance);
		setCommandForce(cmd_force);
	}

	void setCommandCompliance(int cmd_compliance) {assert(cmd_compliance_); *cmd_compliance_ = cmd_compliance;}
	void setCommandForce(double cmd_force) {assert(cmd_force_); *cmd_force_ = cmd_force;}

	std::string getName() const {return name_;}

	int getCommandCompliance() const {assert(cmd_compliance_); return * cmd_compliance_;}
	double getCommandForce() const {assert(cmd_force_); return * cmd_compliance_;}

private:
	std::string name_;
	int * cmd_compliance_;
	double * cmd_force_;
};

class ForceModeInterface : public HardwareResourceManager<ForceModeHandle, ClaimResources> {};

} // hardware interface namespace

#endif
