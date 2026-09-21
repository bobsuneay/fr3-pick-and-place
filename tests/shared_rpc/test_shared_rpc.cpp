#include <cassert>
#include <iostream>
#include <limits>
#include "fairino_hardware/fairino_hardware_interface.hpp"
#include "fairino_hardware/fairino_gripper_hardware_interface.hpp"
using namespace fairino_hardware;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

hardware_interface::HardwareInfo info(int count, const std::string & ip="192.0.2.5") {
    hardware_interface::HardwareInfo result;
    result.hardware_parameters={{"robot_ip",ip},{"gripper_index","1"},{"block","1"}};
    for(int i=0;i<count;++i) result.joints.push_back(
        {"j"+std::to_string(i), {{"position"}}, {{"position"}}});
    return result;
}
void order_test(bool gripper_first, bool release_gripper_first) {
    FRRobot::reset();
    {
        FairinoHardwareInterface arm;
        FairinoGripperHardwareInterface gripper;
        assert(arm.on_init(info(6))==CallbackReturn::SUCCESS);
        assert(gripper.on_init(info(1))==CallbackReturn::SUCCESS);
        if(gripper_first) {
            assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
            assert(arm.on_activate({})==CallbackReturn::SUCCESS);
        } else {
            assert(arm.on_activate({})==CallbackReturn::SUCCESS);
            assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
        }
        assert(FRRobot::rpc_calls==1);
        assert(FRRobot::move_calls==0); // No open/close command during activation.
        assert(arm.write({},{})==return_type::OK);
        for(int i=0;i<6;++i) assert(std::abs(FRRobot::last_command.jPos[i]-(10+i))<1e-9);
        assert(gripper.write({},{})==return_type::OK);
        assert(FRRobot::move_calls==0); // Hold actual gripper feedback at first write.
        auto commands=gripper.export_command_interfaces();
        *commands[0].value=0.03;
        assert(gripper.write({},{})==return_type::OK);
        assert(FRRobot::move_calls==1 && FRRobot::move_percent==0 && FRRobot::move_block==1);
        assert(gripper.write({},{})==return_type::OK && FRRobot::move_calls==1);
        *commands[0].value=0;
        assert(gripper.write({},{})==return_type::OK && FRRobot::move_percent==100);
        FRRobot::feedback_percent=50;
        assert(gripper.read({},{})==return_type::OK);
        assert(std::abs(*gripper.export_state_interfaces()[0].value-0.015)<1e-9);
        if(release_gripper_first) {
            gripper.on_deactivate({});
            assert(FRRobot::close_calls==0 && arm.read({},{})==return_type::OK);
            arm.on_deactivate({});
        } else {
            arm.on_deactivate({});
            assert(FRRobot::close_calls==0 && gripper.read({},{})==return_type::OK);
            gripper.on_deactivate({});
        }
        assert(FRRobot::close_calls==1 && FRRobot::live==0);
    }
    assert(FRRobot::close_calls==1);
}
int main() {
    for(bool first:{false,true}) for(bool release:{false,true}) order_test(first,release);
    FRRobot::reset();
    {
        FairinoHardwareInterface arm;
        FairinoGripperHardwareInterface gripper;
        arm.on_init(info(6)); gripper.on_init(info(1));
        assert(arm.on_activate({})==CallbackReturn::SUCCESS);
        FRRobot::activation_error=13;
        assert(gripper.on_activate({})==CallbackReturn::ERROR);
        assert(FRRobot::close_calls==0 && arm.read({},{})==return_type::OK);
        FRRobot::activation_error=0;
        FRRobot::feedback_error=11;
        assert(gripper.on_activate({})==CallbackReturn::ERROR); // Do not fake an open position.
        assert(FRRobot::close_calls==0);
        assert(arm.read({},{})==return_type::ERROR);
        FRRobot::feedback_error=0;
        assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
        assert(FRRobot::rpc_calls==1);
        FRRobot::feedback_percent=101;
        assert(gripper.read({},{})==return_type::ERROR);
        FRRobot::feedback_percent=25;
        FRRobot::servo_error=7;
        assert(arm.write({},{})==return_type::ERROR);
        FRRobot::servo_error=0;
        auto arm_commands=arm.export_command_interfaces();
        *arm_commands[5].value=std::numeric_limits<double>::quiet_NaN();
        assert(arm.write({},{})==return_type::ERROR); // Sixth joint is also validated.
        arm.on_deactivate({}); gripper.on_deactivate({});
        assert(FRRobot::close_calls==1);
        assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
        assert(arm.on_activate({})==CallbackReturn::SUCCESS);
        assert(FRRobot::rpc_calls==2);
    }
    assert(FRRobot::live==0 && FRRobot::close_calls==2);
    FRRobot::reset();
    {
        FairinoHardwareInterface arm; arm.on_init(info(6));
        FRRobot::rpc_error=9;
        assert(arm.on_activate({})==CallbackReturn::ERROR);
        FRRobot::rpc_error=0;
        assert(arm.on_activate({})==CallbackReturn::SUCCESS);
        assert(FRRobot::rpc_calls==2);
        FairinoGripperHardwareInterface gripper; gripper.on_init(info(1,"192.0.2.2"));
        assert(gripper.on_activate({})==CallbackReturn::ERROR);
        assert(FRRobot::rpc_calls==2); // Wrong process/IP must not start another SDK.
    }
    assert(FRRobot::live==0);
    FRRobot::reset();
    {
        FairinoHardwareInterface arm; arm.on_init(info(6));
        FRRobot::feedback_error=11;
        assert(arm.on_activate({})==CallbackReturn::ERROR);
        assert(FRRobot::live==0 && FRRobot::close_calls==1);
        assert(arm.write({},{})==return_type::ERROR);
        FRRobot::feedback_error=0;
        FairinoGripperHardwareInterface gripper; gripper.on_init(info(1));
        assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
        FRRobot::feedback_error=11;
        assert(arm.on_activate({})==CallbackReturn::ERROR);
        assert(FRRobot::live==1 && FRRobot::close_calls==1);
        FRRobot::feedback_error=0;
        assert(gripper.read({},{})==return_type::OK);
        assert(arm.on_activate({})==CallbackReturn::SUCCESS);
        assert(FRRobot::rpc_calls==2);
    }
    assert(FRRobot::live==0 && FRRobot::close_calls==2);
    {
        FairinoHardwareInterface arm;
        assert(arm.on_init(info(7))==CallbackReturn::ERROR);
    }
    {
        FairinoGripperHardwareInterface gripper;
        assert(gripper.on_init(info(2))==CallbackReturn::ERROR); // mimic is not commanded
        auto config=info(1);
        config.hardware_parameters["block"]="0";
        assert(gripper.on_init(config)==CallbackReturn::ERROR);
        config.hardware_parameters["block"]="1";
        config.joints[0].state_interfaces.push_back({"velocity"});
        assert(gripper.on_init(config)==CallbackReturn::SUCCESS);
        FRRobot::feedback_percent=25;
        assert(gripper.on_activate({})==CallbackReturn::SUCCESS);
        auto states=gripper.export_state_interfaces();
        FRRobot::feedback_percent=50;
        assert(gripper.read({},{})==return_type::OK);
        assert(std::abs(*states[1].value - (-0.0075/0.008))<1e-9);
        assert(gripper.read({},{})==return_type::OK && *states[1].value==0.0);
        FRRobot::feedback_error=5;
        assert(gripper.read({},{})==return_type::ERROR);
        FRRobot::feedback_error=0;
        auto commands=gripper.export_command_interfaces();
        const int calls=FRRobot::move_calls;
        for(double invalid : {-0.001, 0.04, std::numeric_limits<double>::quiet_NaN()}) {
            *commands[0].value=invalid;
            assert(gripper.write({},{})==return_type::ERROR);
            assert(FRRobot::move_calls==calls);
        }
    }
    std::cout << "PASS: both activation/release orders, one RPC, reactivation, failure cleanup, commands/feedback\n";
}
