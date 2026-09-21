#pragma once
#include <cstdint>
using errno_t = int;
using error_t = int;
struct JointPos { double jPos[6]; };
struct ExaxisPos { double ePos[4]; };
class FRRobot {
    bool live_ = false;
public:
    inline static int rpc_calls=0, close_calls=0, live=0, servo_calls=0, move_calls=0;
    inline static int rpc_error=0, activation_error=0, feedback_error=0, servo_error=0;
    inline static int feedback_percent=25, move_percent=-1, move_block=-1;
    inline static JointPos last_command{};
    static void reset() {
        rpc_calls=close_calls=live=servo_calls=move_calls=0;
        rpc_error=activation_error=feedback_error=servo_error=0;
        feedback_percent=25; move_percent=move_block=-1;
    }
    int RPC(const char*) {
        ++rpc_calls;
        if (rpc_error) return rpc_error;
        if (live) return 42; // Reproduce second-connection failure in one SDK process.
        live_=true; ++live; return 0;
    }
    int CloseRPC() {
        ++close_calls;
        if (live_) { live_=false; --live; }
        return 0;
    }
    int StopMotion() { return 0; }
    int GetActualJointPosDegree(int, JointPos * p) {
        for (int i=0; i<6; ++i) p->jPos[i]=10.0+i;
        return feedback_error;
    }
    int ServoJ(JointPos * p, ExaxisPos*, int, int, double, int, int) {
        ++servo_calls; last_command=*p; return servo_error;
    }
    int ActGripper(int, uint8_t) { return activation_error; }
    int GetGripperCurPosition(uint16_t * f, uint8_t * p) {
        *f=0; *p=static_cast<uint8_t>(feedback_percent); return feedback_error;
    }
    int MoveGripper(int, int p, int, int, int, uint8_t b, int, double, int, int) {
        ++move_calls; move_percent=p; move_block=b; return 0;
    }
};
