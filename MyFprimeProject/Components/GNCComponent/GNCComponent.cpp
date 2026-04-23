// ======================================================================
// \title  GNCComponent.cpp
// \author user
// \brief  cpp file for GNCComponent component implementation class
// ======================================================================

#include "MyFprimeProject/Components/GNCComponent/GNCComponent.hpp"
#include <atomic>
#include <memory>
#include "Fw/Prm/ParamValidEnumAc.hpp"
#include "Os/IntervalTimer.hpp"

namespace MyFprimeProject {

std::atomic<U32> total_correction_memory{0};

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

GNCComponent ::GNCComponent(const char* const compName) : GNCComponentComponentBase(compName) {
    this->timer = std::make_unique<Os::IntervalTimer>();
}

GNCComponent ::~GNCComponent() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void GNCComponent ::positionIn_handler(FwIndexType portNum, const MyFprimeProject::Position& position) {
    this->m_current_position = position;
    this->tlmWrite_TlmCurrentPosition(this->m_current_position.to_position());
}

void GNCComponent ::rotationIn_handler(FwIndexType portNum, const MyFprimeProject::Rotation& rotation) {
    this->m_current_rotation = rotation;
    this->tlmWrite_TlmCurrentRotation(this->m_current_rotation.to_rotation());
}

void GNCComponent ::velocityIn_handler(FwIndexType portNum, const MyFprimeProject::Velocity& velocity) {
    this->m_current_velocity = velocity;
    this->tlmWrite_TlmCurrentVelocity(this->m_current_velocity.to_velocity());
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void GNCComponent ::SET_ROTATION_cmdHandler(FwOpcodeType opCode,
                                            U32 cmdSeq,
                                            MyFprimeProject::Rotation target_rotation) {
    this->m_target_rotation = target_rotation;
    Rotation new_rotation = this->m_target_rotation.to_rotation();

    this->m_is_target_rotation_set = true;

    this->tlmWrite_TlmTargetRotation(new_rotation);
    this->log_ACTIVITY_HI_RotationEvent(new_rotation);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GNCComponent ::SET_POSITION_cmdHandler(FwOpcodeType opCode,
                                            U32 cmdSeq,
                                            MyFprimeProject::Position target_position) {
    this->m_target_position = target_position;
    Position new_position = this->m_target_position.to_position();

    this->m_is_target_position_set = true;

    this->tlmWrite_TlmTargetPosition(new_position);
    this->log_ACTIVITY_HI_PositionEvent(new_position);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GNCComponent ::SET_VELOCITY_cmdHandler(FwOpcodeType opCode,
                                            U32 cmdSeq,
                                            MyFprimeProject::Velocity target_velocity) {
    this->m_target_velocity = target_velocity;
    Velocity new_velocity = this->m_target_velocity.to_velocity();

    this->m_is_target_velocity_set = true;
    
    this->tlmWrite_TlmTargetVelocity(new_velocity);
    this->log_ACTIVITY_HI_VelocityEvent(new_velocity);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GNCComponent ::schedIn_handler(FwIndexType portNum, U32 context) {
    // TODO Implement GNC loop step
    if (this->dt.get_safe(0) != 0) {
        this->timer->stop();
        this->dt = this->timer->getDiffUsec();
    }
    
    if (this->m_current_rotation != this->m_target_rotation && this->m_is_target_rotation_set) {
        rotate_to_target();
    }

    if (this->m_current_velocity != this->m_target_velocity && this->m_is_target_velocity_set) {
        velocity_to_target();
    }

    if (this->m_current_angle_velocity != this->m_target_angle_velocity && this->m_is_target_angle_velocity_set) {
        angle_velocity_to_target();
    }

    if (this->m_current_position != this->m_target_position && this->m_is_target_position_set) {
        position_to_target();
    }

    this->timer->start();
}

void GNCComponent ::position_to_target() {
    
}

void GNCComponent ::rotate_to_target() {

}

void GNCComponent ::velocity_to_target() {

}

void GNCComponent ::angle_velocity_to_target() {
    Fw::ParamValid valid;
    F32 kp = this->paramGet_VEL_Kp(valid);
    F32 ki = this->paramGet_VEL_Ki(valid);
    F32 kd = this->paramGet_VEL_Kd(valid);

    ProtectedNumber<F32> error = ProtectedVector3<F32>::distance(
        this->m_target_angle_velocity,
        this->m_current_angle_velocity
    );

    this->m_integral += error * this->dt;
}

}  // namespace MyFprimeProject
