// ======================================================================
// \title  GNCComponent.cpp
// \author user
// \brief  cpp file for GNCComponent component implementation class
// ======================================================================

#include "MyFprimeProject/Components/GNCComponent/GNCComponent.hpp"

namespace MyFprimeProject {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

GNCComponent ::GNCComponent(const char* const compName) : GNCComponentComponentBase(compName) {}

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

    if (this->m_current_rotation != this->m_target_rotation) {
        rotate_to_target();
    }

    if (this->m_current_velocity != this->m_target_velocity) {
        velocity_to_target();
    }

    if (this->m_current_angle_velocity != this->m_target_angle_velocity) {
        angle_velocity_to_target();
    }

    if (this->m_current_position != this->m_target_position) {
        position_to_target();
    }
}

void GNCComponent ::position_to_target() {
    
}

void GNCComponent ::rotate_to_target() {

}

void GNCComponent ::velocity_to_target() {

}

void GNCComponent ::angle_velocity_to_target() {

}

}  // namespace MyFprimeProject
