// ======================================================================
// \title  GNCComponent.cpp
// \author user
// \brief  cpp file for GNCComponent component implementation class
// ======================================================================

#include "MyFprimeProject/Components/GNCComponent/GNCComponent.hpp"
#include "MyFprimeProject/Components/GNCComponent/GncModeEnumAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/WayPointSerializableAc.hpp"

namespace MyFprimeProject {

std::atomic<U32> current_mission_time_ms(0);
std::atomic<U32> total_correction_memory(0);

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

GNCComponent ::GNCComponent(const char* const compName) : GNCComponentComponentBase(compName) {}

GNCComponent ::~GNCComponent() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void GNCComponent ::GyroDataIn_handler(FwIndexType portNum, const MyFprimeProject::GyroData& data) {
    // TODO

    m_current_angular_velocity = data;
}

void GNCComponent ::StarTrackerDataIn_handler(FwIndexType portNum, const MyFprimeProject::OrientData& data) {
    // TODO
}

void GNCComponent ::SunPanelsNormalIn_handler(FwIndexType portNum, const MyFprimeProject::Vec3& normal) {
    // TODO
}

void GNCComponent ::SunVectorIn_handler(FwIndexType portNum, const MyFprimeProject::Vec3& position) {
    // TODO
}

void GNCComponent ::schedIn_handler(FwIndexType portNum, U32 context) {
    // TODO

    this->tlmWrite_TlmECCurrentMemoryCorrections(total_correction_memory);

    WayPoint current_point = this->make_current_way_point();
    this->tlmWrite_TlmCurrentWayPoint(current_point);
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void GNCComponent ::SET_TARGET_WAYPOINT_cmdHandler(FwOpcodeType opCode,
                                                U32 cmdSeq,
                                                MyFprimeProject::WayPoint target_state) {
    // TODO
    m_target_waypoint = target_state;

    this->tlmWrite_TlmTargetWayPoint(target_state);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GNCComponent ::SET_MODE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, MyFprimeProject::GncMode mode) {
    // TODO
    m_gnc_mode = mode;

    this->tlmWrite_TlmCurrentGncMode(mode);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace MyFprimeProject
