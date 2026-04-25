// ======================================================================
// \title  GNCComponent.cpp
// \author user
// \brief  cpp file for GNCComponent component implementation class
// ======================================================================

#include "MyFprimeProject/Components/GNCComponent/GNCComponent.hpp"
#include "Fw/Prm/ParamValidEnumAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/GncModeEnumAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/GyroDataSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/OrientDataSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/QuaternionSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/Vec3SerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/WayPointSerializableAc.hpp"

namespace MyFprimeProject {

std::atomic<U32> total_correction_memory(0);

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

GNCComponent ::GNCComponent(const char* const compName) : GNCComponentComponentBase(compName) {
    Fw::ParamValid valid;
    F32 alpha = this->paramGet_EMA_Alpha(valid);
    if (valid == Fw::ParamValid::VALID) {
        m_ema_filter = EMA(alpha);
    }
}

GNCComponent ::~GNCComponent() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void GNCComponent ::GyroDataIn_handler(FwIndexType portNum, const MyFprimeProject::GyroData& data, bool is_valid) {
    if (is_valid) {
        ProtectedObject<EMA>::Status status;
        EMA ema = m_ema_filter.get(EMA(0.1f), status);
        if (status == ProtectedObject<EMA>::Status::CORRUPTED) {
            Fw::ParamValid valid;
            F32 alpha = this->paramGet_EMA_Alpha(valid);
            if (valid == Fw::ParamValid::INVALID) alpha = 0.1f;

            m_ema_filter = EMA(alpha);
        }

        Vec3 angular_rates;
        F32 x, y, z;

        x = ema.process(data.get_angular_rates().get_x());
        y = ema.process(data.get_angular_rates().get_y());
        z = ema.process(data.get_angular_rates().get_z());

        angular_rates.set(x, y, z);

        m_current_angular_velocity = GyroData(
            angular_rates,
            data.get_timestamp()
        );
        m_current_angular_vel_valid = true;
    } else {
        m_current_angular_vel_valid = false;
    }
}

void GNCComponent ::StarTrackerDataIn_handler(FwIndexType portNum, const MyFprimeProject::OrientData& data, bool is_valid) {
    if (is_valid) {
        ProtectedObject<EMA>::Status status;
        EMA ema = m_ema_filter.get(EMA(0.1f), status);
        if (status == ProtectedObject<EMA>::Status::CORRUPTED) {
            Fw::ParamValid valid;
            F32 alpha = this->paramGet_EMA_Alpha(valid);
            if (valid == Fw::ParamValid::INVALID) alpha = 0.1f;

            m_ema_filter = EMA(alpha);
        }

        Quaternion quaternion;
        F32 w, x, y, z;

        w = ema.process(data.get_orientation().get_w());
        x = ema.process(data.get_orientation().get_x());
        y = ema.process(data.get_orientation().get_y());
        z = ema.process(data.get_orientation().get_z());

        quaternion.set(w, x, y, z);

        m_current_orientation = OrientData(
            quaternion,
            data.get_timestamp()
        );
        m_current_orient_valid = true;
    } else {
        m_current_orient_valid = false;
    }
}

void GNCComponent ::SunPanelsNormalIn_handler(FwIndexType portNum, const MyFprimeProject::Vec3& normal) {
    m_current_sun_panels_normal = normal;
}

void GNCComponent ::SunVectorIn_handler(FwIndexType portNum, const MyFprimeProject::Vec3& position) {
    Vec3 p = position;
    this->vec_normalize(p);
    m_current_sun_normal = p;
}

void GNCComponent ::BatteryStateIn_handler(FwIndexType portNum, const BatteryState& battery_state) {
    m_current_battery_state = battery_state;

    Fw::ParamValid valid;
    F32 min_soc = this->paramGet_MinBatterySoC(valid);
    if (Fw::ParamValid::VALID) {
        if (battery_state.get_SoC() < min_soc) {
            m_gnc_mode = GncMode::SAFE;

            this->tlmWrite_TlmCurrentGncMode(GncMode::SAFE);
        }
    }
}

void GNCComponent ::MagnesticData_handler(FwIndexType portNum, const MyFprimeProject::Vec3& data) {
    m_prev_b_field = m_current_b_field;
    Vec3 new_b_field;
    ProtectedObject<EMA>::Status status;
    EMA ema = m_ema_filter.get(EMA(), status);
    if (status == ProtectedObject<EMA>::Status::CORRUPTED) {
        new_b_field = data;
        m_current_b_field = new_b_field;
        return;
    }

    new_b_field.set(
        ema.process(data.get_x()),
        ema.process(data.get_y()),
        ema.process(data.get_z())
    );
    m_current_b_field = new_b_field;
}

void GNCComponent ::ReactionWheelsRPMData_handler(FwIndexType portNum, const MyFprimeProject::Vec3& data) {
    m_current_reaction_wheels_rpm = data;
}

void GNCComponent ::schedIn_handler(FwIndexType portNum, U32 context) {
    m_dt_ms = this->to_ms(this->getTime()) - m_last_ms;

    this->tlmWrite_TlmECCurrentMemoryCorrections(total_correction_memory);

    WayPoint current_point = this->make_current_way_point();
    this->tlmWrite_TlmCurrentWayPoint(current_point);

    ProtectedObject<GncMode>::Status status;
    GncMode gnc_mode = m_gnc_mode.get(GncMode(GncMode::SAFE), status);
    if (status == ProtectedObject<GncMode>::Status::CORRUPTED) {
        m_gnc_mode = GncMode::SAFE;
        gnc_mode = GncMode::SAFE;
    }

    this->tlmWrite_TlmCurrentGncMode(gnc_mode);

    Fw::ParamValid valid;
    F32 max_torque = this->paramGet_MaxTorque(valid);
    m_max_torque = max_torque;
    if (valid == Fw::ParamValid::INVALID) {
        max_torque = 0.005f;
        m_max_torque = 0.005f;
    }

    U32 max_saturation = this->paramGet_MaxReactionWheelsSaturationPercent(valid);
    if (valid == Fw::ParamValid::INVALID) max_saturation = 70;

    U32 min_saturation = this->paramGet_MinReactionWheelsSaturationPercent(valid);
    if (valid == Fw::ParamValid::INVALID) min_saturation = 20;

    F32 max_rpm = this->paramGet_ReactionWheelsMaxRPM(valid);
    if (valid == Fw::ParamValid::INVALID) {
        gnc_mode = GncMode::SAFE;
        m_gnc_mode = GncMode::SAFE;
    }

    m_max_rpm = max_rpm;
    m_max_saturation_percent = max_saturation;
    m_min_saturation_percent = min_saturation;

    switch (gnc_mode) {
        case GncMode::OFF:
            return;
        
        case GncMode::DETUMBLE:
            this->LOOP_detumble_mode();

        case GncMode::POINTING:
            this->LOOP_pointing_mode();

        case GncMode::SAFE:
            this->LOOP_safe_mode();
    }

    m_last_ms = this->to_ms(this->getTime());
}

void GNCComponent ::LOOP_safe_mode() {
    Vec3 angle_error = this->get_error_from_vectors(
        m_current_sun_panels_normal,
        m_current_sun_normal
    );

    Vec3 ang_vel_error = this->get_error_from_vectors(
        m_current_angular_velocity.get_angular_rates(),
        Vec3(0, 0, 0)
    );

    this->tlmWrite_TlmAngleError(angle_error);
    this->tlmWrite_TlmAngularVelocityError(ang_vel_error);

    Vec3 moment = this->calculate_b_dot_safe(
        m_current_b_field,
        m_prev_b_field,
        10
    );

    this->tlmWrite_TlmControlMagnesticMoment(moment);
    this->MagnesticOut_out(0, moment);
}

void GNCComponent ::LOOP_detumble_mode() {
    Vec3 ang_vel_error = this->get_error_from_vectors(
        m_current_angular_velocity.get_angular_rates(),
        Vec3(0, 0, 0)
    );

    Vec3 angle_error = this->get_error_from_quats(
        m_current_orientation.get_orientation(),
        m_current_orientation.get_orientation()
    );

    this->tlmWrite_TlmAngularVelocityError(ang_vel_error);
    this->tlmWrite_TlmAngleError(angle_error);

    Fw::ParamValid valid;
    Vec3 Kd = this->paramGet_DGain(valid);
    if (valid == Fw::ParamValid::INVALID) {
        m_gnc_mode = GncMode::SAFE;
        return;
    }
    
    Vec3 Kp = this->paramGet_PGain(valid);
    if (valid == Fw::ParamValid::INVALID) {
        m_gnc_mode = GncMode::SAFE;
        return;
    }

    Vec3 torque = this->calculate_torque(
        angle_error,
        m_current_angular_velocity.get_angular_rates(),
        Vec3(0, 0, 0),
        this->to_ms(this->getTime()),
        300 * 1000, // 5 minute
        Kd,
        Kp
    );

    ProtectedObject<F32>::Status status;
    F32 max_torque = m_max_torque.get(0.005, status);
    if (status == ProtectedObject<F32>::Status::CORRUPTED) m_max_torque = 0.005;

    torque = this->clamp_vf(torque, max_torque);

    ProtectedObject<F32>::Status status_max_rpm;
    F32 max_rpm = m_max_rpm.get(6000.0f, status_max_rpm);

    ProtectedObject<U32>::Status status_percent;
    U32 min_percent = m_min_saturation_percent.get(20, status_percent);
    U32 max_percent = m_max_saturation_percent.get(80, status_percent);

    bool is_rpm_greater_max_percent;
    CombinedTorque res_torque = this->calculate_dual_actuator_output(
        torque,
        m_current_reaction_wheels_rpm,
        m_current_b_field,
        max_rpm,
        static_cast<F32>(min_percent) * 0.01f,
        static_cast<F32>(max_percent) * 0.01f,
        is_rpm_greater_max_percent
    );

    if (is_rpm_greater_max_percent) {
        m_gnc_mode = GncMode::DETUMBLE;
    }

    this->tlmWrite_TlmControlTorque(res_torque.wheels_torque);
    this->tlmWrite_TlmControlMagnesticMoment(res_torque.coil_dipole);
    this->MagnesticOut_out(0, res_torque.coil_dipole);
    this->TorqueOut_out(0, res_torque.wheels_torque);
}

void GNCComponent ::LOOP_pointing_mode() {
    ProtectedObject<WayPoint>::Status status;
    WayPoint target_point = m_target_waypoint.get(
        WayPoint(
            this->to_ms(this->getTime()),
            m_current_angular_velocity.get_angular_rates(),
            m_current_orientation.get_orientation()
        ),
        status
    );

    if (status == ProtectedObject<WayPoint>::Status::CORRUPTED) {
        m_gnc_mode = GncMode::SAFE;
        return;
    }

    Vec3 angle_error = get_error_from_quats(m_current_orientation.get_orientation(), target_point.get_orientation());
    Vec3 ang_vel_error = get_error_from_vectors(
        m_current_angular_velocity.get_angular_rates(),
        target_point.get_angular_velocity()
    );

    this->tlmWrite_TlmAngleError(angle_error);
    this->tlmWrite_TlmAngularVelocityError(ang_vel_error);

    Fw::ParamValid valid;
    Vec3 Kd = this->paramGet_DGain(valid);
    if (valid == Fw::ParamValid::INVALID) {
        m_gnc_mode = GncMode::SAFE;
        return;
    }
    
    Vec3 Kp = this->paramGet_PGain(valid);
    if (valid == Fw::ParamValid::INVALID) {
        m_gnc_mode = GncMode::SAFE;
        this->tlmWrite_TlmCurrentGncMode(GncMode::SAFE);
        return;
    }

    Vec3 torque = this->calculate_torque(
        angle_error,
        m_current_angular_velocity.get_angular_rates(),
        target_point.get_angular_velocity(),
        this->to_ms(this->getTime()),
        target_point.get_target_time(),
        Kd,
        Kp
    );

    ProtectedObject<F32>::Status status_max_torque;
    F32 max_torque = m_max_torque.get(0.005f, status_max_torque);
    if (status_max_torque == ProtectedObject<F32>::Status::CORRUPTED) m_max_torque = 0.005f;

    torque = this->clamp_vf(torque, max_torque);

    ProtectedObject<F32>::Status status_max_rpm;
    F32 max_rpm = m_max_rpm.get(6000.0f, status_max_rpm);

    ProtectedObject<U32>::Status status_percent;
    U32 min_percent = m_min_saturation_percent.get(20, status_percent);
    U32 max_percent = m_max_saturation_percent.get(80, status_percent);

    bool is_rpm_greater_max_percent;
    CombinedTorque res_torque = this->calculate_dual_actuator_output(
        torque,
        m_current_reaction_wheels_rpm,
        m_current_b_field,
        max_rpm,
        static_cast<F32>(min_percent) * 0.01f,
        static_cast<F32>(max_percent) * 0.01f,
        is_rpm_greater_max_percent
    );

    if (is_rpm_greater_max_percent) {
        m_gnc_mode = GncMode::DETUMBLE;
    }

    this->tlmWrite_TlmControlTorque(res_torque.wheels_torque);
    this->tlmWrite_TlmControlMagnesticMoment(res_torque.coil_dipole);
    this->MagnesticOut_out(0, res_torque.coil_dipole);
    this->TorqueOut_out(0, res_torque.wheels_torque);
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void GNCComponent ::SET_TARGET_WAYPOINT_cmdHandler(FwOpcodeType opCode,
                                                U32 cmdSeq,
                                                MyFprimeProject::WayPoint target_state) {
    m_target_waypoint = target_state;

    this->tlmWrite_TlmTargetWayPoint(target_state);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void GNCComponent ::SET_MODE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, MyFprimeProject::GncMode mode) {
    m_gnc_mode = mode;

    this->tlmWrite_TlmCurrentGncMode(mode);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace MyFprimeProject
