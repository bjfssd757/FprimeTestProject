// ======================================================================
// \title  GCComponent.hpp
// \author user
// \brief  hpp file for GCComponent component implementation class
// ======================================================================

#ifndef GCProject_GCComponent_HPP
#define GCProject_GCComponent_HPP

#include <array>
#include <cmath>
#include <cstring>
#include "Fw/Prm/ParamValidEnumAc.hpp"
#include "Fw/Time/Time.hpp"
#include "Fw/Types/BasicTypes.h"
#include "GCProject/Components/GCComponent/BatteryStateSerializableAc.hpp"
#include "GCProject/Components/GCComponent/GCComponentComponentAc.hpp"
#include "GCProject/Components/GCComponent/GncModeEnumAc.hpp"
#include "GCProject/Components/GCComponent/GyroDataSerializableAc.hpp"
#include "GCProject/Components/GCComponent/OrientDataSerializableAc.hpp"
#include "GCProject/Components/GCComponent/Vec3SerializableAc.hpp"
#include "GCProject/Components/GCComponent/WayPointSerializableAc.hpp"
#include <atomic>
#include <type_traits>

namespace GCProject {

extern std::atomic<U32> total_correction_memory;

template<typename T>
struct ProtectedObject {
private:
  static constexpr size_t Words = (sizeof(T) + (sizeof(U64) - 1) / sizeof(U64));
  using MirrorStorage = std::array<U64, Words>;

  alignas(64) T m_copy1;
  alignas(64) T m_copy2;
  alignas(64) T m_copy3;
  alignas(64) MirrorStorage m_mirror;

  inline void update_mirror() {
    const U64* raw = reinterpret_cast<U64*>(&m_copy1);
    for (size_t i = 0; i < Words; ++i) {
      if (std::is_polymorphic<T>::value) {
        m_mirror[i] = raw[i];
        continue;
      }
      m_mirror[i] = ~raw[i];
    }
  }

public:
  enum Status {OK, RESTORED, CORRUPTED};

  ProtectedObject<T>& operator=(const T& v) {
    set(v);
    return *this;
  }

  constexpr ProtectedObject(const T& v) { set(v); }
  constexpr ProtectedObject() : m_copy1{}, m_copy2{}, m_copy3{} {
    update_mirror();
  }

  inline void set(const T& value) {
    m_copy1 = value;
    m_copy2 = value;
    m_copy3 = value;
    update_mirror();
  }

  //! Check, restore (if need) and return value OR set and return given base value if can't restore corrupted value.
  //! Return value AND state in given param if provided.
  [[nodiscard]]
  inline T get(const T& base, Status& out_status) {
    // Best way: value is valid
    if (__builtin_expect(validated_mirror(), 1)) {
      out_status = OK;
      return m_copy1;
    }

    return repair_and_get(base, out_status);
  }

private:
  bool validated_mirror() const {
    const U64* raw = reinterpret_cast<const U64*>(&m_copy1);
    for (size_t i = 0; i < Words; ++i) {
      if (std::is_polymorphic<T>::value) {
        if (raw[i] != m_mirror[i]) return false;
      }
      if (__builtin_expect(raw[i] != ~m_mirror[i], 1)) return false;
    }
    return true;
  }

  [[gnu::cold, gnu::noinline]]
  T repair_and_get(const T& base, Status& out_status) {
    auto cmp = [](const T& a, const T& b) {
      return memcmp(static_cast<const void*>(&a), static_cast<const void*>(&b), sizeof(T)) == 0;
    };

    bool c12 = cmp(m_copy1, m_copy2);
    bool c13 = cmp(m_copy1, m_copy3);
    bool c23 = cmp(m_copy2, m_copy3);

    // Error: copy 2 or copy 3 is corrupted, but copy 1 is ok
    if (c12 || c13) {
      m_copy2 = m_copy1;
      m_copy3 = m_copy1;
      update_mirror();
      total_correction_memory++;
      out_status = RESTORED;
      return m_copy1;
    }
    
    // Error: copy 1 is corrupted, but copy 2 is ok
    if (c23) {
      m_copy1 = m_copy2;
      m_copy3 = m_copy2;
      update_mirror();
      total_correction_memory++;
      out_status = RESTORED;
      return m_copy2;
    }

    // CRITICAL WAY: We can't restore corrupted value, return base value
    out_status = CORRUPTED;
    return base;
  }
};

class EMA {
  ProtectedObject<F32> m_alpha;
  bool m_first_value;
  F32 m_filtered_value;

public:
  EMA(F32 alpha = 0.1f) : m_alpha(alpha), m_first_value(true), m_filtered_value(0.0f) {}

  F32 process(F32 input) {
    if (m_first_value) {
      m_filtered_value = input;
      m_first_value = false;
    } else {
      ProtectedObject<F32>::Status is_alpha_valid;
      F32 alpha = m_alpha.get(0.1f, is_alpha_valid);

      if (is_alpha_valid == ProtectedObject<F32>::Status::CORRUPTED) {
        m_alpha = 0.1f;
      }

      m_filtered_value = alpha * input + (1.0f - alpha) * m_filtered_value;
    }
    return m_filtered_value;
  }

  void reset() {
    m_first_value = true;
  }

  void set_alpha(F32 alpha) {
    m_alpha = alpha;
  }
};

class GCComponent final : public GCComponentComponentBase {
  private:
    U32 m_dt_ms = 0;
    U64 m_last_ms = 0;

    ProtectedObject<WayPoint> m_target_waypoint;
    ProtectedObject<F32> m_max_torque;
    ProtectedObject<GncMode> m_gnc_mode;
    ProtectedObject<GncMode> m_last_gnc_mode;
    ProtectedObject<EMA> m_ema_filter;
    ProtectedObject<F32> m_max_rpm;
    ProtectedObject<U32> m_min_saturation_percent;
    ProtectedObject<U32> m_max_saturation_percent;

    bool m_current_orient_valid = true;
    bool m_current_angular_vel_valid = true;

    Vec3 m_current_sun_normal;
    Vec3 m_current_sun_panels_normal;
    OrientData m_current_orientation;
    GyroData m_current_angular_velocity;
    Vec3 m_current_position;
    BatteryState m_current_battery_state;
    Vec3 m_current_b_field;
    Vec3 m_prev_b_field;
    Vec3 m_current_reaction_wheels_rpm;

    void LOOP_detumble_mode();
    void LOOP_safe_mode();
    void LOOP_pointing_mode();

    struct CombinedTorque {
      Vec3 wheels_torque;
      Vec3 coil_dipole;
    };

    CombinedTorque calculate_dual_actuator_output(
      const Vec3& required_torque,
      const Vec3& current_rpm,
      const Vec3& current_b_field,
      F32 max_rpm,
      F32 min_percent,
      F32 max_percent,
      bool& out_is_rpm_greater_max
    ) {
      CombinedTorque out;
      out.wheels_torque = required_torque;
      out.coil_dipole = Vec3(0, 0, 0);

      F32 low_limit = max_rpm * min_percent;
      F32 high_limit = max_rpm * max_percent;

      Vec3 desat_torque;
      Fw::ParamValid valid;
      F32 k_desat = this->paramGet_KDesat(valid);
      if (valid == Fw::ParamValid::INVALID) k_desat = 0.1f;

      auto get_desat_component = [&](F32 rpm) {
        F32 abs_rpm = (rpm < 0) ? -rpm : rpm;
        if (abs_rpm < low_limit) return 0.0f;

        return rpm * k_desat;
      };

      Vec3 h_vector;
      h_vector.set_x(get_desat_component(current_rpm.get_x()));
      h_vector.set_y(get_desat_component(current_rpm.get_y()));
      h_vector.set_z(get_desat_component(current_rpm.get_z()));

      F32 b_mag_sq = current_b_field.get_x() * current_b_field.get_x() + 
                     current_b_field.get_y() * current_b_field.get_y() + 
                     current_b_field.get_z() * current_b_field.get_z();

      if (b_mag_sq > 1e-12f) {
        F32 inv_b2 = 1.0f / b_mag_sq;
        out.coil_dipole.set_x((h_vector.get_y() * current_b_field.get_z() - h_vector.get_z() * current_b_field.get_y()) * inv_b2);
        out.coil_dipole.set_y((h_vector.get_z() * current_b_field.get_x() - h_vector.get_x() * current_b_field.get_z()) * inv_b2);
        out.coil_dipole.set_z((h_vector.get_x() * current_b_field.get_y() - h_vector.get_y() * current_b_field.get_x()) * inv_b2);

        F32 m_x = out.coil_dipole.get_y() * current_b_field.get_z() - out.coil_dipole.get_z() * current_b_field.get_y();
        F32 m_y = out.coil_dipole.get_z() * current_b_field.get_x() - out.coil_dipole.get_x() * current_b_field.get_z();
        F32 m_z = out.coil_dipole.get_x() * current_b_field.get_y() - out.coil_dipole.get_y() * current_b_field.get_x();

        out.wheels_torque.set_x(required_torque.get_x() - m_x);
        out.wheels_torque.set_y(required_torque.get_y() - m_y);
        out.wheels_torque.set_z(required_torque.get_z() - m_z);
      }

      for (int i = 0; i < 3; i++) {
        F32 r = (i == 0) ? current_rpm.get_x() : (i == 1) ? current_rpm.get_y() : current_rpm.get_z();
        if (((r < 0) ? -r : r) > high_limit) {
          out_is_rpm_greater_max = true;
        }
      }

      return out;
    }

    Vec3 calculate_b_dot_safe(const Vec3& currentB, const Vec3& prevB, float dt, float J_min = 0.02f) {
      Vec3 b_dot;
      b_dot.set_x((currentB.get_x() - prevB.get_x()) / dt);
      b_dot.set_y((currentB.get_y() - prevB.get_y()) / dt);
      b_dot.set_z((currentB.get_z() - prevB.get_z()) / dt);

      float b_mag_sq = currentB.get_x() * currentB.get_x() + 
                      currentB.get_y() * currentB.get_y() + 
                      currentB.get_z() * currentB.get_z();

      if (b_mag_sq < 1e-12f) return Vec3(0, 0, 0);

      float k = (2.0f * J_min) / b_mag_sq;

      Vec3 dipole;
      dipole.set_x(-k * b_dot.get_x());
      dipole.set_y(-k * b_dot.get_y());
      dipole.set_z(-k * b_dot.get_z());

      return dipole;
    }

    Vec3 calculate_torque(
      const Vec3& angle_error,
      const Vec3& current_angular_vel,
      const Vec3& target_angular_vel,
      U64 current_time_ms,
      U64 target_time_ms,
      const Vec3& Kd,
      const Vec3& Kp
    ) {
      U64 diff = (target_time_ms > current_time_ms) ? (target_time_ms - current_time_ms) : 1;
      F32 dt_sec = static_cast<F32>(diff) / 1000.0f;

      if (dt_sec < 0.01f) dt_sec = 0.01f;

      Vec3 torque;

      auto calc_axis = [&](F32 err, F32 cur_v, F32 tar_v, F32 kp_v, F32 kd_v) {
        F32 v_req = (err * kp_v / dt_sec) + tar_v;
        return (v_req - cur_v) * kd_v;
      };

      torque.set_x(calc_axis(angle_error.get_x(), current_angular_vel.get_x(), target_angular_vel.get_x(), Kp.get_x(), Kd.get_x()));
      torque.set_y(calc_axis(angle_error.get_y(), current_angular_vel.get_y(), target_angular_vel.get_y(), Kp.get_y(), Kd.get_y()));
      torque.set_z(calc_axis(angle_error.get_z(), current_angular_vel.get_z(), target_angular_vel.get_z(), Kp.get_z(), Kd.get_z()));

      return torque;
    }

    WayPoint make_current_way_point() const {
      WayPoint point = WayPoint(
        to_ms(this->getTime()),
        m_current_angular_velocity.get_angular_rates(),
        m_current_orientation.get_orientation()
      );
      return point;
    }

    static inline Vec3 get_error_from_vectors(const Vec3& current, const Vec3& target) {
        Vec3 err;
        err.set_x(current.get_y() * target.get_z() - current.get_z() * target.get_y());
        err.set_y(current.get_z() * target.get_x() - current.get_x() * target.get_z());
        err.set_z(current.get_x() * target.get_y() - current.get_y() * target.get_x());
        return err;
    }

    static inline Vec3 get_error_from_quats(const Quaternion& current, const Quaternion& target) {
        F32 tw = target.get_w(), tx = target.get_x(), ty = target.get_y(), tz = target.get_z();
        F32 cw = current.get_w(), cx = current.get_x(), cy = current.get_y(), cz = current.get_z();

        float x =  tw * -cx + tx * cw + ty * -cz - tz * -cy;
        float y =  tw * -cx - tx * -cz + ty * cw + tz * -cx;
        float z =  tw * -cz + tx * -cy - ty * -cx + tz * cw;

        float dot = cw * tw + cx * tx + cy * ty + cz * tz;
        return (dot < 0.0f) ? Vec3(-x, -y, -z) : Vec3(x, y, z);
    }


    static inline U64 to_ms(Fw::Time time) {
      return (static_cast<U64>(time.getSeconds()) * 1000) +
             (static_cast<U64>(time.getUSeconds()) / 1000);
    }

    static inline Vec3 calculate_rotation_error(const Vec3& normal_a, const Vec3& normal_b) {
      Vec3 error_axis;
      error_axis.set_x(normal_a.get_y() * normal_b.get_z() - normal_a.get_z() * normal_b.get_y());
      error_axis.set_y(normal_a.get_z() * normal_b.get_x() - normal_a.get_x() * normal_b.get_z());
      error_axis.set_z(normal_a.get_x() * normal_b.get_y() - normal_a.get_y() * normal_b.get_x());

      if (dot(normal_a, normal_b) < -0.999f) {
        error_axis.set_x(0.0f);
      }

      return error_axis;
    }

    static inline void vec_normalize(Vec3& v) {
      float x2 = v.get_x() * v.get_x() + v.get_y() * v.get_y() + v.get_z() * v.get_z();
      if (x2 < 0.0001f) return;

      float invLen = 1.0f / std::sqrt(x2); 
      v.set(v.get_x() * invLen, v.get_y() * invLen, v.get_z() * invLen);
    }

    static inline F32 dot(const Vec3& v1, const Vec3& v2) {
      F32 v1x = v1.get_x(), v1y = v1.get_y(), v1z = v1.get_z();
      F32 v2x = v2.get_x(), v2y = v2.get_y(), v2z = v2.get_z();

      return v1x * v2x + v1y * v2y + v1z * v2z;
    }

    //! Clamp vector by max (max type is F32)
    inline static Vec3 clamp_vf(const Vec3& v, const F32 max) {
      Vec3 res;
      if (v.get_x() > max) res.set_x(max);
      if (v.get_y() > max) res.set_y(max);
      if (v.get_z() > max) res.set_z(max);

      if (v.get_x() < -max) res.set_x(-max);
      if (v.get_y() < -max) res.set_y(-max);
      if (v.get_z() < -max) res.set_z(-max);

      return res;
    }
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct GCComponent object
    GCComponent(const char* const compName  //!< The component name
    );

    //! Destroy GCComponent object
    ~GCComponent();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for GyroDataIn
    //!
    //! Port for transfer data from gyroscop to GNC component. \
      //! GNC uses this data as current angular velocity for EKF prediction
    void GyroDataIn_handler(FwIndexType portNum,  //!< The port number
                            const GCProject::GyroData& data,
                            bool is_valid) override;

    //! Handler implementation for StarTrackerDataIn
    //!
    //! Port for transfer data from star tracker to GNC component. \
      //! GNC use this data as current orientation
    void StarTrackerDataIn_handler(FwIndexType portNum,  //!< The port number
                                   const GCProject::OrientData& data,
                                  bool is_valid) override;

    //! Handler implementation for SunPanelsNormalIn
    //!
    //! Port for transfer normalize vector of all sun panels on the spacecraft
    void SunPanelsNormalIn_handler(FwIndexType portNum,  //!< The port number
                                   const GCProject::Vec3& normal) override;

    //! Handler implementation for SunVectorIn
    //!
    //! Port for transfer vector to Sun from sensor/tracker
    void SunVectorIn_handler(FwIndexType portNum,  //!< The port number
                             const GCProject::Vec3& position) override;

    //! Handler implementation for schedIn
    //!
    //! GNC Tick Port. Should called every 10 ms
    void schedIn_handler(FwIndexType portNum,  //!< The port number
                         U32 context           //!< The call order
                         ) override;

    //! Handler implementation for BatteryStateIn
    //!
    //! Port for transfer battery state.
    //! GNC use this state for managing system
    void BatteryStateIn_handler(FwIndexType portNum,  //!< The port number
                                const GCProject::BatteryState& battery_state) override;

    //! Handler implementation for MagnesticData
    //!
    //! Port for transfer magnetometor data
    void MagnesticData_handler(FwIndexType portNum,  //!< The port number
                               const GCProject::Vec3& data) override;

    //! Handler implementation for ReactionWheelsRPMData
    //!
    //! Port for transfer current RPM of reaction wheels
    void ReactionWheelsRPMData_handler(FwIndexType portNum,
                                       const GCProject::Vec3& data) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_TARGET_STATE
    //!
    //! Command to set target state for specific time in ms by start of the mission
    void SET_TARGET_WAYPOINT_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                     U32 cmdSeq,           //!< The command sequence number
                                     GCProject::WayPoint target_state) override;

    //! Handler implementation for command SET_MODE
    void SET_MODE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                             U32 cmdSeq,           //!< The command sequence number
                             GCProject::GncMode mode) override;
};

}  // namespace GCProject

#endif
