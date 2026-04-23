// ======================================================================
// \title  GNCComponent.hpp
// \author user
// \brief  hpp file for GNCComponent component implementation class
// ======================================================================

#ifndef MyFprimeProject_GNCComponent_HPP
#define MyFprimeProject_GNCComponent_HPP

#include <array>
#include <cstring>
#include "Eigen/Core"
#include "Fw/Types/BasicTypes.h"
#include "MyFprimeProject/Components/GNCComponent/GNCComponentComponentAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/GncModeEnumAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/GyroDataSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/OrientDataSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/Vec3SerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/WayPointSerializableAc.hpp"
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace MyFprimeProject {

extern std::atomic<U32> total_correction_memory;
extern std::atomic<U32> current_mission_time_ms;

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
      m_mirror[i] = ~raw[i];
    }
  }

public:
  enum Status {OK, RESTORED, CORRUPTED};

  ProtectedObject<T>& operator=(const T v) {
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
  inline T get(T base, Status& out_status) {
    // Best way: value is valid
    if (__buildin_expect(validated_mirror(), 1)) {
      out_status = OK;
      return m_copy1;
    }

    return repair_and_get(base, out_status);
  }

private:
  bool validated_mirror() const {
    const U64* raw = reinterpret_cast<const U64*>(&m_copy1);
    for (size_t i = 0; i < Words; ++i) {
      if (__builtin_expect(raw[i] != ~m_mirror[i], 1)) return false;
    }
    return true;
  }

  [[gnu::cold, gnu::noinline]]
  T repair_and_get(T base, Status& out_status) {
    bool c12 = (memcmp(&m_copy1, &m_copy2, sizeof(T)) == 0);
    bool c13 = (memcmp(&m_copy1, &m_copy3, sizeof(T)) == 0);
    bool c23 = (memcmp(&m_copy2, &m_copy3, sizeof(T)) == 0);

    // Error: copy 2 or copy 3 is corrupted, but copy 1 is ok
    if (c12 || c13) {
      m_copy2 = m_copy1;
      m_copy3 = m_copy1;
      update_mirror();
      out_status = RESTORED;
      return m_copy1;
    }
    
    // Error: copy 1 is corrupted, but copy 2 is ok
    if (c23) {
      m_copy1 = m_copy2;
      m_copy3 = m_copy2;
      update_mirror();
      out_status = RESTORED;
      return m_copy2;
    }

    // CRITICAL WAY: We can't restore corrupted value, return base value
    out_status = CORRUPTED;
    return base;
  }
};

//! Extend Kalman Filter
//! S - state size
//! M - measurement size
template<int S, int M>
class EKF {
  Eigen::Matrix<F32, S, 1> x;
  Eigen::Matrix<F32, S, S> P;
  Eigen::Matrix<F32, S, S> Q;
  Eigen::Matrix<F32, M, M> R;

  void predict(F32 dt) {
    Eigen::Matrix<F32, S, S> F = compute_jacobian_F(x, dt);

    P.normalize() = F * P * F.transpose() + Q;
  }

  void update(const Eigen::Matrix<F32, M, 1>& z) {
    Eigen::Matrix<F32, M, S> H = compute_jacobian_H(x);

    auto S_mat = H * P * H.transpose() + R;
    Eigen::Matrix<F32, S, M> K = P * H.transpose() * S_mat.inverse();

    x = x + K * (z - h(x));

    P = (Eigen::Matrix<F32, S, S>::Identity() - K * H) * P;
  }

  static inline Eigen::Matrix<F32, S, S> compute_jacobian_F(Eigen::Matrix<F32, S, 1> target_x, F32 dt) {
    // TODO
  }

  static inline Eigen::Matrix<F32, M, S> compute_jacobian_H(Eigen::Matrix<F32, S, 1> target_x) {
    // TODO
  }
};

class GNCComponent final : public GNCComponentComponentBase {
  private:
    ProtectedObject<WayPoint> m_target_waypoint;
    ProtectedObject<F64> m_max_torque;
    ProtectedObject<Vec3> m_proportional_gain;
    ProtectedObject<Vec3> m_derivative_gain;
    ProtectedObject<Vec3> m_q_sigma;
    ProtectedObject<Vec3> m_q_gyro_sigma;
    ProtectedObject<Vec3> q_star_sigma;
    ProtectedObject<GncMode> m_gnc_mode;
    ProtectedObject<Vec3> m_sun_panels_normal;

    Vec3 m_current_sun_vector;
    OrientData m_current_orientation;
    GyroData m_current_angular_velocity;
    Vec3 m_current_position;

    inline WayPoint make_current_way_point() const {
      WayPoint point = WayPoint(
        current_mission_time_ms.load(),
        m_current_position,
        m_current_angular_velocity.get_angular_rates(),
        m_current_orientation.get_orientation()
      );
      return point;
    }
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct GNCComponent object
    GNCComponent(const char* const compName  //!< The component name
    );

    //! Destroy GNCComponent object
    ~GNCComponent();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for GyroDataIn
    //!
    //! Port for transfer data from gyroscop to GNC component. \
      //! GNC uses this data as current angular velocity for EKF prediction
    void GyroDataIn_handler(FwIndexType portNum,  //!< The port number
                            const MyFprimeProject::GyroData& data) override;

    //! Handler implementation for StarTrackerDataIn
    //!
    //! Port for transfer data from star tracker to GNC component. \
      //! GNC use this data as current orientation
    void StarTrackerDataIn_handler(FwIndexType portNum,  //!< The port number
                                   const MyFprimeProject::OrientData& data) override;

    //! Handler implementation for SunPanelsNormalIn
    //!
    //! Port for transfer normalize vector of all sun panels on the spacecraft
    void SunPanelsNormalIn_handler(FwIndexType portNum,  //!< The port number
                                   const MyFprimeProject::Vec3& normal) override;

    //! Handler implementation for SunVectorIn
    //!
    //! Port for transfer vector to Sun from sensor/tracker
    void SunVectorIn_handler(FwIndexType portNum,  //!< The port number
                             const MyFprimeProject::Vec3& position) override;

    //! Handler implementation for schedIn
    //!
    //! GNC Tick Port. Should called every 10 ms
    void schedIn_handler(FwIndexType portNum,  //!< The port number
                         U32 context           //!< The call order
                         ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_TARGET_STATE
    //!
    //! Command to set target state for specific time in ms by start of the mission
    void SET_TARGET_WAYPOINT_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                     U32 cmdSeq,           //!< The command sequence number
                                     MyFprimeProject::WayPoint target_state) override;

    //! Handler implementation for command SET_MODE
    void SET_MODE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                             U32 cmdSeq,           //!< The command sequence number
                             MyFprimeProject::GncMode mode) override;
};

}  // namespace MyFprimeProject

#endif
