// ======================================================================
// \title  GNCComponent.hpp
// \author user
// \brief  hpp file for GNCComponent component implementation class
// ======================================================================

#ifndef MyFprimeProject_GNCComponent_HPP
#define MyFprimeProject_GNCComponent_HPP

#include <array>
#include <cstring>
#include <memory>
#include "Fw/Types/BasicTypes.h"
#include "MyFprimeProject/Components/GNCComponent/GNCComponentComponentAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/PositionSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/RotationSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/VelocitySerializableAc.hpp"
#include "Os/IntervalTimer.hpp"
#include <atomic>

namespace MyFprimeProject {

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

  inline void set(const T& value) {
    m_copy1 = value;
    m_copy2 = value;
    m_copy3 = value;
    update_mirror();
  }

  //! Check, restore (if need) and return value OR set and return given base value if can't restore corrupted value.
  //! Return value AND state in given param if provided.
  __attribute__((warn_unused_result("Use result of get function!")))
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
      if (__builtin_expect(raw[i] != ~m_mirror, 1)) return false;
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

class GNCComponent final : public GNCComponentComponentBase {
  private:
    ProtectedObject<F32> m_target_rotation = 0.0f;
    ProtectedObject<F32> m_target_position = 0.0f;
    ProtectedObject<F32> m_target_velocity = 0.0f;
    ProtectedObject<F32> m_target_angle_velocity = 0.0f;

    F32 m_current_rotation = 0.0f;
    F32 m_current_position = 0.0f;
    F32 m_current_velocity = 0.0f;
    F32 m_current_angle_velocity = 0.0f;

    U32 m_last_iter_time_ms = 0;
    U32 dt = 0;

    std::unique_ptr<Os::IntervalTimer> timer;

    bool m_is_target_rotation_set = false;
    bool m_is_target_position_set = false;
    bool m_is_target_velocity_set = false;
    bool m_is_target_angle_velocity_set = false;

    ProtectedObject<U32> time_for_rotate_ms = 0;
    ProtectedObject<U32> time_for_exchange_position = 0;
    ProtectedObject<U32> time_for_exchange_velocity = 0;
    ProtectedObject<U32> time_for_exchange_angle_velocity = 0;

    void rotate_to_target();
    void velocity_to_target();
    void position_to_target();
    void angle_velocity_to_target();

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

    //! Handler implementation for positionIn
    void positionIn_handler(FwIndexType portNum,                       //!< The port number
                            const MyFprimeProject::Position& position  //!< Position value
                            ) override;

    //! Handler implementation for rotationIn
    void rotationIn_handler(FwIndexType portNum,                       //!< The port number
                            const MyFprimeProject::Rotation& rotation  //!< Rotation value
                            ) override;

    //! Handler implementation for velocityIn
    void velocityIn_handler(FwIndexType portNum,                       //!< The port number
                            const MyFprimeProject::Velocity& velocity  //!< Velocity value
                            ) override;

    //! Handler implementation for schedIn
    void schedIn_handler(FwIndexType portNum,  //!< The port number
                         U32 context           //!< The call order
                         ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_ROTATION
    //!
    //! Command to change current Rotation to vector x, y, z
    void SET_ROTATION_cmdHandler(FwOpcodeType opCode,                       //!< The opcode
                                 U32 cmdSeq,                                //!< The command sequence number
                                 MyFprimeProject::Rotation target_rotation  //!< Target rotation
                                 ) override;

    //! Handler implementation for command SET_POSITION
    //!
    //! Command to change current position to vector x, y, z
    void SET_POSITION_cmdHandler(FwOpcodeType opCode,                       //!< The opcode
                                 U32 cmdSeq,                                //!< The command sequence number
                                 MyFprimeProject::Position target_position  //!< Target position
                                 ) override;

    //! Handler implementation for command SET_VELOCITY
    //!
    //! Command to change current velocity to vector x, y, z
    void SET_VELOCITY_cmdHandler(FwOpcodeType opCode,                       //!< The opcode
                                 U32 cmdSeq,                                //!< The command sequence number
                                 MyFprimeProject::Velocity target_velocity  //!< Target velocity
                                 ) override;
};

}  // namespace MyFprimeProject

#endif
