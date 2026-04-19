// ======================================================================
// \title  GNCComponent.hpp
// \author user
// \brief  hpp file for GNCComponent component implementation class
// ======================================================================

#ifndef MyFprimeProject_GNCComponent_HPP
#define MyFprimeProject_GNCComponent_HPP

#include <cstring>
#include <initializer_list>
#include <memory>
#include <type_traits>
#include "Fw/Types/BasicTypes.h"
#include "MyFprimeProject/Components/GNCComponent/GNCComponentComponentAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/PositionSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/RotationSerializableAc.hpp"
#include "MyFprimeProject/Components/GNCComponent/VelocitySerializableAc.hpp"
#include "Os/IntervalTimer.hpp"
#include <immintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

namespace MyFprimeProject {

template<typename T>
struct ProtectedNumber {
  static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type!");

  enum ProtectionState { OK, RESTORED, CORRUPTED };

  using MirrorType = typename std::conditional<sizeof(T) == 8, U64, U32>::type;

  alignas(64) mutable T value;
  alignas(64) mutable T copy;
  alignas(64) mutable T copy2;
  alignas(64) mutable MirrorType mirror;

  ProtectedNumber<T>& operator=(const T number) {
    set(number);
    return *this;
  }

  constexpr ProtectedNumber() : value(0), mirror(~0) {}

  ProtectedNumber(T v) {
    set(v);
  }

  inline void set(T v) const {
    this->value = v;
    this->copy = v;
    this->copy2 = v;
    MirrorType temp;
    std::memcpy(&temp, &v, sizeof(T));
    this->mirror = ~temp;
  }

  //! Check, restore (if need) and return value OR set and return given base value if can't restore corrupted value.
  //! Return value AND state in given param if provided.
  __attribute__((warn_unused_result("Use result of get function!")))
  inline T get_safe(T base, ProtectionState* state = nullptr) const {
    MirrorType raw;
    std::memcpy(&raw, &this->value, sizeof(T));

    // Best way: value is valid
    if (__builtin_expect(raw == ~this->mirror, 1)) {
      if (state) *state = OK;
      return value;
    }

    return this->repair_and_get(base, state);
  }

  __attribute__((cold, noinline))
  T repair_and_get(T base, ProtectionState* state = nullptr) const {
    MirrorType raw, raw_copy, raw_copy2, raw_base;
    std::memcpy(&raw, &this->value, sizeof(T));
    std::memcpy(&raw_copy, &this->copy, sizeof(T));
    std::memcpy(&raw_copy2, &this->copy2, sizeof(T));
    std::memcpy(&raw_base, &base, sizeof(T));

    bool critical = false;
    MirrorType winner = vote(raw,
      raw_copy,
      raw_copy2,
      mirror,
      raw_base,
      &critical
    );
    
    // CRITICAL: We can't restore value. We should return and set provided base value
    if (critical) {
        set(base);
        if (state) *state = CORRUPTED;
        return base;
    }

    // Here we have an error, but we know how to restore value
    T restored_val;
    std::memcpy(&restored_val, &winner, sizeof(T));
    set(restored_val); // Restore all copies

    if (state) *state = RESTORED;
    return restored_val;
  }

  inline bool is_error() const {
    MirrorType raw;
    std::memcpy(&raw, &this->value, sizeof(T));
    return raw != ~this->mirror;
  }

  private:
    static inline MirrorType vote(MirrorType v1, MirrorType v2, MirrorType v3, MirrorType inv, MirrorType base, bool* is_corrupted = nullptr) {
      if (v1 == ~inv) return v1;
      if (v2 == ~inv) return v2;
      if (v3 == ~inv) return v3;
      if (v1 == v2 || v1 == v3) return v1;
      if (v2 == v3) return v2;

      if (is_corrupted) *is_corrupted = true;
      return base;
    }

  public:
    ProtectedNumber<T>& operator+(const ProtectedNumber<T>& other) {
      set(this->value + other.value);
      return *this;
    }

    ProtectedNumber<T>& operator*(const ProtectedNumber<T>& other) {
      set(this->value * other.value);
      return *this;
    }

    bool operator==(ProtectedNumber<T> other) {
      return this->value == other.value;
    }

    bool operator!=(ProtectedNumber<T> other) {
      return this->value != other.value;
    }

    bool operator<=(ProtectedNumber<T> other) {
      return this->value <= other.value;
    }

    bool operator>=(ProtectedNumber<T> other) {
      return this->value >= other.value;
    }

    bool operator>(ProtectedNumber<T> other) {
      return this->value > other.value;
    }

    bool operator<(ProtectedNumber<T> other) {
      return this->value < other.value;
    }
};

template<typename T>
struct ProtectedVector3 {
  ProtectedNumber<T> x;
  ProtectedNumber<T> y;
  ProtectedNumber<T> z;

  ProtectedVector3(std::initializer_list<ProtectedNumber<T>> init) {
    auto it = init.begin();
    (it != init.end()) ? this->x = *it++ : this->x = 0;
    (it != init.end()) ? this->y = *it++ : this->y = 0;
    (it != init.end()) ? this->z = *it : this->z = 0;
  }

  ProtectedVector3(T x, T y, T z) {
    this->x = ProtectedNumber<T>(x);
    this->y = ProtectedNumber<T>(y);
    this->z = ProtectedNumber<T>(z);
  }

  constexpr ProtectedVector3() : x(0), y(0), z(0) {}
  
  ProtectedVector3<T>& operator=(const Position& other) {
    this->x = other.get_x();
    this->y = other.get_y();
    this->z = other.get_z();
    return *this;
  }

  ProtectedVector3<T>& operator=(const Rotation& other) {
    this->x = other.get_x();
    this->y = other.get_y();
    this->z = other.get_z();
    return *this;
  }

  ProtectedVector3<T>& operator=(const Velocity& other) {
    this->x = other.get_x();
    this->y = other.get_y();
    this->z = other.get_z();
    return *this;
  }

  ProtectedVector3<T>& operator+(const ProtectedVector3<T>& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }

  ProtectedVector3<T>& operator-(const ProtectedVector3<T>& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    return *this;
  }

  static constexpr inline ProtectedNumber<F32> distance(const ProtectedVector3<T>& v1, const ProtectedVector3<T>& v2) {
    __m128 _mm_v1 = _mm_setr_ps(v1.x.get_safe(0), v1.y.get_safe(0), v1.z.get_safe(0), 0.0f);
    __m128 _mm_v2 = _mm_setr_ps(v2.x.get_safe(0), v2.y.get_safe(0), v2.z.get_safe(0), 0.0f);

    __m128 diff = _mm_sub_ps(_mm_v1, _mm_v2);
    __m128 dot = _mm_dp_ps(diff, diff, 0x71);
    __m128 res = _mm_sqrt_ps(dot);

    ProtectedNumber<F32> res_as_f = ProtectedNumber<F32>(_mm_cvtss_f32(res));
    return res_as_f;
  }

  bool operator==(const ProtectedVector3<T>& other) {
    if (this->x == other.x && this->y == other.y && this->z == other.z) return true;
    return false;
  }

  bool operator!=(const ProtectedVector3<T>& other) {
    if (this->x != other.x && this->y != other.y && this->z != other.z) return true;
    return false;
  }

  constexpr bool is_any_error() const {
    return this->x.is_error() || this->y.is_error() || this->z.is_error();
  }

  constexpr bool is_x_error() const { return this->x.is_error(); }
  constexpr bool is_y_error() const { return this->y.is_error(); }
  constexpr bool is_z_error() const { return this->z.is_error(); }

  inline Position to_position() const {
    Position pos = Position().set(this->x, this->y, this->z);
    return pos;
  }

  inline Rotation to_rotation() const {
    Rotation pos = Rotation().set(this->x, this->y, this->z);
    return pos;
  }

  inline Velocity to_velocity() const {
    Velocity pos = Velocity().set(this->x, this->y, this->z);
    return pos;
  }
};

class GNCComponent final : public GNCComponentComponentBase {
  private:
    ProtectedVector3<F32> m_target_rotation;
    ProtectedVector3<F32> m_target_position;
    ProtectedVector3<F32> m_target_velocity;
    ProtectedVector3<F32> m_target_angle_velocity;

    ProtectedVector3<F32> m_current_rotation;
    ProtectedVector3<F32> m_current_position;
    ProtectedVector3<F32> m_current_velocity;
    ProtectedVector3<F32> m_current_angle_velocity;

    ProtectedNumber<U32> m_last_iter_time_ms;
    ProtectedNumber<U32> dt;

    ProtectedNumber<F32> m_integral;

    std::unique_ptr<Os::IntervalTimer> timer;

    bool m_is_target_rotation_set = false;
    bool m_is_target_position_set = false;
    bool m_is_target_velocity_set = false;
    bool m_is_target_angle_velocity_set = false;

    ProtectedNumber<U32> time_for_rotate_ms;
    ProtectedNumber<U32> time_for_exchange_position;
    ProtectedNumber<U32> time_for_exchange_velocity;
    ProtectedNumber<U32> time_for_exchange_angle_velocity;

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
