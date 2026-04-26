# F´ GC Component

Implementation of the Guidance and Control (GC) component for the F´ (F Prime) framework.

# Table of Contents

- [Introduction](#introduction)
- [Architecture](#architecture)
    - [PD Controller](#pd-controller)
    - [Operating Modes](#operating-modes)
        - [Pointing Mode](#pointing-mode)
        - [Detumble Mode](#detumble-mode-angular-velocity-hold)
        - [Safe Mode](#safe-mode)
        - [Off Mode](#off-mode)
    - [Error Correction Code (ECC)](#error-correction-code-ecc)

# Introduction

F´ is an open-source framework developed by **NASA JPL**, tailored for embedded systems and small-scale spacecraft. It was notably used as the software foundation for the Mars 2020 mission’s **Ingenuity** helicopter.

The framework's architecture is built around **components** and their interactions, as well as the **GDS (Ground Data System)** — a ground station interface implemented as a Python application.

<details>
    <summary>A brief overview of the F´ architecture</summary>

- **Ports** facilitate communication between components.
- **Telemetry** transmits real-time key data from components to the GDS.
- **Parameters** allow the GDS to send configuration values to components.
- **Commands** enable the GDS to send specific instructions to components.
- **Events** provide a mechanism for components to send alerts and logs to the GDS.
</details>

One of the primary advantages of F´ is the **FPP (F Prime Prime)** declarative language. It is used to define component definitions, which are then used to generate C++ boilerplate code, allowing developers to focus strictly on the component's logic.

---

**GC (Guidance and Control)** is a system responsible for orientation and stabilization, ensuring a spacecraft precisely follows the attitude set by the Mission Control Center (MCC).

Typically, a full **GNC** (Guidance, Navigation, and Control) system is implemented. However, since navigation requires highly specific spacecraft knowledge, this component focuses strictly on the **GC** portion to remain versatile and reusable.


# Architecture

The system's foundation lies in data received from sensors, which arrives pre-processed by a **navigation component**. This GC implementation uses an **EMA (Exponential Moving Average)** filter to smooth out noise. For high-precision requirements, it is recommended to pre-filter the data through a non-linear filter, such as an **Extended Kalman Filter (EKF)**.

All input vectors are immediately **normalized** for use in the PD controllers. While this operation requires a square root calculation (to ensure high precision), it is performed only once upon data reception to save resources.

## PD Controller

*Why not a **PID** controller?* — An integral component can accumulate error (**integral windup**), leading to reduced precision and potential instability. This is especially critical in systems at risk of **actuator saturation** (in our case, the reaction wheels).

The component utilizes three regulator configurations:
- **Magnetorquer-only** torque calculation.
- **Combined torque** (Reaction wheels + Magnetorquers).
- **Reaction wheel-only** torque calculation.

*Why the complexity?* The answer lies in **saturation**...
Reaction wheels allow for high-precision attitude control, but they suffer from a significant drawback: they saturate. When a wheel reaches its **maximum rotational speed**, it can no longer generate torque. 

To counteract this effect (i.e., for **desaturation**), an auxiliary orientation system is required. Since this GC implementation is designed for **Low Earth Orbit (LEO)**, **magnetorquers** (magnetic coils) serve as the auxiliary system.

The GDS sets the minimum and maximum saturation thresholds (as percentages). 
- If wheel saturation is **below the minimum**: Only reaction wheels are used for orientation.
- If it is **between the minimum and maximum**: Torque is calculated for both the wheels and the magnetorquers. The higher the saturation, the more torque is allocated to the magnetorquers to unload (desaturate) the wheels.
- If saturation **exceeds the maximum**: The reaction wheels are disabled entirely. The system switches to [Detumble Mode](#detumble-mode), and only the magnetorquers are responsible for control.

## Operating Modes

This GC implementation provides several operating modes. While the system primarily manages state transitions automatically, they can also be manually controlled via GDS commands.

### Pointing Mode

The primary operational mode, active for approximately 90% of the mission. The GDS provides a target setpoint, and the GC calculates the required torque for the reaction wheels and magnetorquers (if desaturation is required). The controller accounts for both current and target time (in ms) and system parameters to ensure the task is completed precisely by the specified time.

### Detumble Mode (Angular Velocity Hold)

Under normal conditions, this mode is activated only via GDS command. 
**Objective:** Minimize and stabilize angular velocities (rate damping).
**Features:**
- Current target setpoint is ignored.
- Only magnetorquers are utilized as actuators.

### Safe Mode

Activated automatically in case of unrecoverable memory errors (see [ECC](#error-correction-code-ecc)), critical battery levels, or via manual GDS command.
**Objectives:** Prevent uncontrolled rotation, conserve energy, and allow the battery to recharge.
**Features:**
- Current target setpoint is ignored.
- **Sun Pointing:** The satellite aligns the normal of its solar panels with the Sun vector.
- Uses magnetorquers only to minimize power consumption (reaction wheels are disabled).

### Off Mode

All calculations are suspended. In this mode, no processing occurs: input ports are ignored, and no data is written to the output ports.

## Error Correction Code (ECC)

Commonly known as ECC, this is typically a hardware-level feature designed to detect and correct memory errors caused by ionizing radiation (Single Event Upsets) striking the chip. 

This component provides a **software-based implementation** of an ECC container class using a **Majority Voting** principle (Triple Modular Redundancy - TMR).

### Implementation Details
The class stores **three primary copies** and **one bit-inverted (mirrored) copy** of the same value. To maximize reliability, both the copies and the mirror are aligned to 64-bit boundaries (`alignas(64)`). This ensures that each version resides on a **different processor cache line**. Theoretically, this protects the data if a particle strikes the boundary between lines, as it would damage only one copy, which can then be easily restored.

### Performance and Overhead
*   **Memory:** The container introduces significant overhead, occupying 4x more memory than the raw object.
*   **Write Speed:** Each write operation requires four memory writes plus the calculation of the inverted mirror.
*   **Read Speed:** A significant performance hit occurs **only when a mismatch is detected** between the primary copy and the mirror. In this case, the container initiates a recovery procedure to restore the data.

**Recommendation:** Use this container only for variables that must be stored for long periods. For variables that are updated frequently (e.g., every 10ms), protection may be unnecessary as errors will be overwritten naturally. However, if the read frequency significantly exceeds the write frequency, this protection is highly recommended.

### Fault Tolerance
If too many copies are corrupted and the data becomes unrecoverable, the system overwrites the corrupted value with a **default value** provided during the `get` operation. While the GC component does not crash, the corruption of a critical system object will trigger a transition to **Safe Mode**.
