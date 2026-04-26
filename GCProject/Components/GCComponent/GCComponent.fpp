module GCProject {

    struct Vec3 {
        x: F32
        y: F32
        z: F32
    }

    struct Vec7 {
        quaterion: Quaternion @< Spacecraft orientation
        drifting: Vec3 @< Sensor drifting
    }

    struct Quaternion {
        w: F32
        x: F32
        y: F32
        z: F32
    }

    struct GyroData {
        angular_rates: Vec3 @< Data from gyroscops (or other sensor which provide current angular velocity) with current angular velocity by X, Y and Z axis
        timestamp: U32 @< What time this data has been taken from gyroscops. Time should be provided from last data take
    }

    struct OrientData {
        orientation: Quaternion @< Data from star tracker (or other sensors wich provide current orientation in space) casted to quaternion
        timestamp: U32 @< What time this data has been taken from sensors. Time should be provided from last data take
    }

    @ Way point for provided time
    struct WayPoint {
        target_time: U64 @< Target moment (ms) of mission execution
        angular_velocity: Vec3 @< Target angular velocity in this moment
        orientation: Quaternion @< Target orientation (quaterion) in this moment
    }

    struct BatteryState {
        SoC: F32 @< State of Charge battery
        voltage: F32
        current_amperage: F32 @< Current battery's ampere
        status: BatteryStatus
    }

    enum BatteryStatus {
        @ Battery level is normal
        OK

        @ Battery level is low
        LOW

        @ Battery level is critical
        CRITICAL

        @ Battery on charging
        CHARGE
    }

    enum GncMode {
        @ GNC is OFF
        OFF

        @ Damping of angular velocities
        DETUMBLE

        @ Guidance to the target (following way point)
        POINTING
        
        @ GNC in Safe Mode:
        @ 1. Orientation solar panels of the spacecraft to Sun
        @ 2. Damping of angular velocities
        SAFE
    }

    @ Port for transfer data from gyroscop to GNC component \
    @ GNC uses this data as current angular velocity for EKF prediction
    port GyroSensorDataPort(
        data: GyroData
        is_valid: bool
    )

    @ Port for transfer data from star tracker to GNC component. \
    @ GNC use this data as current orientation
    port StarSensorDataPort(
        data: OrientData
        is_valid: bool
    )

    @ Port for transfer torque calculated by GNC component
    port TorqueCommandPort(
        torque: Vec3
    )

    @ Port for transfer moment to magnestic calculated by GNC component
    port MagnesticCommandPort(
        moment: Vec3
    )

    @ Port for transfer vector to Sun from sensor/tracker
    port SunVectorPort(
        position: Vec3
    )

    @ Port for transfer normalize vector of all sun panels on the spacecraft
    port SunPanelsNormalPort(
        normal: Vec3
    )

    @ Port for transfer battery state.
    @ GNC use this state for managing system
    port BatteryStatePort(
        battery_state: BatteryState
    )

    @ Port for transfer magnetometor data
    port MagnesticDataPort(
        data: Vec3
    )

    @ Port for transfer type of out from GNC component (magnestic or not)
    port IsMagnesticCommandPort(
        is_magnestic: bool
    )

    @ Port for transfer current RPM of reaction wheels
    port ReactionWheelsRPMDataPort(
        data: Vec3
    )

    @ Component for Guidance, Navigation and Control of spaceship
    active component GCComponent {

        @ Proportial coefficient for PD controller
        param PGain: Vec3

        @ Derivative coefficient for PD controller
        param DGain: Vec3

        @ Desaturation K coefficient
        param KDesat: F32

        @ Max Torque which GNC can set to reaction wheels
        param MaxTorque: F32

        param MinBatterySoC: F32

        param MinBatteryVoltage: F32

        param MinBatteryAmpere: F32

        param EMA_Alpha: F32

        param TimeToStabilizationInSafeMode_Ms: U32

        @ Maximum saturation of reaction wheels in percent (0-100%)
        param MaxReactionWheelsSaturationPercent: U32

        @ Minimum saturation of reaction wheels in percent (0-100%)
        param MinReactionWheelsSaturationPercent: U32

        @ Max Rotation Per Minute (RPM) of reaction wheels
        param ReactionWheelsMaxRPM:         F32

        # ---------------- Commands and Events ---------------

        @ Command to set target state for specific time in ms by start of the mission
        async command SET_TARGET_WAYPOINT(
            target_state: WayPoint
        )

        @ Event for SET_TARGET_STATE command. Call on receive command
        event SetTargetStateEvent(
            target_state: WayPoint
        ) severity activity high format "GNC receive target state: {}"


        async command SET_MODE(
            mode: GncMode
        )

        event SetModeEvent(
            mode: GncMode
        ) severity activity high format "GNC receive new mode: {}"

        # ----------- Way Point Telementry ---------

        telemetry TlmCurrentWayPoint:               WayPoint id 1
        telemetry TlmTargetWayPoint:                WayPoint id 2

        # ---------- Error Correction Code Telemetry ----------

        @ How many corrections made by ECC in GNC component
        telemetry TlmECCurrentMemoryCorrections:    U32 id 3

        # ---------- PD Controller Telemetry ----------

        @ Error by angles
        telemetry TlmAngleError:                 Vec3 id 4

        @ Error by angular velocity
        telemetry TlmAngularVelocityError:       Vec3 id 5

        # ---------- General Telemetry -----------

        @ Last provided torque to reaction wheels
        telemetry TlmControlTorque:              Vec3 id 6

        @ Current GNC Mode
        telemetry TlmCurrentGncMode:             GncMode id 7

        @ Is Sun lost
        telemetry TlmSunLost:                     bool id 8

        @ Last provided momemnt to magnestic
        telemetry TlmControlMagnesticMoment:      Vec3 id 9

        @ Is reaction wheels in unload state
        telemetry TlmIsReactionWheelsOnUnload:    bool id 10

        # --------------- Input Ports ----------------

        @ Port for transfer data from gyroscop to GNC component. \
        @ GNC uses this data as current angular velocity for EKF prediction
        sync input port GyroDataIn:         GyroSensorDataPort

        @ Port for transfer data from star tracker to GNC component. \
        @ GNC use this data as current orientation
        sync input port StarTrackerDataIn:  StarSensorDataPort

        @ Port for transfer vector to Sun from sensor/tracker
        sync input port SunVectorIn:        SunVectorPort

        @ Port for transfer normalize vector of all sun panels on the spacecraft
        sync input port SunPanelsNormalIn:  SunPanelsNormalPort

        @ Port for transfer battery state.
        @ GNC use this state for managing system
        sync input port BatteryStateIn:     BatteryStatePort

        @ Port for transfer magnetometor data
        sync input port MagnesticData:      MagnesticDataPort

        @ Port for transfer current RPM of reaction wheels
        sync input port ReactionWheelsRPMData:  ReactionWheelsRPMDataPort

        @ GNC Tick Port. Should called every 10 ms
        sync input port schedIn:            Svc.Sched

        # -------------- Output Ports ---------------

        @ Port for transfer torque calculated by GNC component
        output port TorqueOut:      TorqueCommandPort

        @ Port for transfer moment to magnestic calculated by GNC component
        output port MagnesticOut:   MagnesticCommandPort

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################        
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}