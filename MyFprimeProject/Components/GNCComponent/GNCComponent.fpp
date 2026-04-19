module MyFprimeProject {

    struct Velocity {
        x: F32,
        y: F32,
        z: F32,
    } default {
        x = 0,
        y = 0,
        z = 0,
    }

    struct Position {
        x: F32,
        y: F32,
        z: F32,
    } default {
        x = 0,
        y = 0,
        z = 0,
    }

    struct Rotation {
        x: F32,
        y: F32,
        z: F32,
    } default {
        x = 0,
        y = 0,
        z = 0,
    }

    @ Port for transfer current velocity to GNC component
    port VelocityPort(
        $velocity: Velocity @< Velocity value
    )

    @ Port for transfer current position to GNC component
    port PositionPort(
        $position: Position @< Position value
    )

    @ Port for transfer current rotation to GNC component
    port RotationPort(
        $rotation: Rotation @< Rotation value
    )

    @ Component for Guidance, Navigation and Control of spaceship
    active component GNCComponent {

        @ Stability PID controller: Proportional coefficient
        param STAB_PID_Kp: F32 default 1.0

        @ Stability PID controller: Integral coefficient
        param STAB_PID_Ki: F32 default 0.1

        @ Stability PID controller: Derivative coefficient
        param STAB_PID_Kd: F32 default 0.01


        @ Velocity PID controller: Proportional coefficient
        param VEL_Kp: F32 default 1.1

        @ Velocity PID controller: Integral coefficient
        param VEL_Ki: F32 default 0.2

        @ Velocity PID controller: Derivative coefficient
        param VEL_Kd: F32 default 0.02

        @ Command to change current Rotation to vector x, y, z
        async command SET_ROTATION(
            target_rotation: Rotation @< Target rotation
        )

        @ Rotation event with new target for Rotation
        event RotationEvent(
            target_rotation: Rotation @< Target rotation
        ) severity activity high format "GNC receive new target rotation: {}"


        @ Command to change current position to vector x, y, z
        async command SET_POSITION(
            target_position: Position @< Target position
        )

        @ Position event with new target for position
        event PositionEvent(
            target_position: Position @< Target position
        ) severity activity high format "GNC receive new target position: {}"


        @ Command to change current velocity to vector x, y, z
        async command SET_VELOCITY(
            target_velocity: Velocity @< Target velocity
        )

        @ Velocity event with new target for velocity
        event VelocityEvent(
            target_velocity: Velocity @< Target velocity
        ) severity activity high format "GNC receive new target velocity: {}"


        telemetry TlmCurrentPosition: Position id 1
        telemetry TlmCurrentRotation: Rotation id 2
        telemetry TlmCurrentVelocity: Velocity id 3

        telemetry TlmTargetPosition: Position id 4
        telemetry TlmTargetRotation: Rotation id 5
        telemetry TlmTargetVelocity: Velocity id 6


        sync input port velocityIn: VelocityPort
        sync input port positionIn: PositionPort
        sync input port rotationIn: RotationPort

        sync input port schedIn: Svc.Sched

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