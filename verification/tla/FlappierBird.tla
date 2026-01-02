---- MODULE FlappierBird ----
(*
  TLA+ Specification for Flappier Bird Ornithopter Control System
  
  This specification models the state machine and safety properties
  of the autonomous ornithopter control system.
  
  Author: FlappierBird Development Team
  Date: 2026-01-02
*)

EXTENDS Naturals, Reals, Sequences, TLC

CONSTANTS
    MIN_ALTITUDE,           \* Minimum safe altitude (meters)
    MAX_ALTITUDE,           \* Maximum safe altitude (meters)
    MAX_PITCH_ANGLE,        \* Maximum pitch angle (degrees)
    MAX_ROLL_ANGLE,         \* Maximum roll angle (degrees)
    MIN_LEAN,               \* Minimum servo1 lean angle
    MAX_LEAN,               \* Maximum servo1 lean angle
    SAFE_THROTTLE_MIN,      \* Minimum safe throttle
    SAFE_THROTTLE_MAX       \* Maximum safe throttle

VARIABLES
    altitude,               \* Current altitude in meters
    pitch,                  \* Current pitch angle in degrees
    roll,                   \* Current roll angle in degrees
    yaw,                    \* Current yaw angle in degrees
    throttle,               \* Motor throttle (0-180)
    servo1_angle,           \* Lean control servo angle
    servo2_angle,           \* Trim control servo angle
    accel_x,                \* IMU acceleration X
    accel_y,                \* IMU acceleration Y
    accel_z,                \* IMU acceleration Z
    gyro_x,                 \* Gyroscope reading X
    gyro_y,                 \* Gyroscope reading Y
    gyro_z,                 \* Gyroscope reading Z
    battery_voltage,        \* Battery voltage
    system_state,           \* Current system state
    imu_calibrated,         \* IMU calibration status
    radio_connected,        \* Radio connection status
    emergency_stop_active   \* Emergency stop flag

vars == <<altitude, pitch, roll, yaw, throttle, servo1_angle, servo2_angle,
          accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
          battery_voltage, system_state, imu_calibrated, 
          radio_connected, emergency_stop_active>>

-----------------------------------------------------------------------------

(*
  Type Invariants - Define the valid ranges for all variables
*)

TypeOK ==
    /\ altitude \in Real
    /\ pitch \in Real
    /\ roll \in Real
    /\ yaw \in Real
    /\ throttle \in SAFE_THROTTLE_MIN..SAFE_THROTTLE_MAX
    /\ servo1_angle \in MIN_LEAN..MAX_LEAN
    /\ servo2_angle \in 0..180
    /\ accel_x \in Real
    /\ accel_y \in Real
    /\ accel_z \in Real
    /\ gyro_x \in Real
    /\ gyro_y \in Real
    /\ gyro_z \in Real
    /\ battery_voltage \in Real
    /\ system_state \in {"INIT", "CALIBRATING", "ARMED", "FLYING", "LANDING", "EMERGENCY"}
    /\ imu_calibrated \in BOOLEAN
    /\ radio_connected \in BOOLEAN
    /\ emergency_stop_active \in BOOLEAN

(*
  Safety Invariants - Properties that must always hold
*)

SafetyInvariant ==
    /\ altitude >= MIN_ALTITUDE
    /\ altitude <= MAX_ALTITUDE
    /\ pitch >= -MAX_PITCH_ANGLE
    /\ pitch <= MAX_PITCH_ANGLE
    /\ roll >= -MAX_ROLL_ANGLE
    /\ roll <= MAX_ROLL_ANGLE
    /\ battery_voltage >= 6.8  \* Minimum safe battery voltage
    /\ throttle >= SAFE_THROTTLE_MIN
    /\ throttle <= SAFE_THROTTLE_MAX

(*
  Emergency Condition Detection
*)

EmergencyCondition ==
    \/ altitude < MIN_ALTITUDE
    \/ altitude > MAX_ALTITUDE
    \/ pitch < -MAX_PITCH_ANGLE
    \/ pitch > MAX_PITCH_ANGLE
    \/ roll < -MAX_ROLL_ANGLE
    \/ roll > MAX_ROLL_ANGLE
    \/ battery_voltage < 6.8
    \/ ~imu_calibrated
    \/ ~radio_connected

-----------------------------------------------------------------------------

(*
  Initial State
*)

Init ==
    /\ altitude = 0.0
    /\ pitch = 0.0
    /\ roll = 0.0
    /\ yaw = 0.0
    /\ throttle = SAFE_THROTTLE_MIN
    /\ servo1_angle = 90
    /\ servo2_angle = 90
    /\ accel_x = 0.0
    /\ accel_y = 0.0
    /\ accel_z = -9.81  \* Gravity at rest
    /\ gyro_x = 0.0
    /\ gyro_y = 0.0
    /\ gyro_z = 0.0
    /\ battery_voltage = 8.4  \* Fully charged 2S LiPo
    /\ system_state = "INIT"
    /\ imu_calibrated = FALSE
    /\ radio_connected = FALSE
    /\ emergency_stop_active = FALSE

-----------------------------------------------------------------------------

(*
  State Transitions
*)

\* Initialize system and begin calibration
StartCalibration ==
    /\ system_state = "INIT"
    /\ battery_voltage >= 7.4
    /\ system_state' = "CALIBRATING"
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, servo1_angle, 
                   servo2_angle, accel_x, accel_y, accel_z, gyro_x, gyro_y, 
                   gyro_z, battery_voltage, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Complete IMU calibration
CompleteCalibration ==
    /\ system_state = "CALIBRATING"
    /\ imu_calibrated' = TRUE
    /\ system_state' = "ARMED"
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, servo1_angle, 
                   servo2_angle, accel_x, accel_y, accel_z, gyro_x, gyro_y, 
                   gyro_z, battery_voltage, radio_connected, 
                   emergency_stop_active>>

\* Establish radio connection
EstablishRadio ==
    /\ system_state \in {"CALIBRATING", "ARMED"}
    /\ radio_connected' = TRUE
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, servo1_angle, 
                   servo2_angle, accel_x, accel_y, accel_z, gyro_x, gyro_y, 
                   gyro_z, battery_voltage, system_state, imu_calibrated, 
                   emergency_stop_active>>

\* Transition to flying state
StartFlying ==
    /\ system_state = "ARMED"
    /\ imu_calibrated = TRUE
    /\ radio_connected = TRUE
    /\ battery_voltage >= 7.4
    /\ ~emergency_stop_active
    /\ system_state' = "FLYING"
    /\ throttle' \in (SAFE_THROTTLE_MIN+20)..SAFE_THROTTLE_MAX
    /\ UNCHANGED <<altitude, pitch, roll, yaw, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Read IMU sensor data
ReadIMU ==
    /\ system_state \in {"FLYING", "LANDING"}
    /\ imu_calibrated = TRUE
    /\ accel_x' \in Real
    /\ accel_y' \in Real
    /\ accel_z' \in Real
    /\ gyro_x' \in Real
    /\ gyro_y' \in Real
    /\ gyro_z' \in Real
    /\ pitch' \in [pitch - 5.0 .. pitch + 5.0]
    /\ roll' \in [roll - 5.0 .. roll + 5.0]
    /\ yaw' \in [yaw - 5.0 .. yaw + 5.0]
    /\ UNCHANGED <<altitude, throttle, servo1_angle, servo2_angle, 
                   battery_voltage, system_state, imu_calibrated, 
                   radio_connected, emergency_stop_active>>

\* Update control surfaces based on orientation
ControlSurfaces ==
    /\ system_state = "FLYING"
    /\ ~emergency_stop_active
    /\ servo1_angle' \in MIN_LEAN..MAX_LEAN
    /\ servo2_angle' \in 0..180
    /\ \* Lean control: servo1_angle ≈ 90 - roll
       /\ servo1_angle' >= 90 - roll - 10
       /\ servo1_angle' <= 90 - roll + 10
    /\ \* Trim control: servo2_angle ≈ 90 + pitch
       /\ servo2_angle' >= 90 + pitch - 10
       /\ servo2_angle' <= 90 + pitch + 10
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, accel_x, accel_y, 
                   accel_z, gyro_x, gyro_y, gyro_z, battery_voltage, 
                   system_state, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Update throttle
AdjustThrottle ==
    /\ system_state = "FLYING"
    /\ ~emergency_stop_active
    /\ throttle' \in SAFE_THROTTLE_MIN..SAFE_THROTTLE_MAX
    /\ UNCHANGED <<altitude, pitch, roll, yaw, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, system_state, imu_calibrated, 
                   radio_connected, emergency_stop_active>>

\* Update altitude based on throttle and pitch
UpdateAltitude ==
    /\ system_state = "FLYING"
    /\ altitude' \in [altitude - 2.0 .. altitude + 2.0]
    /\ altitude' >= 0.0
    /\ UNCHANGED <<pitch, roll, yaw, throttle, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, system_state, imu_calibrated, 
                   radio_connected, emergency_stop_active>>

\* Battery discharge
BatteryDischarge ==
    /\ system_state \in {"FLYING", "LANDING"}
    /\ battery_voltage' \in [battery_voltage - 0.1 .. battery_voltage]
    /\ battery_voltage' >= 0.0
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, servo1_angle, 
                   servo2_angle, accel_x, accel_y, accel_z, gyro_x, gyro_y, 
                   gyro_z, system_state, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Initiate landing
StartLanding ==
    /\ system_state = "FLYING"
    /\ \/ battery_voltage < 7.0
       \/ altitude > MAX_ALTITUDE - 10.0
    /\ system_state' = "LANDING"
    /\ UNCHANGED <<altitude, pitch, roll, yaw, throttle, servo1_angle, 
                   servo2_angle, accel_x, accel_y, accel_z, gyro_x, gyro_y, 
                   gyro_z, battery_voltage, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Complete landing
CompleteLanding ==
    /\ system_state = "LANDING"
    /\ altitude <= 0.5
    /\ throttle' = SAFE_THROTTLE_MIN
    /\ system_state' = "ARMED"
    /\ UNCHANGED <<altitude, pitch, roll, yaw, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, imu_calibrated, radio_connected, 
                   emergency_stop_active>>

\* Emergency stop procedure
EmergencyStop ==
    /\ system_state # "EMERGENCY"
    /\ EmergencyCondition
    /\ system_state' = "EMERGENCY"
    /\ throttle' = SAFE_THROTTLE_MIN
    /\ emergency_stop_active' = TRUE
    /\ UNCHANGED <<altitude, pitch, roll, yaw, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, imu_calibrated, radio_connected>>

\* Reset from emergency (manual intervention required)
ResetEmergency ==
    /\ system_state = "EMERGENCY"
    /\ ~EmergencyCondition
    /\ emergency_stop_active = TRUE
    /\ system_state' = "INIT"
    /\ throttle' = SAFE_THROTTLE_MIN
    /\ emergency_stop_active' = FALSE
    /\ UNCHANGED <<altitude, pitch, roll, yaw, servo1_angle, servo2_angle, 
                   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                   battery_voltage, imu_calibrated, radio_connected>>

-----------------------------------------------------------------------------

(*
  Next State Relation
*)

Next ==
    \/ StartCalibration
    \/ CompleteCalibration
    \/ EstablishRadio
    \/ StartFlying
    \/ ReadIMU
    \/ ControlSurfaces
    \/ AdjustThrottle
    \/ UpdateAltitude
    \/ BatteryDischarge
    \/ StartLanding
    \/ CompleteLanding
    \/ EmergencyStop
    \/ ResetEmergency

(*
  Specification
*)

Spec == Init /\ [][Next]_vars

-----------------------------------------------------------------------------

(*
  Temporal Properties
*)

\* The system should always satisfy safety constraints
AlwaysSafe == []SafetyInvariant

\* If the system is armed with good battery, it should eventually fly
EventuallyFlying == 
    [](system_state = "ARMED" /\ imu_calibrated /\ radio_connected /\ battery_voltage >= 7.4
       => <>(system_state = "FLYING"))

\* If in emergency, the throttle should be at minimum
EmergencyThrottleMin == 
    [](system_state = "EMERGENCY" => throttle = SAFE_THROTTLE_MIN)

\* System should not deadlock
NoDeadlock == []<>ENABLED Next

\* If battery is low, system should eventually land or enter emergency
LowBatteryLanding == 
    [](battery_voltage < 7.0 /\ system_state = "FLYING" 
       => <>(system_state \in {"LANDING", "EMERGENCY"}))

\* Once calibrated, IMU should stay calibrated unless in emergency
CalibrationPersistence == 
    [](imu_calibrated /\ system_state # "EMERGENCY" => []imu_calibrated)

-----------------------------------------------------------------------------

(*
  Verification Theorems
*)

THEOREM Spec => AlwaysSafe
THEOREM Spec => EmergencyThrottleMin
THEOREM Spec => NoDeadlock

====
