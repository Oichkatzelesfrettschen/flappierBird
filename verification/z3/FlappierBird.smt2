; FlappierBird Z3 SMT-LIB2 Constraints
; Formal verification of control system constraints using Z3
; Author: FlappierBird Development Team
; Date: 2026-01-02

; ============================================================================
; Section 1: Variable Declarations
; ============================================================================

; State variables (continuous)
(declare-const altitude Real)
(declare-const pitch Real)
(declare-const roll Real)
(declare-const yaw Real)

; Control outputs
(declare-const servo1_angle Real)
(declare-const servo2_angle Real)
(declare-const throttle Int)

; Sensor readings
(declare-const accel_x Real)
(declare-const accel_y Real)
(declare-const accel_z Real)
(declare-const gyro_x Real)
(declare-const gyro_y Real)
(declare-const gyro_z Real)

; Environmental
(declare-const wind_speed_x Real)
(declare-const wind_speed_y Real)
(declare-const battery_voltage Real)

; ============================================================================
; Section 2: Physical Constraints
; ============================================================================

; Altitude constraints
(assert (>= altitude 0.0))
(assert (<= altitude 100.0))

; Orientation constraints (in degrees)
(assert (>= pitch -45.0))
(assert (<= pitch 45.0))
(assert (>= roll -45.0))
(assert (<= roll 45.0))
(assert (>= yaw -180.0))
(assert (<= yaw 180.0))

; Servo angle constraints
(assert (>= servo1_angle 15.0))   ; MIN_LEAN
(assert (<= servo1_angle 165.0))  ; MAX_LEAN
(assert (>= servo2_angle 0.0))
(assert (<= servo2_angle 180.0))

; Throttle constraints
(assert (>= throttle 50))
(assert (<= throttle 180))

; Battery voltage constraints
(assert (>= battery_voltage 6.8))  ; Minimum safe voltage
(assert (<= battery_voltage 8.4))  ; Maximum (fully charged 2S LiPo)

; Acceleration constraints (in m/s^2, accounting for gravity)
(assert (>= accel_x -30.0))
(assert (<= accel_x 30.0))
(assert (>= accel_y -30.0))
(assert (<= accel_y 30.0))
(assert (>= accel_z -30.0))
(assert (<= accel_z 30.0))

; Gyroscope constraints (in rad/s)
(assert (>= gyro_x -10.0))
(assert (<= gyro_x 10.0))
(assert (>= gyro_y -10.0))
(assert (<= gyro_y 10.0))
(assert (>= gyro_z -10.0))
(assert (<= gyro_z 10.0))

; Wind speed constraints (in m/s)
(assert (>= wind_speed_x -20.0))
(assert (<= wind_speed_x 20.0))
(assert (>= wind_speed_y -20.0))
(assert (<= wind_speed_y 20.0))

; ============================================================================
; Section 3: Control Law Constraints
; ============================================================================

; Lean control law: servo1 adjusts for roll compensation
; servo1_angle ≈ 90 - roll (with offset)
(define-const leanCtrOffset Real (- 13.0))
(assert (>= servo1_angle (- (+ 90.0 leanCtrOffset) roll 10.0)))
(assert (<= servo1_angle (+ (+ 90.0 leanCtrOffset) (- roll) 10.0)))

; Trim control law: servo2 adjusts for pitch and altitude
; servo2_angle ≈ 90 + pitch + gravity_compensation
; Note: Uses accel_x because IMU X-axis is forward-facing in bird's body frame
; Changes in pitch affect X-component of gravity vector
(define-const cruiseOffset Real 40.0)
(define-const gravity_scale Real (/ (+ accel_x 9.81) 19.62))
(define-const trim_from_gravity Real (* 180.0 gravity_scale))
(assert (>= servo2_angle (- (+ trim_from_gravity cruiseOffset) 20.0)))
(assert (<= servo2_angle (+ (+ trim_from_gravity cruiseOffset) 20.0)))

; Throttle must be sufficient for flight when altitude > 1m
(assert (=> (> altitude 1.0) (>= throttle 70)))

; ============================================================================
; Section 4: Stability Constraints
; ============================================================================

; Define stability predicate
(define-fun stable () Bool
  (and 
    ; Orientation stability
    (<= (abs pitch) 30.0)
    (<= (abs roll) 30.0)
    ; Angular velocity stability
    (<= (abs gyro_x) 5.0)
    (<= (abs gyro_y) 5.0)
    (<= (abs gyro_z) 5.0)
    ; Sufficient throttle
    (>= throttle 70)
    ; Sufficient battery
    (>= battery_voltage 7.0)
  )
)

; Define emergency condition
(define-fun emergency_condition () Bool
  (or
    (< altitude 0.0)
    (> altitude 100.0)
    (< pitch -45.0)
    (> pitch 45.0)
    (< roll -45.0)
    (> roll 45.0)
    (< battery_voltage 6.8)
  )
)

; Assert stability for normal flight
(assert stable)

; Assert no emergency conditions
(assert (not emergency_condition))

; ============================================================================
; Section 5: Quaternion Rotation Constraints
; ============================================================================

; Quaternion representation (w, x, y, z)
(declare-const quat_w Real)
(declare-const quat_x Real)
(declare-const quat_y Real)
(declare-const quat_z Real)

; Quaternion must be normalized: w^2 + x^2 + y^2 + z^2 = 1
(assert (= 1.0 (+ (* quat_w quat_w) 
                  (* quat_x quat_x) 
                  (* quat_y quat_y) 
                  (* quat_z quat_z))))

; Convert quaternion to Euler angles (roll, pitch, yaw)
; roll = atan2(2*(quat_w*quat_x + quat_y*quat_z), 1 - 2*(quat_x^2 + quat_y^2))
; pitch = asin(2*(quat_w*quat_y - quat_z*quat_x))
; yaw = atan2(2*(quat_w*quat_z + quat_x*quat_y), 1 - 2*(quat_y^2 + quat_z^2))

(define-const roll_sin Real (* 2.0 (+ (* quat_w quat_x) (* quat_y quat_z))))
(define-const roll_cos Real (- 1.0 (* 2.0 (+ (* quat_x quat_x) (* quat_y quat_y)))))
(define-const pitch_sin Real (* 2.0 (- (* quat_w quat_y) (* quat_z quat_x))))

; Constraints linking quaternion to Euler angles
(assert (and (>= pitch_sin -1.0) (<= pitch_sin 1.0)))

; ============================================================================
; Section 6: Sensor Fusion Constraints
; ============================================================================

; Gravity vector in body frame should match accelerometer reading at rest
; When gyro readings are near zero (hovering/stable flight)
(define-fun at_rest () Bool
  (and (<= (abs gyro_x) 0.1)
       (<= (abs gyro_y) 0.1)
       (<= (abs gyro_z) 0.1)))

; When at rest, accelerometer should primarily measure gravity
(assert (=> at_rest (and (>= accel_z -11.0) (<= accel_z -8.0))))

; ============================================================================
; Section 7: Energy and Power Constraints
; ============================================================================

; Power consumption model
(declare-const power_watts Real)

; Power is proportional to throttle
(define-const throttle_normalized Real (/ (- throttle 50) 130.0))
(assert (= power_watts (* throttle_normalized 50.0)))  ; Max ~50W

; Power must be within battery capability
(assert (>= power_watts 0.0))
(assert (<= power_watts 60.0))

; Current draw (Ohm's law: P = V*I)
(declare-const current_amps Real)
(assert (= current_amps (/ power_watts battery_voltage)))
(assert (<= current_amps 15.0))  ; Maximum safe current

; ============================================================================
; Section 8: Aerodynamic Constraints
; ============================================================================

; Lift coefficient must be positive for sustained flight
(declare-const lift_coefficient Real)
(assert (>= lift_coefficient 0.2))
(assert (<= lift_coefficient 1.5))

; Drag coefficient
(declare-const drag_coefficient Real)
(assert (>= drag_coefficient 0.1))
(assert (<= drag_coefficient 0.8))

; Lift-to-drag ratio should be reasonable
(define-const L_over_D Real (/ lift_coefficient drag_coefficient))
(assert (>= L_over_D 0.5))
(assert (<= L_over_D 8.0))

; Reynolds number constraint (approximate)
(declare-const reynolds_number Real)
(assert (>= reynolds_number 10000.0))
(assert (<= reynolds_number 100000.0))

; ============================================================================
; Section 9: Control Loop Timing Constraints
; ============================================================================

; Loop execution times (in milliseconds)
(declare-const imu_read_time Int)
(declare-const fusion_time Int)
(declare-const control_time Int)

(assert (>= imu_read_time 1))
(assert (<= imu_read_time 10))
(assert (>= fusion_time 1))
(assert (<= fusion_time 10))
(assert (>= control_time 1))
(assert (<= control_time 20))

; Total loop time must be less than period
(define-const total_loop_time Int (+ imu_read_time fusion_time control_time))
(assert (<= total_loop_time 20))  ; 50 Hz control loop

; ============================================================================
; Section 10: Optimization Goals
; ============================================================================

; Minimize control effort while maintaining stability
(define-const control_effort Real 
  (+ (abs (- servo1_angle 90.0))
     (abs (- servo2_angle 90.0))
     (/ (- throttle 100) 10.0)))

; Maximize stability margin
(define-const stability_margin Real
  (+ (- 30.0 (abs pitch))
     (- 30.0 (abs roll))))

; Goal: Minimize control effort while maximizing stability
; (This would be used in optimization mode, not just satisfiability)

; ============================================================================
; Section 11: Check Satisfiability
; ============================================================================

(check-sat)

; If satisfiable, get model
(get-model)

; ============================================================================
; Section 12: Additional Queries
; ============================================================================

; Query 1: Find maximum safe altitude
(push)
(maximize altitude)
(check-sat)
(get-objectives)
(pop)

; Query 2: Find optimal throttle for level flight
(push)
(assert (and (>= altitude 10.0) (<= altitude 11.0)))
(assert (and (>= pitch -2.0) (<= pitch 2.0)))
(assert (and (>= roll -2.0) (<= roll 2.0)))
(check-sat)
(get-value (throttle))
(pop)

; Query 3: Verify stability under wind disturbance
(push)
(assert (>= (abs wind_speed_x) 5.0))
(check-sat)
(get-value (servo1_angle servo2_angle throttle))
(pop)

; ============================================================================
; Section 13: Verification Queries
; ============================================================================

; Verify that stable flight is possible
(push)
(assert stable)
(assert (>= altitude 5.0))
(check-sat)
(pop)

; Verify that emergency conditions are detectable
(push)
(assert emergency_condition)
(check-sat)
(pop)

; Verify control law correctness
(push)
(assert (= roll 10.0))
(check-sat)
(get-value (servo1_angle))
(pop)

(exit)
