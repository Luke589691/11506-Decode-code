package org.firstinspires.ftc.teamcode.config

/**
 * ============================================================
 *  TUNING CONFIG  —  APOC Bot (DECODE Game)
 * ============================================================
 *  All robot-tunable constants live here.
 *  Fill in YOUR gear ratios when you have them.
 * ============================================================
 */
object TuningConfig {

    // ════════════════════════════════════════════════════════
    //  DRIVE  (Swift V2 / Meconium)
    // ════════════════════════════════════════════════════════

    /** Max drive speed scalar  (0.0 – 1.0) */
    const val DRIVE_MAX_SPEED       = 0.85

    /** Slow-mode speed scalar */
    const val DRIVE_SLOW_SPEED      = 0.35

    /** Strafe power correction factor (tune until strafing is straight) */
    const val DRIVE_STRAFE_CORRECT  = 1.05


    // ════════════════════════════════════════════════════════
    //  INTAKE
    // ════════════════════════════════════════════════════════

    const val INTAKE_SPEED          = 1.0
    const val INTAKE_REVERSE_SPEED  = -0.6


    // ════════════════════════════════════════════════════════
    //  TURRET SPIN
    // ════════════════════════════════════════════════════════

    /**
     *  TODO: Fill in once you have the gear ratio.
     *  TICKS_PER_DEGREE = (motor encoder ticks/rev × gear ratio) / 360
     *  Example: GoBILDA 312rpm = 537.7 ticks/rev
     *           If gear ratio is 5:1  →  537.7 * 5 / 360  ≈  7.47
     */
    const val TURRET_GEAR_RATIO          = 1.0   // ← REPLACE with your ratio
    const val TURRET_MOTOR_TICKS_PER_REV = 537.7 // ← Change for your motor
    val   TURRET_TICKS_PER_DEGREE        = (TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0

    /** Soft limits for 360 ° of travel (encoder ticks from home) */
    const val TURRET_MIN_TICKS = 0
    val   TURRET_MAX_TICKS     = (TURRET_TICKS_PER_DEGREE * 360).toInt()

    // PID – turret position loop
    const val TURRET_KP = 0.005   // proportional
    const val TURRET_KI = 0.0     // integral
    const val TURRET_KD = 0.0002  // derivative
    const val TURRET_MAX_POWER  = 0.6
    const val TURRET_TOLERANCE_TICKS = 10   // "close enough" deadband

    // HuskyLens tracking
    /** Horizontal FOV of HuskyLens in degrees (check your lens spec; ~52 ° typical) */
    const val HUSKY_HFOV_DEG       = 52.0
    /** P-gain for turret auto-track (pixels of error → power) */
    const val TURRET_TRACK_KP      = 0.003
    /** Dead-band: ignore tracking error smaller than this many pixels */
    const val TURRET_TRACK_DEAD_PX = 10.0
    /** HuskyLens frame centre X (pixels) */
    const val HUSKY_FRAME_CENTER_X = 160.0


    // ════════════════════════════════════════════════════════
    //  TURRET HOOD (pivot servo – 20 ° total travel)
    // ════════════════════════════════════════════════════════

    /**
     *  TODO: Fill in once you have the servo gear ratio / linkage ratio.
     *  Map degrees (0 – 20) to servo position (0.0 – 1.0).
     *
     *  If the servo output shaft directly drives the hood:
     *    servoPos = HOOD_SERVO_MIN + (angleDeg / 20.0) * (HOOD_SERVO_MAX - HOOD_SERVO_MIN)
     */
    const val HOOD_SERVO_MIN      = 0.30  // ← servo position at 0 °  (tune me)
    const val HOOD_SERVO_MAX      = 0.70  // ← servo position at 20 ° (tune me)
    const val HOOD_ANGLE_MIN_DEG  = 0.0
    const val HOOD_ANGLE_MAX_DEG  = 20.0


    // ════════════════════════════════════════════════════════
    //  SHOOTER
    // ════════════════════════════════════════════════════════

    /** Full-speed shooter target (0.0 – 1.0) */
    const val SHOOTER_SPEED         = 0.90

    /** Idle / spin-down speed */
    const val SHOOTER_IDLE_SPEED    = 0.0

    /** Velocity tolerance for "up to speed" check (ticks/sec – if using encoders) */
    const val SHOOTER_VELOCITY_TOL  = 50.0
}
