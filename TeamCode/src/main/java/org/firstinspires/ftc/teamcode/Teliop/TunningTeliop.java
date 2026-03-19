package org.firstinspires.ftc.teamcode.Teliop;

/**
 * TunningTeliop.java
 *
 * All tunable constants for the robot.
 * Edit values here without touching any other file.
 */
public class TunningTeliop {

    // ========================================
    // DRIVE TUNING
    // ========================================
    public static final double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static final double TURN_SPEED_MULTIPLIER  = 0.8;
    public static final double DEADZONE               = 0.05;

    // ========================================
    // SHOOTER PID TUNING
    // ========================================
    // Target velocity in encoder ticks/sec (from DcMotorEx.getVelocity())
    // Increase/decrease to change how fast the shooter spins.
    // Typical range for a bare 6000 RPM motor: 1500–2800 ticks/sec
    public static final double SHOOTER_TARGET_VELOCITY  = 2000.0; // ticks/sec — TUNE THIS

    // Free-spin velocity of your motor at full power (used for feedforward).
    // Run shooter at full power and read getVelocity() from telemetry to find this.
    public static final double SHOOTER_MAX_TICKS_PER_SEC = 2800.0; // TUNE THIS

    // PID gains — see ShooterPID.java for tuning guide
    public static final double SHOOTER_kP = 0.0005; // Start here, raise until it oscillates
    public static final double SHOOTER_kI = 0.0001; // Add slowly to fix steady-state error
    public static final double SHOOTER_kD = 0.0;    // Usually not needed; add to reduce overshoot

    // How close to target velocity counts as "at speed" (ticks/sec)
    public static final double SHOOTER_VELOCITY_TOLERANCE = 50.0;

    // ========================================
    // SHOOTER POWERS (used in human player / fallback modes)
    // ========================================
    public static final double DEFAULT_SHOOTER_POWER      = 0.6;
    public static final double HUMAN_PLAYER_SHOOTER_POWER = -1.0;
    public static final double HUMAN_PLAYER_INTAKE_POWER  = 0.5;

    // ========================================
    // SHOOTER TARGET VELOCITY ADJUSTMENT
    // ========================================
    public static final double VELOCITY_ADJUST_LARGE = 100.0; // Bumper step (ticks/sec)
    public static final double VELOCITY_ADJUST_SMALL = 20.0;  // D-pad step (ticks/sec)
    public static final double VELOCITY_MIN          = 200.0;
    public static final double VELOCITY_MAX          = 2800.0;

    // ========================================
    // SERVO POSITIONS
    // ========================================
    public static final double SERVO_POSITION_LOW  = 0.5;
    public static final double SERVO_POSITION_HIGH = 0.9;
    public static final double TILT_POSITION_DOWN  = 0.1;
    public static final double TILT_POSITION_UP    = 0.7;

    // ========================================
    // RAPID SHOOT TIMING (milliseconds)
    // ========================================
    public static final long RAPID_SHOOT_CYCLE_TIME = 500;
    public static final long RAPID_SHOOT_BURST_TIME  = 200;

    // ========================================
    // HARDWARE MAP NAMES
    // ========================================
    public static final String MOTOR_SHOOTER_LEFT  = "shooterLeft";
    public static final String MOTOR_SHOOTER_RIGHT = "shooterRight";
    public static final String MOTOR_INTAKE        = "intakeWheels";

    public static final String MOTOR_FRONT_LEFT  = "frontLeft";
    public static final String MOTOR_FRONT_RIGHT = "frontRight";
    public static final String MOTOR_BACK_LEFT   = "backLeft";
    public static final String MOTOR_BACK_RIGHT  = "backRight";

    public static final String SERVO_TILT_LEFT  = "tiltLeft";
    public static final String SERVO_TILT_RIGHT = "tiltRight";
    public static final String SERVO_STOP       = "stop";
}