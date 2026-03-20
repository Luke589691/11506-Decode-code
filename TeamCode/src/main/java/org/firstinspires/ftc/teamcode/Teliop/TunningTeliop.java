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
    // SHOOTER POWERS
    // ========================================
    public static final double DEFAULT_SHOOTER_POWER      = 0.6;
    public static final double HUMAN_PLAYER_SHOOTER_POWER = -1.0;
    public static final double HUMAN_PLAYER_INTAKE_POWER  = 0.5;

    // ========================================
    // POWER ADJUSTMENT INCREMENTS
    // ========================================
    public static final double POWER_ADJUST_LARGE = 0.05;
    public static final double POWER_ADJUST_SMALL = 0.01;

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