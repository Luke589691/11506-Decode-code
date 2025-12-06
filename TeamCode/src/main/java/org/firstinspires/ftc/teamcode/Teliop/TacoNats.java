package org.firstinspires.ftc.teamcode.Teliop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Nats Teliop", group = "Competition")
public class TacoNats extends LinearOpMode {

    private Servo tiltLeft;
    private Servo tiltRight;
    private Servo stop;

    // ========================================
    // EASY TUNING AREA - ADJUST THESE VALUES
    // ========================================

    // DRIVE TUNING
    private static final double DRIVE_SPEED_MULTIPLIER = 1.0;    // Normal drive speed (0.0-1.0)
    private static final double TURN_SPEED_MULTIPLIER = 0.8;     // Turning speed multiplier
    private static final double DEADZONE = 0.05;                 // Joystick deadzone to prevent drift

    // MANUAL ADJUSTMENT INCREMENTS
    private static final double POWER_ADJUST_LARGE = 0.05;  // Bumper adjustment
    private static final double POWER_ADJUST_SMALL = 0.01;  // D-pad Left/Right adjustment

    // SERVO TOGGLE POSITIONS
    private static final double SERVO_POSITION_LOW = 0.5;
    private static final double SERVO_POSITION_HIGH = 0.9;

    // RAPID SHOOT TIMING (milliseconds)
    private static final long RAPID_SHOOT_CYCLE_TIME = 500;  // Time between intake bursts
    private static final long RAPID_SHOOT_BURST_TIME = 200;  // How long each intake burst runs

    // DEFAULT POWERS
    private static final double DEFAULT_SHOOTER_POWER = 0.6;
    private static final double HUMAN_PLAYER_SHOOTER_POWER = -1.0;  // Full reverse
    private static final double HUMAN_PLAYER_INTAKE_POWER = 0.5;   // Half speed reverse

    // ========================================
    // END OF TUNING AREA
    // ========================================

    // Hardware
    private DcMotorEx shooterLeft, shooterRight, intakeWheels;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // State Variables
    private boolean lastAPress = false;
    private boolean lastBPress = false;
    private boolean lastXPress = false;
    private boolean lastYPress = false;
    private boolean lastDpadUpPress = false;
    private boolean lastDpadDownPress = false;
    private boolean lastLeftBumperPress = false;
    private boolean lastRightBumperPress = false;
    private boolean lastDpadLeftPress = false;
    private boolean lastDpadRightPress = false;

    private boolean intakeRunning = false;
    private boolean humanPlayerMode = false;
    private boolean rapidShootMode = false;
    private boolean servoHigh = false;

    private int shooterMode = 0; // 0: Off, 1: Manual
    private int shooterModeBeforeRapid = 0; // Store mode before rapid shoot
    private long rapidShootStartTime = 0;

    private double shooterLeftPower = DEFAULT_SHOOTER_POWER;
    private double shooterRightPower = DEFAULT_SHOOTER_POWER;
    private double intakeWheelsPower = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Initialize servos
        tiltRight = hardwareMap.get(Servo.class, "tiltRight");
        tiltLeft = hardwareMap.get(Servo.class, "servoTest");
        stop = hardwareMap.get(Servo.class, "stop");

        // Set motor directions
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        // Set drive motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor run modes
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior to brake for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize toggle servo to low position
        stop.setPosition(SERVO_POSITION_LOW);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "L-Stick: Drive | R-Stick: Turn");
        telemetry.addData("X", "Manual Shooter | Y: Toggle Servo");
        telemetry.addData("Bumpers", "Speed ±0.05 | DPad L/R: ±0.01");
        telemetry.addData("DPad Up", "Rapid Shoot | DPad Down: Human Player");
        telemetry.addData("A/B", "Intake Control");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========================================
            // DRIVE CONTROLS (Robot-Centric Mecanum)
            // ========================================
            double drive = applyDeadzone(gamepad1.left_stick_y);
            double strafe = applyDeadzone(gamepad1.left_stick_x);
            double turn = applyDeadzone(gamepad1.right_stick_x);

            double speedMultiplier = DRIVE_SPEED_MULTIPLIER;

            // Calculate mecanum drive motor powers
            double frontLeftPower = (drive + strafe + turn * TURN_SPEED_MULTIPLIER) * speedMultiplier;
            double frontRightPower = (drive - strafe - turn * TURN_SPEED_MULTIPLIER) * speedMultiplier;
            double backLeftPower = (drive - strafe + turn * TURN_SPEED_MULTIPLIER) * speedMultiplier;
            double backRightPower = (drive + strafe - turn * TURN_SPEED_MULTIPLIER) * speedMultiplier;

            // Normalize wheel powers if any exceed 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // ========================================
            // SHOOTER/INTAKE CONTROLS
            // ========================================

            // LEFT BUMPER: Decrease shooter power by 0.05
            if (gamepad1.left_bumper && !lastLeftBumperPress && !humanPlayerMode) {
                shooterLeftPower -= POWER_ADJUST_LARGE;
                shooterRightPower -= POWER_ADJUST_LARGE;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
            }
            lastLeftBumperPress = gamepad1.left_bumper;

            // RIGHT BUMPER: Increase shooter power by 0.05
            if (gamepad1.right_bumper && !lastRightBumperPress && !humanPlayerMode) {
                shooterLeftPower += POWER_ADJUST_LARGE;
                shooterRightPower += POWER_ADJUST_LARGE;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }
            lastRightBumperPress = gamepad1.right_bumper;

            // DPAD RIGHT: Increase shooter power by 0.01
            if (gamepad1.dpad_right && !lastDpadRightPress && !humanPlayerMode) {
                shooterLeftPower += POWER_ADJUST_SMALL;
                shooterRightPower += POWER_ADJUST_SMALL;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }
            lastDpadRightPress = gamepad1.dpad_right;

            // DPAD LEFT: Decrease shooter power by 0.01
            if (gamepad1.dpad_left && !lastDpadLeftPress && !humanPlayerMode) {
                shooterLeftPower -= POWER_ADJUST_SMALL;
                shooterRightPower -= POWER_ADJUST_SMALL;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
            }
            lastDpadLeftPress = gamepad1.dpad_left;

            // X Button for manual shooter mode
            if (gamepad1.x && !lastXPress && !humanPlayerMode && !rapidShootMode) {
                shooterMode = (shooterMode == 1) ? 0 : 1;
            }
            lastXPress = gamepad1.x;

            // Y Button for servo toggle
            if (gamepad1.y && !lastYPress) {
                servoHigh = !servoHigh;
                if (servoHigh) {
                    stop.setPosition(SERVO_POSITION_HIGH);
                } else {
                    stop.setPosition(SERVO_POSITION_LOW);
                }
            }
            lastYPress = gamepad1.y;

            // DPAD UP: Rapid Shoot Mode (keeps shooter in current mode, pulses intake)
            if (gamepad1.dpad_up && !lastDpadUpPress && !humanPlayerMode) {
                if (!rapidShootMode) {
                    rapidShootMode = true;
                    shooterModeBeforeRapid = shooterMode;
                    rapidShootStartTime = System.currentTimeMillis();
                } else {
                    rapidShootMode = false;
                    shooterMode = shooterModeBeforeRapid;
                }
            }
            lastDpadUpPress = gamepad1.dpad_up;

            // DPAD DOWN: Human Player Mode
            if (gamepad1.dpad_down && !lastDpadDownPress) {
                humanPlayerMode = !humanPlayerMode;
                if (humanPlayerMode) {
                    shooterRightPower = HUMAN_PLAYER_SHOOTER_POWER;
                    shooterLeftPower = HUMAN_PLAYER_SHOOTER_POWER;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                    intakeWheels.setPower(HUMAN_PLAYER_INTAKE_POWER);
                    rapidShootMode = false;
                } else {
                    shooterRightPower = DEFAULT_SHOOTER_POWER;
                    shooterLeftPower = DEFAULT_SHOOTER_POWER;
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    intakeWheels.setPower(0);
                }
            }
            lastDpadDownPress = gamepad1.dpad_down;

            // --- Shooter Logic ---
            if (humanPlayerMode) {
                // Human player mode - shooter and intake already set when mode activated
                shooterRight.setPower(HUMAN_PLAYER_SHOOTER_POWER);
                shooterLeft.setPower(HUMAN_PLAYER_SHOOTER_POWER);
                intakeWheels.setPower(HUMAN_PLAYER_INTAKE_POWER);
            } else if (rapidShootMode) {
                // Rapid shoot mode - maintain shooter mode, pulse intake
                if (shooterMode == 1) {
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                }

                // Pulse intake for rapid shooting
                long elapsed = System.currentTimeMillis() - rapidShootStartTime;
                long cycleTime = elapsed % RAPID_SHOOT_CYCLE_TIME;
                if (cycleTime < RAPID_SHOOT_BURST_TIME) {
                    intakeWheels.setPower(-1.0);  // Intake burst
                } else {
                    intakeWheels.setPower(0);     // Wait
                }
            } else if (shooterMode == 1) {
                // Manual shooter mode
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                // Shooter off
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            // --- Intake Controls (A and B buttons) ---
            if (!humanPlayerMode && !rapidShootMode) {
                // A Button: Intake at -1.0 power (toggle)
                if (gamepad1.a && !lastAPress) {
                    if (intakeRunning && intakeWheelsPower == -1) {
                        intakeRunning = false;
                        intakeWheels.setPower(0);
                    } else {
                        intakeRunning = true;
                        intakeWheelsPower = -1;
                        intakeWheels.setPower(intakeWheelsPower);
                    }
                }
                lastAPress = gamepad1.a;

                // B Button: Intake at 1.0 power (toggle)
                if (gamepad1.b && !lastBPress) {
                    if (intakeRunning && intakeWheelsPower == 1.0) {
                        intakeRunning = false;
                        intakeWheels.setPower(0);
                    } else {
                        intakeRunning = true;
                        intakeWheelsPower = 1.0;
                        intakeWheels.setPower(intakeWheelsPower);
                    }
                }
                lastBPress = gamepad1.b;

                // Apply intake power if running
                if (intakeRunning) {
                    intakeWheels.setPower(intakeWheelsPower);
                } else {
                    intakeWheels.setPower(0);
                }
            }

            // ========================================
            // TELEMETRY
            // ========================================
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("FL/FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL/BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Power", "%.2f", shooterRightPower);
            telemetry.addData("Mode", getShooterModeName());
            telemetry.addData("=== INTAKE ===", "");
            telemetry.addData("Power", "%.2f", intakeWheelsPower);
            telemetry.addData("Running", intakeRunning);
            telemetry.addData("=== SERVO ===", "");
            telemetry.addData("Toggle Servo", servoHigh ? "DOWN" : "UP");
            telemetry.update();
        }
    }

    private String getShooterModeName() {
        if (humanPlayerMode) {
            return "Human Player";
        } else if (rapidShootMode) {
            return "Rapid Shoot (" + (shooterModeBeforeRapid == 1 ? "Manual" : "Off") + ")";
        }

        switch (shooterMode) {
            case 0:
                return "Off";
            case 1:
                return "Manual";
            default:
                return "Unknown";
        }
    }

    // Helper method to apply deadzone to joystick inputs
    private double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) {
            return 0.0;
        }
        return value;
    }
}