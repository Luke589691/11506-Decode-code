package org.firstinspires.ftc.teamcode.Teliop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Nats Teliop", group = "Competition")
public class TacoNats extends LinearOpMode {

    // Hardware
    private DcMotorEx shooterLeft, shooterRight, intakeWheels;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Servo tiltLeft, tiltRight, stop;

    // Input handler
    private GamepadControls controls;

    // Shooter PID controllers (one per motor for independent correction)
    private ShooterPID pidLeft;
    private ShooterPID pidRight;

    // Robot state
    private boolean intakeRunning   = false;
    private boolean humanPlayerMode = false;
    private boolean rapidShootMode  = false;
    private boolean servoHigh       = false;
    private boolean tiltServosUp    = false;

    private int  shooterMode            = 0; // 0: Off, 1: PID
    private int  shooterModeBeforeRapid = 0;
    private long rapidShootStartTime    = 0;

    // Target velocity shared by both motors (adjusted by bumpers/dpad)
    private double targetVelocity    = TunningTeliop.SHOOTER_TARGET_VELOCITY;
    private double intakeWheelsPower = 0.0;

    @Override
    public void runOpMode() {

        // --- Hardware init ---
        shooterLeft  = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_SHOOTER_LEFT);
        shooterRight = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_SHOOTER_RIGHT);
        intakeWheels = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_INTAKE);

        frontLeft  = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, TunningTeliop.MOTOR_BACK_RIGHT);

        tiltLeft  = hardwareMap.get(Servo.class, TunningTeliop.SERVO_TILT_LEFT);
        tiltRight = hardwareMap.get(Servo.class, TunningTeliop.SERVO_TILT_RIGHT);
        stop      = hardwareMap.get(Servo.class, TunningTeliop.SERVO_STOP);

        // --- Motor directions ---
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // --- Run modes ---
        // Shooter uses RUN_WITHOUT_ENCODER so we control power directly (PID is manual)
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Zero power behavior ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Servo starting positions ---
        stop.setPosition(TunningTeliop.SERVO_POSITION_LOW);
        tiltLeft.setPosition(TunningTeliop.TILT_POSITION_DOWN);
        tiltRight.setPosition(TunningTeliop.TILT_POSITION_DOWN);

        // --- PID controllers ---
        pidLeft  = new ShooterPID(TunningTeliop.SHOOTER_kP, TunningTeliop.SHOOTER_kI, TunningTeliop.SHOOTER_kD);
        pidRight = new ShooterPID(TunningTeliop.SHOOTER_kP, TunningTeliop.SHOOTER_kI, TunningTeliop.SHOOTER_kD);

        // --- Gamepad controls ---
        controls = new GamepadControls(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Refresh all gamepad inputs
            controls.update();

            // ========================================
            // DRIVE — Robot-Centric Mecanum
            // ========================================
            double frontLeftPower  = (controls.drive + controls.strafe + controls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double frontRightPower = (controls.drive - controls.strafe - controls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double backLeftPower   = (controls.drive - controls.strafe + controls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double backRightPower  = (controls.drive + controls.strafe - controls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;

            double maxPower = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower),  Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower  /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower   /= maxPower;
                backRightPower  /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // ========================================
            // TILT SERVOS — Triggers
            // ========================================
            if (controls.leftTriggerHeld) {
                tiltLeft.setPosition(TunningTeliop.TILT_POSITION_DOWN);
                tiltRight.setPosition(TunningTeliop.TILT_POSITION_DOWN);
                tiltServosUp = false;
            }
            if (controls.rightTriggerHeld) {
                tiltLeft.setPosition(TunningTeliop.TILT_POSITION_UP);
                tiltRight.setPosition(TunningTeliop.TILT_POSITION_UP);
                tiltServosUp = true;
            }

            // ========================================
            // TARGET VELOCITY ADJUSTMENT — Bumpers & D-pad L/R
            // Replaces old "power adjust" — now adjusts RPM target instead
            // ========================================
            if (controls.pressedLeftBumper && !humanPlayerMode) {
                targetVelocity = Math.max(TunningTeliop.VELOCITY_MIN, targetVelocity - TunningTeliop.VELOCITY_ADJUST_LARGE);
            }
            if (controls.pressedRightBumper && !humanPlayerMode) {
                targetVelocity = Math.min(TunningTeliop.VELOCITY_MAX, targetVelocity + TunningTeliop.VELOCITY_ADJUST_LARGE);
            }
            if (controls.pressedDpadLeft && !humanPlayerMode) {
                targetVelocity = Math.max(TunningTeliop.VELOCITY_MIN, targetVelocity - TunningTeliop.VELOCITY_ADJUST_SMALL);
            }
            if (controls.pressedDpadRight && !humanPlayerMode) {
                targetVelocity = Math.min(TunningTeliop.VELOCITY_MAX, targetVelocity + TunningTeliop.VELOCITY_ADJUST_SMALL);
            }

            // ========================================
            // X — Toggle PID Shooter
            // ========================================
            if (controls.pressedX && !humanPlayerMode && !rapidShootMode) {
                shooterMode = (shooterMode == 1) ? 0 : 1;
                if (shooterMode == 0) {
                    pidLeft.reset();
                    pidRight.reset();
                }
            }

            // ========================================
            // Y — Toggle Stop Servo
            // ========================================
            if (controls.pressedY) {
                servoHigh = !servoHigh;
                stop.setPosition(servoHigh ? TunningTeliop.SERVO_POSITION_HIGH : TunningTeliop.SERVO_POSITION_LOW);
            }

            // ========================================
            // DPAD UP — Rapid Shoot Mode
            // ========================================
            if (controls.pressedDpadUp && !humanPlayerMode) {
                if (!rapidShootMode) {
                    rapidShootMode         = true;
                    shooterModeBeforeRapid = shooterMode;
                    rapidShootStartTime    = System.currentTimeMillis();
                } else {
                    rapidShootMode = false;
                    shooterMode    = shooterModeBeforeRapid;
                }
            }

            // ========================================
            // DPAD DOWN — Human Player Mode
            // ========================================
            if (controls.pressedDpadDown) {
                humanPlayerMode = !humanPlayerMode;
                if (humanPlayerMode) {
                    pidLeft.reset();
                    pidRight.reset();
                    shooterLeft.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                    shooterRight.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                    intakeWheels.setPower(TunningTeliop.HUMAN_PLAYER_INTAKE_POWER);
                    rapidShootMode = false;
                } else {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    intakeWheels.setPower(0);
                }
            }

            // ========================================
            // SHOOTER LOGIC
            // ========================================
            double leftVelocity  = shooterLeft.getVelocity();
            double rightVelocity = shooterRight.getVelocity();

            if (humanPlayerMode) {
                // Raw power — no PID
                shooterLeft.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                shooterRight.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                intakeWheels.setPower(TunningTeliop.HUMAN_PLAYER_INTAKE_POWER);

            } else if (rapidShootMode) {
                // Rapid shoot — PID active if shooter was on, pulse intake
                if (shooterModeBeforeRapid == 1) {
                    pidLeft.setTarget(targetVelocity);
                    pidRight.setTarget(targetVelocity);
                    shooterLeft.setPower(pidLeft.compute(leftVelocity));
                    shooterRight.setPower(pidRight.compute(rightVelocity));
                } else {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                }
                long cycleTime = (System.currentTimeMillis() - rapidShootStartTime) % TunningTeliop.RAPID_SHOOT_CYCLE_TIME;
                intakeWheels.setPower(cycleTime < TunningTeliop.RAPID_SHOOT_BURST_TIME ? -1.0 : 0);

            } else if (shooterMode == 1) {
                // Normal PID mode
                pidLeft.setTarget(targetVelocity);
                pidRight.setTarget(targetVelocity);
                shooterLeft.setPower(pidLeft.compute(leftVelocity));
                shooterRight.setPower(pidRight.compute(rightVelocity));

            } else {
                // Shooter off
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            // ========================================
            // INTAKE — A and B buttons
            // ========================================
            if (!humanPlayerMode && !rapidShootMode) {

                if (controls.pressedA) {
                    if (intakeRunning && intakeWheelsPower == -1.0) {
                        intakeRunning = false;
                        intakeWheels.setPower(0);
                    } else {
                        intakeRunning     = true;
                        intakeWheelsPower = -1.0;
                        intakeWheels.setPower(intakeWheelsPower);
                    }
                }

                if (controls.pressedB) {
                    if (intakeRunning && intakeWheelsPower == 1.0) {
                        intakeRunning = false;
                        intakeWheels.setPower(0);
                    } else {
                        intakeRunning     = true;
                        intakeWheelsPower = 1.0;
                        intakeWheels.setPower(intakeWheelsPower);
                    }
                }

                intakeWheels.setPower(intakeRunning ? intakeWheelsPower : 0);
            }

            // ========================================
            // TELEMETRY
            // ========================================
            boolean atSpeed = Math.abs(leftVelocity  - targetVelocity) < TunningTeliop.SHOOTER_VELOCITY_TOLERANCE
                    && Math.abs(rightVelocity - targetVelocity) < TunningTeliop.SHOOTER_VELOCITY_TOLERANCE;

            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("FL/FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL/BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("=== SHOOTER PID ===", "");
            telemetry.addData("Mode", getShooterModeName());
            telemetry.addData("Target Velocity", "%.0f ticks/s", targetVelocity);
            telemetry.addData("Left  Velocity",  "%.0f ticks/s", leftVelocity);
            telemetry.addData("Right Velocity",  "%.0f ticks/s", rightVelocity);
            telemetry.addData("Left  Error",     "%.0f", pidLeft.getLastError());
            telemetry.addData("Right Error",     "%.0f", pidRight.getLastError());
            telemetry.addData("At Speed",        atSpeed ? "YES ✓" : "spinning up...");
            telemetry.addData("=== INTAKE ===", "");
            telemetry.addData("Power", "%.2f", intakeWheelsPower);
            telemetry.addData("Running", intakeRunning);
            telemetry.addData("=== SERVOS ===", "");
            telemetry.addData("Toggle Servo", servoHigh ? "DOWN" : "UP");
            telemetry.addData("Tilt Servos", tiltServosUp ? "UP (0.7)" : "DOWN (0.0)");
            telemetry.update();
        }
    }

    private String getShooterModeName() {
        if (humanPlayerMode) return "Human Player (raw power)";
        if (rapidShootMode)  return "Rapid Shoot PID (" + (shooterModeBeforeRapid == 1 ? "On" : "Off") + ")";
        return shooterMode == 1 ? "PID Active" : "Off";
    }
}