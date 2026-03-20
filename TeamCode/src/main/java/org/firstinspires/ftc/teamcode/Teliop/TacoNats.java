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

    // Input handlers
    private GamepadControls driveControls;   // gamepad1 — driver
    private GamepadControls controls;        // gamepad2 — operator

    // Robot state
    private boolean intakeRunning   = false;
    private boolean humanPlayerMode = false;
    private boolean rapidShootMode  = false;
    private boolean servoHigh       = false;
    private boolean tiltServosUp    = false;

    private int shooterMode            = 0;
    private int shooterModeBeforeRapid = 0;
    private long rapidShootStartTime   = 0;

    private double shooterLeftPower  = TunningTeliop.DEFAULT_SHOOTER_POWER;
    private double shooterRightPower = TunningTeliop.DEFAULT_SHOOTER_POWER;
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

        // --- Gamepad controls ---
        driveControls = new GamepadControls(gamepad1); // Driver
        controls      = new GamepadControls(gamepad2); // Operator

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Refresh all gamepad inputs
            driveControls.update();
            controls.update();

            // ========================================
            // DRIVE — gamepad1, Robot-Centric Mecanum
            // ========================================
            double frontLeftPower  = (driveControls.drive + driveControls.strafe + driveControls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double frontRightPower = (driveControls.drive - driveControls.strafe - driveControls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double backLeftPower   = (driveControls.drive - driveControls.strafe + driveControls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;
            double backRightPower  = (driveControls.drive + driveControls.strafe - driveControls.turn * TunningTeliop.TURN_SPEED_MULTIPLIER) * TunningTeliop.DRIVE_SPEED_MULTIPLIER;

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
            // TILT SERVOS — gamepad2 Triggers
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
            // SHOOTER POWER ADJUSTMENT — gamepad2
            // ========================================
            if (controls.pressedLeftBumper && !humanPlayerMode) {
                shooterLeftPower  = Math.max(0.0, shooterLeftPower  - TunningTeliop.POWER_ADJUST_LARGE);
                shooterRightPower = Math.max(0.0, shooterRightPower - TunningTeliop.POWER_ADJUST_LARGE);
            }
            if (controls.pressedRightBumper && !humanPlayerMode) {
                shooterLeftPower  = Math.min(1.0, shooterLeftPower  + TunningTeliop.POWER_ADJUST_LARGE);
                shooterRightPower = Math.min(1.0, shooterRightPower + TunningTeliop.POWER_ADJUST_LARGE);
            }
            if (controls.pressedDpadLeft && !humanPlayerMode) {
                shooterLeftPower  = Math.max(0.0, shooterLeftPower  - TunningTeliop.POWER_ADJUST_SMALL);
                shooterRightPower = Math.max(0.0, shooterRightPower - TunningTeliop.POWER_ADJUST_SMALL);
            }
            if (controls.pressedDpadRight && !humanPlayerMode) {
                shooterLeftPower  = Math.min(1.0, shooterLeftPower  + TunningTeliop.POWER_ADJUST_SMALL);
                shooterRightPower = Math.min(1.0, shooterRightPower + TunningTeliop.POWER_ADJUST_SMALL);
            }

            // ========================================
            // X — Toggle Manual Shooter — gamepad2
            // ========================================
            if (controls.pressedX && !humanPlayerMode && !rapidShootMode) {
                shooterMode = (shooterMode == 1) ? 0 : 1;
            }

            // ========================================
            // Y — Toggle Stop Servo — gamepad2
            // ========================================
            if (controls.pressedY) {
                servoHigh = !servoHigh;
                stop.setPosition(servoHigh ? TunningTeliop.SERVO_POSITION_HIGH : TunningTeliop.SERVO_POSITION_LOW);
            }

            // ========================================
            // DPAD UP — Rapid Shoot Mode — gamepad2
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
            // DPAD DOWN — Human Player Mode — gamepad2
            // ========================================
            if (controls.pressedDpadDown) {
                humanPlayerMode = !humanPlayerMode;
                if (humanPlayerMode) {
                    shooterLeftPower  = TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER;
                    shooterRightPower = TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER;
                    shooterLeft.setPower(shooterLeftPower);
                    shooterRight.setPower(shooterRightPower);
                    intakeWheels.setPower(TunningTeliop.HUMAN_PLAYER_INTAKE_POWER);
                    rapidShootMode = false;
                } else {
                    shooterLeftPower  = TunningTeliop.DEFAULT_SHOOTER_POWER;
                    shooterRightPower = TunningTeliop.DEFAULT_SHOOTER_POWER;
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    intakeWheels.setPower(0);
                }
            }

            // ========================================
            // SHOOTER LOGIC
            // ========================================
            if (humanPlayerMode) {
                shooterLeft.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                shooterRight.setPower(TunningTeliop.HUMAN_PLAYER_SHOOTER_POWER);
                intakeWheels.setPower(TunningTeliop.HUMAN_PLAYER_INTAKE_POWER);

            } else if (rapidShootMode) {
                shooterLeft.setPower(shooterMode == 1 ? shooterLeftPower   : 0);
                shooterRight.setPower(shooterMode == 1 ? shooterRightPower : 0);

                long cycleTime = (System.currentTimeMillis() - rapidShootStartTime) % TunningTeliop.RAPID_SHOOT_CYCLE_TIME;
                intakeWheels.setPower(cycleTime < TunningTeliop.RAPID_SHOOT_BURST_TIME ? -1.0 : 0);

            } else if (shooterMode == 1) {
                shooterLeft.setPower(shooterLeftPower);
                shooterRight.setPower(shooterRightPower);

            } else {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            // ========================================
            // INTAKE — gamepad2 A and B buttons
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
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("FL/FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL/BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Power", "%.2f", shooterRightPower);
            telemetry.addData("Mode", getShooterModeName());
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
        if (humanPlayerMode) return "Human Player";
        if (rapidShootMode)  return "Rapid Shoot (" + (shooterModeBeforeRapid == 1 ? "Manual" : "Off") + ")";
        return shooterMode == 1 ? "Manual" : "Off";
    }
}