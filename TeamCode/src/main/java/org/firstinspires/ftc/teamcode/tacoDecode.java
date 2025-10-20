package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "tacoDecode", group = "Competition")
public class tacoDecode extends LinearOpMode {

    // Vision and AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Hardware
    private DcMotorEx shooterLeft, shooterRight, intakeWheels;
    private Servo Limelightspin, Limelighttilt;
    private ColorSensor colorSensor;

    // State Variables
    private boolean lastAPress = false;
    private boolean lastBPress = false;
    private boolean lastXPress = false;
    private boolean lastYPress = false;
    private boolean intakeRunning = false;
    private boolean humanPlayerMode = false;
    private boolean autoAimEnabled = false;
    private boolean batteryCompensationEnabled = false;
    private boolean limelightTrackingEnabled = false;
    private boolean intakeSequenceRunning = false;
    private boolean ballDetected = false;

    private int shooterMode = 0; // 0: Off, 1: Auto, 2: Manual, 3: Auto-Aim, 4: Auto-Aim with Comp
    private int intakeSequenceStep = 0;
    private int ballCount = 0;
    private long intakeSequenceStartTime = 0;

    private double shooterLeftPower = 0.6;
    private double shooterRightPower = 0.6;
    private double intakeWheelsPower = 0.0;
    private double targetDistance = 0.0;
    private double spinPosition = 0.5;
    private double tiltPosition = 0.1;

    // Smoothing for Limelight Tracking
    private double smoothedXError = 0.0;
    private double smoothedYError = 0.0;
    private static final double SMOOTHING_FACTOR = 0.1;
    private static final double DEADBAND = 0.02;

    // Servo and Motor Constants
    private static final double SPIN_GAIN = 0.1;
    private static final double TILT_GAIN = 0.005;
    private static final double MIN_SPEED = 0.05;
    private static final double MAX_SPEED = 0.3;
    private static final double TILT_MIN = 0.0;
    private static final double TILT_MAX = 0.2;

    // Physical Constants
    private static final double APRILTAG_HEIGHT = 0.33; // Height of AprilTag center from ground (meters)
    private static final double SHOOTER_HEIGHT = 0.40;  // Height of shooter muzzle from ground (meters)
    private static final double TARGET_HEIGHT_ABOVE_TAG = 0.30; // Aiming 30cm above tag center

    // Battery Compensation
    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final double MIN_VOLTAGE = 11.0;

    // Color Sensor
    private static final int colorThreshold = 1000; // Calibrate this value

    @Override
    public void runOpMode() {
        // Initialize hardware
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        Limelightspin = hardwareMap.get(Servo.class, "Limelightspin");
        Limelighttilt = hardwareMap.get(Servo.class, "Limelighttilt");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Limelightspin.setPosition(0.5);
        Limelighttilt.setPosition(tiltPosition);

        // Initialize vision
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            // --- Gamepad Controls ---
            if (gamepad1.dpadUpWasPressed() && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= 0.05;
                shooterRightPower -= 0.05;
                if (shooterLeftPower < 0.0) shooterLeftPower = 1.0;
                if (shooterRightPower < 0.0) shooterRightPower = 1.0;
            }

            if (gamepad1.dpadDownWasPressed() && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += 0.05;
                shooterRightPower += 0.05;
                if (shooterLeftPower > 1.0) shooterLeftPower = 0.0;
                if (shooterRightPower > 1.0) shooterRightPower = 0.0;
            }

            if (gamepad1.dpadLeftWasPressed() && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= 0.01;
                shooterRightPower -= 0.01;
                if (shooterLeftPower < 0.0) shooterLeftPower = 1.0;
                if (shooterRightPower < 0.0) shooterRightPower = 1.0;
            }

            if (gamepad1.dpadRightWasPressed() && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += 0.01;
                shooterRightPower += 0.01;
                if (shooterLeftPower > 1.0) shooterLeftPower = 0.0;
                if (shooterRightPower > 1.0) shooterRightPower = 0.0;
            }

            // X Button for manual shooter mode
            if (gamepad1.x && !lastXPress && !humanPlayerMode) {
                shooterMode = (shooterMode == 2) ? 0 : 2;
                autoAimEnabled = false;
            }
            lastXPress = gamepad1.x;

            // Y Button for auto-aim mode
            if (gamepad1.y && !lastYPress && !humanPlayerMode) {
                if (autoAimEnabled) {
                    autoAimEnabled = false;
                    limelightTrackingEnabled = false;
                    shooterMode = 0;
                    Limelightspin.setPosition(0.5); // Stop spinning
                } else {
                    autoAimEnabled = true;
                    batteryCompensationEnabled = false;
                    limelightTrackingEnabled = true;
                    shooterMode = 3;
                }
            }
            lastYPress = gamepad1.y;

            // --- Shooter Logic ---
            if (autoAimEnabled) { // Covers modes 3 and 4
                double calculatedPower = calculateShooterPower();
                if (shooterMode == 4 && batteryCompensationEnabled) {
                    calculatedPower = applyBatteryCompensation(calculatedPower);
                }

                if (calculatedPower > 0) {
                    shooterLeftPower = calculatedPower;
                    shooterRightPower = calculatedPower;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                    if (limelightTrackingEnabled) {
                        trackAprilTagWithLimelight();
                    }
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    if (limelightTrackingEnabled) {
                        Limelightspin.setPosition(0.5); // Stop spinning
                    }
                }
            } else if (shooterMode == 2) { // Manual Power
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else if (shooterMode == 1) { // Preset Power for Intake Sequence
                shooterRightPower = 0.60;
                shooterLeftPower = 0.60;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else { // Off
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            // --- Intake Controls ---
            if (gamepad1.a && !lastAPress && !humanPlayerMode) {
                intakeRunning = !(intakeRunning && intakeWheelsPower == -1.0);
                intakeWheelsPower = -1.0;
            }
            lastAPress = gamepad1.a;

            if (gamepad1.b && !lastBPress && !humanPlayerMode) {
                intakeRunning = !(intakeRunning && intakeWheelsPower == 1.0);
                intakeWheelsPower = 1.0;
            }
            lastBPress = gamepad1.b;

            // --- Intake Sequence ---
            if (gamepad1.dpad_up && !intakeSequenceRunning) {
                intakeSequenceRunning = true;
                intakeSequenceStartTime = System.currentTimeMillis();
                intakeSequenceStep = 0;
                shooterMode = 1;
                ballCount = 0;
            }

            if (intakeSequenceRunning) {
                long elapsed = System.currentTimeMillis() - intakeSequenceStartTime;
                if (intakeSequenceStep < 4) {
                    long cycleTime = elapsed % 2500;
                    if (elapsed >= (intakeSequenceStep + 1) * 2500) {
                        intakeSequenceStep++;
                    }

                    if (cycleTime < 1000) intakeWheels.setPower(0);
                    else if (cycleTime < 1500) intakeWheels.setPower(-1.0);
                    else intakeWheels.setPower(0);
                } else {
                    intakeSequenceRunning = false;
                    intakeWheels.setPower(0);
                }
            } else if (intakeRunning) {
                intakeWheels.setPower(intakeWheelsPower);
            } else {
                intakeWheels.setPower(0);
            }

            // --- Ball Counting ---
            int totalColor = colorSensor.red() + colorSensor.green() + colorSensor.blue();
            if (totalColor > colorThreshold && !ballDetected) {
                ballDetected = true;
                ballCount++;
            } else if (totalColor <= colorThreshold && ballDetected) {
                ballDetected = false;
            }

            // --- Human Player Mode ---
            if (gamepad1.dpadDownWasPressed()) {
                humanPlayerMode = !humanPlayerMode;
                shooterRightPower = 0;
                shooterLeftPower = 0;
                autoAimEnabled = false;
                batteryCompensationEnabled = false;
                limelightTrackingEnabled = false;
                Limelightspin.setPosition(0.5);
            }

            if (gamepad1.a && humanPlayerMode) {
                shooterRightPower = -0.5;
                shooterLeftPower = -0.5;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
                intakeWheelsPower = 0.5;
                intakeWheels.setPower(intakeWheelsPower);
            }

            // --- Telemetry ---
            telemetry.addData("Shooter Vel", "%.2f", shooterRight.getVelocity());
            telemetry.addData("Shooter SetPower", "%.2f", shooterRightPower);
            telemetry.addData("Shooter ActualPower", "%.2f", shooterRight.getPower());
            telemetry.addData("Battery Voltage", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Human Player Mode", humanPlayerMode);
            telemetry.addData("Shooter Mode", shooterMode);
            telemetry.addData("Auto-Aim Enabled", autoAimEnabled);
            telemetry.addData("Ball Count", ballCount);
            telemetry.update();
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        double tagSize = 0.21; // 210mm in meters
        tagLibraryBuilder.addTag(20, "Target Tag 20", tagSize, DistanceUnit.METER);
        tagLibraryBuilder.addTag(24, "Target Tag 24", tagSize, DistanceUnit.METER);
        AprilTagLibrary customTagLibrary = tagLibraryBuilder.build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(customTagLibrary)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private double calculateShooterPower() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            targetDistance = 0.0;
            return 0.0;
        }

        AprilTagDetection detection = currentDetections.get(0);
        targetDistance = detection.ftcPose.range;
        double heightDifference = (APRILTAG_HEIGHT + TARGET_HEIGHT_ABOVE_TAG) - SHOOTER_HEIGHT;

        double[][] powerMap = {
                {0.5, 0.35}, {1.0, 0.45}, {1.5, 0.55}, {2.0, 0.65},
                {2.5, 0.75}, {3.0, 0.85}, {3.5, 0.95}, {4.0, 1.00}
        };

        double calculatedPower = 0.5; // Default

        if (targetDistance <= powerMap[0][0]) {
            calculatedPower = powerMap[0][1];
        } else if (targetDistance >= powerMap[powerMap.length - 1][0]) {
            calculatedPower = powerMap[powerMap.length - 1][1];
        } else {
            for (int i = 0; i < powerMap.length - 1; i++) {
                if (targetDistance >= powerMap[i][0] && targetDistance <= powerMap[i + 1][0]) {
                    double d1 = powerMap[i][0], p1 = powerMap[i][1];
                    double d2 = powerMap[i + 1][0], p2 = powerMap[i + 1][1];
                    calculatedPower = p1 + (targetDistance - d1) * (p2 - p1) / (d2 - d1);
                    break;
                }
            }
        }

        if (heightDifference > 0.1) calculatedPower += heightDifference * 0.1;
        else if (heightDifference < -0.1) calculatedPower -= Math.abs(heightDifference) * 0.05;

        return Math.max(0.30, Math.min(1.0, calculatedPower));
    }

    private void trackAprilTagWithLimelight() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            Limelightspin.setPosition(0.5);
            return;
        }

        AprilTagDetection detection = currentDetections.get(0);
        int imageWidth = 1280;
        int imageHeight = 720;

        double xError = (detection.center.x - imageWidth / 2.0) / (imageWidth / 2.0);
        double yError = (detection.center.y - imageHeight / 2.0) / (imageHeight / 2.0);

        smoothedXError = SMOOTHING_FACTOR * xError + (1 - SMOOTHING_FACTOR) * smoothedXError;
        smoothedYError = SMOOTHING_FACTOR * yError + (1 - SMOOTHING_FACTOR) * smoothedYError;

        double finalXError = Math.abs(smoothedXError) < DEADBAND ? 0 : smoothedXError;
        double finalYError = Math.abs(smoothedYError) < DEADBAND ? 0 : smoothedYError;

        if (Math.abs(finalXError) > 0.3) {
            double spinSpeed = finalXError * SPIN_GAIN;
            if (Math.abs(spinSpeed) > 0 && Math.abs(spinSpeed) < MIN_SPEED) {
                spinSpeed = Math.signum(spinSpeed) * MIN_SPEED;
            }
            spinSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, spinSpeed));
            spinPosition = 0.5 + spinSpeed;
            Limelightspin.setPosition(spinPosition);
        } else {
            spinPosition = 0.5;
            Limelightspin.setPosition(spinPosition);
            smoothedXError = 0.0;
        }

        // Corrected: Tilt control logic should be outside the spin 'else' block
        if (Math.abs(finalYError) > 0.3) {
            double tiltAdjustment = -finalYError * TILT_GAIN;
            tiltPosition += tiltAdjustment;
            tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));
            Limelighttilt.setPosition(tiltPosition);
        }
    }

    private double applyBatteryCompensation(double basePower) {
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (currentVoltage < MIN_VOLTAGE) {
            telemetry.addData("WARNING", "Battery voltage critically low!");
            return basePower;
        }

        double voltageRatio = currentVoltage / NOMINAL_VOLTAGE;
        double compensatedPower = basePower / voltageRatio;

        if (voltageRatio < 0.95) {
            double efficiencyLoss = (1.0 - voltageRatio) * 0.15; // 15% extra per volt lost
            compensatedPower += efficiencyLoss;
        }

        return Math.min(1.0, compensatedPower);
    }
}
