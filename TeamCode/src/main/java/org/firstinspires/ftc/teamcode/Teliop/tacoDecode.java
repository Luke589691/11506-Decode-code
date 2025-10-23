package org.firstinspires.ftc.teamcode.Teliop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

    // ========================================
    // EASY TUNING AREA - ADJUST THESE VALUES
    // ========================================

    // DISTANCE-TO-POWER MAPPING (Distance in meters, Power 0.0-1.0)
    private static final double[][] POWER_MAP = {
            // {Distance (m), Power}
            {0.75,  0.52},
            {1.0,   0.55},
            {1.25,  0.55},
            {1.50,  0.57},
            {1.75,  0.60},
            {2.0,   1.00},
            {2.25,  1.00},
            {2.50,  1.00},
            {2.75,  1.00},
            {3.0,   1.00},
            {3.25,  1.00},
            {3.50,  1.00},
            {3.75,  1.00},
            {4.0,   1.00},
            {4.25,  1.00},
            {4.50,  1.00},
            {4.75,  1.00},
            {5.0,   1.00},
            {5.25,  1.00},
            {5.50,  1.00},
            {5.75,  1.00},
            {6.0,   1.00}
    };

    // BATTERY COMPENSATION - Simple power boost when battery is low
    private static final double LOW_BATTERY_THRESHOLD = 12.0;  // Volts - when to start boosting
    private static final double BATTERY_POWER_BOOST = 10;    // Extra power added when below threshold
    private static final boolean ENABLE_BATTERY_BOOST = true;  // Set to false to disable

    // HEIGHT COMPENSATION
    private static final double HEIGHT_COMPENSATION_UP = 0.1;    // Power added per meter when shooting up
    private static final double HEIGHT_COMPENSATION_DOWN = 0.05; // Power reduced per meter when shooting down
    private static final double HEIGHT_COMPENSATION_THRESHOLD = 0.1; // Minimum height difference to compensate (meters)

    // MANUAL ADJUSTMENT INCREMENTS
    private static final double POWER_ADJUST_LARGE = 0.05;  // D-pad Up/Down adjustment
    private static final double POWER_ADJUST_SMALL = 0.01;  // D-pad Left/Right adjustment

    // TURRET TRACKING TUNING - Optimized for smooth, responsive tracking
    private static final double SMOOTHING_FACTOR = 0.4;      // Higher = faster response (0.0-1.0)
    private static final double DEADBAND = 0.01;             // Ignore very small movements
    private static final double SPIN_GAIN = 0.25;            // Horizontal tracking speed
    private static final double TILT_GAIN = 0.015;           // Vertical tracking speed
    private static final double MIN_SPEED = 0.03;            // Minimum servo speed
    private static final double MAX_SPEED = 0.6;             // Maximum servo speed
    private static final double ERROR_THRESHOLD = 0.03;      // Start tracking when 3% off center

    // Continuous servo adjustment - higher values = more aggressive tracking
    private static final double SPIN_CONTINUOUS_MULTIPLIER = 1.5;  // Boost for continuous rotation

    // SERVO LIMITS
    private static final double TILT_MIN = 0.0;
    private static final double TILT_MAX = 0.2;
    private static final double SPIN_CENTER = 0.5;

    // PHYSICAL MEASUREMENTS (in meters)
    private static final double APRILTAG_HEIGHT = 0.33;          // Height of AprilTag center from ground
    private static final double SHOOTER_HEIGHT = 0.40;           // Height of shooter muzzle from ground
    private static final double TARGET_HEIGHT_ABOVE_TAG = 0.30;  // Aiming point above tag center

    // DISTANCE CALIBRATION
    private static final double DISTANCE_OFFSET = -0.45;  // Adjust this value (meters)
    private static final boolean USE_DISTANCE_OFFSET = true;  // Set to false to disable

    // APRILTAG SIZE - Verify this matches your actual tag size!
    private static final double APRILTAG_SIZE = 0.166;  // Default 166mm (adjust if different)

    // COLOR SENSOR
    private static final int COLOR_THRESHOLD = 1000;  // Adjust based on your sensor

    // INTAKE SEQUENCE TIMING (milliseconds)
    private static final long INTAKE_CYCLE_TIME = 2500;
    private static final long INTAKE_WAIT_TIME = 1000;
    private static final long INTAKE_RUN_TIME = 500;
    private static final int INTAKE_CYCLES = 4;

    // DEFAULT POWERS
    private static final double DEFAULT_SHOOTER_POWER = 0.6;
    private static final double INTAKE_SEQUENCE_POWER = 0.60;
    private static final double HUMAN_PLAYER_POWER = -0.5;
    private static final double HUMAN_PLAYER_INTAKE = 0.5;

    // ========================================
    // END OF TUNING AREA
    // ========================================

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
    private boolean limelightTrackingEnabled = false;
    private boolean intakeSequenceRunning = false;
    private boolean ballDetected = false;

    private int shooterMode = 0; // 0: Off, 1: Auto, 2: Manual, 3: Auto-Aim
    private int intakeSequenceStep = 0;
    private int ballCount = 0;
    private long intakeSequenceStartTime = 0;

    private double shooterLeftPower = DEFAULT_SHOOTER_POWER;
    private double shooterRightPower = DEFAULT_SHOOTER_POWER;
    private double intakeWheelsPower = 0.0;
    private double targetDistance = 0.0;
    private double rawDistance = 0.0;
    private double spinPosition = SPIN_CENTER;
    private double tiltPosition = 0.1;
    private double currentVoltage = 12.5;

    // Smoothing for Turret Tracking
    private double smoothedXError = 0.0;
    private double smoothedYError = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        Limelightspin = hardwareMap.get(Servo.class, "Limelightspin");
        Limelighttilt = hardwareMap.get(Servo.class, "Limelighttilt");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Limelightspin.setPosition(SPIN_CENTER);
        Limelighttilt.setPosition(tiltPosition);

        // Initialize vision
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {

            // Update battery voltage every loop
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            // --- Gamepad Controls ---
            if (gamepad1.dpad_up && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += POWER_ADJUST_LARGE;
                shooterRightPower += POWER_ADJUST_LARGE;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }

            if (gamepad1.dpad_down && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= POWER_ADJUST_LARGE;
                shooterRightPower -= POWER_ADJUST_LARGE;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
            }

            if (gamepad1.dpad_right && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += POWER_ADJUST_SMALL;
                shooterRightPower += POWER_ADJUST_SMALL;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }

            if (gamepad1.dpad_left && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= POWER_ADJUST_SMALL;
                shooterRightPower -= POWER_ADJUST_SMALL;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
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
                    Limelightspin.setPosition(SPIN_CENTER);
                } else {
                    autoAimEnabled = true;
                    limelightTrackingEnabled = true;
                    shooterMode = 3;
                }
            }
            lastYPress = gamepad1.y;

            // --- Shooter Logic ---
            if (autoAimEnabled) {
                double calculatedPower = calculateShooterPower();

                if (calculatedPower > 0) {
                    shooterLeftPower = calculatedPower;
                    shooterRightPower = calculatedPower;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                    if (limelightTrackingEnabled) {
                        trackAprilTagWithTurret();
                    }
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    if (limelightTrackingEnabled) {
                        Limelightspin.setPosition(SPIN_CENTER);
                    }
                }
            } else if (shooterMode == 2) {
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else if (shooterMode == 1) {
                shooterRightPower = INTAKE_SEQUENCE_POWER;
                shooterLeftPower = INTAKE_SEQUENCE_POWER;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            // --- Intake Controls ---
            if (gamepad1.a && !lastAPress && !humanPlayerMode) {
                intakeRunning = !intakeRunning;
                intakeWheelsPower = -1.0;
            }
            lastAPress = gamepad1.a;

            if (gamepad1.b && !lastBPress && !humanPlayerMode) {
                intakeRunning = !intakeRunning;
                intakeWheelsPower = 1.0;
            }
            lastBPress = gamepad1.b;

            // --- Intake Sequence ---
            if (gamepad1.right_bumper && !intakeSequenceRunning) {
                intakeSequenceRunning = true;
                intakeSequenceStartTime = System.currentTimeMillis();
                intakeSequenceStep = 0;
                shooterMode = 1;
                ballCount = 0;
            }

            if (intakeSequenceRunning) {
                long elapsed = System.currentTimeMillis() - intakeSequenceStartTime;
                if (intakeSequenceStep < INTAKE_CYCLES) {
                    long cycleTime = elapsed % INTAKE_CYCLE_TIME;
                    if (elapsed >= (intakeSequenceStep + 1) * INTAKE_CYCLE_TIME) {
                        intakeSequenceStep++;
                    }

                    if (cycleTime < INTAKE_WAIT_TIME) {
                        intakeWheels.setPower(0);
                    } else if (cycleTime < INTAKE_WAIT_TIME + INTAKE_RUN_TIME) {
                        intakeWheels.setPower(-1.0);
                    } else {
                        intakeWheels.setPower(0);
                    }
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
            if (totalColor > COLOR_THRESHOLD && !ballDetected) {
                ballDetected = true;
                ballCount++;
            } else if (totalColor <= COLOR_THRESHOLD && ballDetected) {
                ballDetected = false;
            }

            // --- Human Player Mode ---
            if (gamepad1.back) {
                humanPlayerMode = !humanPlayerMode;
                shooterRightPower = 0;
                shooterLeftPower = 0;
                autoAimEnabled = false;
                limelightTrackingEnabled = false;
                Limelightspin.setPosition(SPIN_CENTER);
            }

            if (gamepad1.a && humanPlayerMode) {
                shooterRightPower = HUMAN_PLAYER_POWER;
                shooterLeftPower = HUMAN_PLAYER_POWER;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
                intakeWheelsPower = HUMAN_PLAYER_INTAKE;
                intakeWheels.setPower(intakeWheelsPower);
            }

            // --- Telemetry ---
            boolean batteryBoostActive = ENABLE_BATTERY_BOOST && currentVoltage < LOW_BATTERY_THRESHOLD;
            telemetry.addData("=== DISTANCE ===", "");
            telemetry.addData("Raw", "%.3f m", rawDistance);
            telemetry.addData("Corrected", "%.3f m", targetDistance);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Power", "%.2f", shooterRightPower);
            telemetry.addData("Battery", "%.2f V %s", currentVoltage, batteryBoostActive ? "(BOOST ON)" : "");
            telemetry.addData("Mode", humanPlayerMode ? "Human Player" : getShooterModeName());
            telemetry.addData("Balls", ballCount);
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Spin", "%.3f", spinPosition);
            telemetry.addData("Tilt", "%.3f", tiltPosition);
            telemetry.addData("X Err", "%.3f", smoothedXError);
            telemetry.addData("Y Err", "%.3f", smoothedYError);
            telemetry.update();
        }

        visionPortal.close();
    }

    private String getShooterModeName() {
        switch (shooterMode) {
            case 0: return "Off";
            case 1: return "Intake Seq";
            case 2: return "Manual";
            case 3: return "Auto-Aim";
            default: return "Unknown";
        }
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        double tagSize = APRILTAG_SIZE;
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
            rawDistance = 0.0;
            return 0.0;
        }

        AprilTagDetection detection = currentDetections.get(0);
        rawDistance = detection.ftcPose.range;

        // Apply calibration offset if enabled
        if (USE_DISTANCE_OFFSET) {
            targetDistance = rawDistance + DISTANCE_OFFSET;
        } else {
            targetDistance = rawDistance;
        }

        // Ensure distance is positive
        if (targetDistance < 0.1) {
            targetDistance = 0.1;
        }

        double heightDifference = (APRILTAG_HEIGHT + TARGET_HEIGHT_ABOVE_TAG) - SHOOTER_HEIGHT;

        // Interpolate power from power map
        double calculatedPower = 0.5;

        if (targetDistance <= POWER_MAP[0][0]) {
            calculatedPower = POWER_MAP[0][1];
        } else if (targetDistance >= POWER_MAP[POWER_MAP.length - 1][0]) {
            calculatedPower = POWER_MAP[POWER_MAP.length - 1][1];
        } else {
            for (int i = 0; i < POWER_MAP.length - 1; i++) {
                if (targetDistance >= POWER_MAP[i][0] && targetDistance <= POWER_MAP[i + 1][0]) {
                    double d1 = POWER_MAP[i][0], p1 = POWER_MAP[i][1];
                    double d2 = POWER_MAP[i + 1][0], p2 = POWER_MAP[i + 1][1];
                    calculatedPower = p1 + (targetDistance - d1) * (p2 - p1) / (d2 - d1);
                    break;
                }
            }
        }

        // Apply height compensation
        if (heightDifference > HEIGHT_COMPENSATION_THRESHOLD) {
            calculatedPower += heightDifference * HEIGHT_COMPENSATION_UP;
        } else if (heightDifference < -HEIGHT_COMPENSATION_THRESHOLD) {
            calculatedPower -= Math.abs(heightDifference) * HEIGHT_COMPENSATION_DOWN;
        }

        // Apply battery boost if enabled and voltage is low
        if (ENABLE_BATTERY_BOOST && currentVoltage < LOW_BATTERY_THRESHOLD) {
            calculatedPower += BATTERY_POWER_BOOST;
        }

        return Math.max(0.30, Math.min(1.0, calculatedPower));
    }

    private void trackAprilTagWithTurret() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            Limelightspin.setPosition(SPIN_CENTER);
            smoothedXError = 0.0;
            smoothedYError = 0.0;
            return;
        }

        AprilTagDetection detection = currentDetections.get(0);
        int imageWidth = 1280;
        int imageHeight = 720;

        // Calculate normalized error (-1.0 to 1.0)
        double xError = (detection.center.x - imageWidth / 2.0) / (imageWidth / 2.0);
        double yError = (detection.center.y - imageHeight / 2.0) / (imageHeight / 2.0);

        // Apply exponential smoothing for responsive but stable tracking
        smoothedXError = SMOOTHING_FACTOR * xError + (1 - SMOOTHING_FACTOR) * smoothedXError;
        smoothedYError = SMOOTHING_FACTOR * yError + (1 - SMOOTHING_FACTOR) * smoothedYError;

        // Apply deadband to prevent jitter
        double finalXError = Math.abs(smoothedXError) < DEADBAND ? 0 : smoothedXError;
        double finalYError = Math.abs(smoothedYError) < DEADBAND ? 0 : smoothedYError;

        // HORIZONTAL TRACKING (Spin servo - continuous rotation)
        if (Math.abs(finalXError) > ERROR_THRESHOLD) {
            // Proportional control with enhanced gain for continuous servos
            double spinSpeed = -finalXError * SPIN_GAIN * SPIN_CONTINUOUS_MULTIPLIER;

            // Apply minimum speed threshold to overcome friction
            if (Math.abs(spinSpeed) > 0 && Math.abs(spinSpeed) < MIN_SPEED) {
                spinSpeed = Math.signum(spinSpeed) * MIN_SPEED;
            }

            // Clamp to maximum speed
            spinSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, spinSpeed));

            // For continuous rotation servo: 0.5 = stop, <0.5 = CCW, >0.5 = CW
            spinPosition = SPIN_CENTER + spinSpeed;
            Limelightspin.setPosition(spinPosition);
        } else {
            // Target is centered - stop rotation
            spinPosition = SPIN_CENTER;
            Limelightspin.setPosition(spinPosition);
            smoothedXError = 0.0;
        }

        // VERTICAL TRACKING (Tilt servo - positional)
        // Always adjust tilt when target is visible, more aggressive than horizontal
        double tiltAdjustment = -finalYError * TILT_GAIN;
        tiltPosition += tiltAdjustment;
        tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));
        Limelighttilt.setPosition(tiltPosition);
    }
}