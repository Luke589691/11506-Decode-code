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

    // DRIVE TUNING
    private static final double DRIVE_SPEED_MULTIPLIER = 1.0;    // Normal drive speed (0.0-1.0)
    private static final double DRIVE_SLOW_MULTIPLIER = 0.35;    // Slow mode speed (not used with new controls)
    private static final double TURN_SPEED_MULTIPLIER = 0.8;     // Turning speed multiplier
    private static final double DEADZONE = 0.05;                 // Joystick deadzone to prevent drift

    // DISTANCE-TO-POWER MAPPING (Distance in meters, Power 0.0-1.0)
    private static final double[][] POWER_MAP = {
            // {Distance (m), Power}
            {0.75, 0.52},
            {1.0, 0.55},
            {1.25, 0.55},
            {1.50, 0.57},
            {1.75, 0.60},
            {2.0, 1.00},
            {2.25, 1.00},
            {2.50, 1.00},
            {2.75, 1.00},
            {3.0, 1.00},
            {3.25, 1.00},
            {3.50, 1.00},
            {3.75, 1.00},
            {4.0, 1.00},
            {4.25, 1.00},
            {4.50, 1.00},
            {4.75, 1.00},
            {5.0, 1.00},
            {5.25, 1.00},
            {5.50, 1.00},
            {5.75, 1.00},
            {6.0, 1.00}
    };

    // BATTERY COMPENSATION - Simple power boost when battery is low
    private static final double LOW_BATTERY_THRESHOLD = 12.0;  // Volts - when to start boosting
    private static final double BATTERY_POWER_BOOST = 0.1;     // Extra power added when below threshold
    private static final boolean ENABLE_BATTERY_BOOST = true;  // Set to false to disable

    // HEIGHT COMPENSATION
    private static final double HEIGHT_COMPENSATION_UP = 0.1;    // Power added per meter when shooting up
    private static final double HEIGHT_COMPENSATION_DOWN = 0.05; // Power reduced per meter when shooting down
    private static final double HEIGHT_COMPENSATION_THRESHOLD = 0.1; // Minimum height difference to compensate (meters)

    // MANUAL ADJUSTMENT INCREMENTS
    private static final double POWER_ADJUST_LARGE = 0.05;  // Bumper adjustment
    private static final double POWER_ADJUST_SMALL = 0.01;  // D-pad Left/Right adjustment

    // TURRET TRACKING - SIMPLE STEP MODE
    private static final double PIXEL_DEADZONE = 50.0;       // 5cm in pixels (adjust based on distance)
    private static final double SERVO_STEP = 0.01;           // Move servo by 0.01 each time

    // TURRET HEADING CONTROL - Keep turret facing forward (60 degrees)
    private static final double TARGET_HEADING = 60.0;       // Forward direction in degrees
    private static final double HEADING_SMOOTHING = 0.85;    // Keep 85% OLD, 15% new
    private static final double HEADING_GAIN = 0.002;        // How aggressively to correct heading
    private static final double HEADING_DEADZONE = 2.0;      // Degrees - don't correct within this range



    // SERVO LIMITS - ADJUST THESE FOR YOUR PHYSICAL SERVO RANGE
    private static final double TILT_MIN = 0.0;
    private static final double TILT_MAX = 0.2;
    private static final double SPIN_CENTER = 0.5;
    private static final double SPIN_MIN = 0.1;    // Left limit (wraps to SPIN_MAX)
    private static final double SPIN_MAX = 1.0;    // Right limit (wraps to SPIN_MIN)
    private static final boolean SPIN_WRAPAROUND_ENABLED = true;  // Enable continuous rotation wraparound

    // PIVOT SERVO SETTINGS
    private static final double PIVOT_MIN = 0.0;      // Minimum pivot angle (flat/down)
    private static final double PIVOT_MAX = 1.0;      // Maximum pivot angle (up)
    private static final double PIVOT_DEFAULT = 0.3;  // Default position when not auto-aiming

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

    // RAPID SHOOT TIMING (milliseconds)
    private static final long RAPID_SHOOT_CYCLE_TIME = 500;  // Time between intake bursts
    private static final long RAPID_SHOOT_BURST_TIME = 200;  // How long each intake burst runs

    // DEFAULT POWERS
    private static final double DEFAULT_SHOOTER_POWER = 0.6;
    private static final double HUMAN_PLAYER_SHOOTER_POWER = -1.0;  // Full reverse
    private static final double HUMAN_PLAYER_INTAKE_POWER = -0.5;   // Half speed reverse
    private static final double SMOOTHING_FACTOR = 0.85;
    private static final double SPIN_GAIN = 0.01;
    private static final double TILT_GAIN = 0.01;
    private static final double MIN_SERVO_MOVEMENT = 0.01;


    // ========================================
    // END OF TUNING AREA
    // ========================================

    // Vision and AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Hardware
    private DcMotorEx shooterLeft, shooterRight, intakeWheels;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Servo Limelightspin, Limelighttilt;
    private ColorSensor colorSensor;

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
    private boolean autoAimEnabled = false;
    private boolean limelightTrackingEnabled = false;
    private boolean rapidShootMode = false;
    private boolean ballDetected = false;

    private int shooterMode = 0; // 0: Off, 1: Manual, 2: Auto-Aim, 3: Rapid Shoot
    private int shooterModeBeforeRapid = 0; // Store mode before rapid shoot
    private int ballCount = 0;
    private long rapidShootStartTime = 0;

    private double shooterLeftPower = DEFAULT_SHOOTER_POWER;
    private double shooterRightPower = DEFAULT_SHOOTER_POWER;
    private double intakeWheelsPower = 0.0;
    private double targetDistance = 0.0;
    private double rawDistance = 0.0;
    private double spinPosition = SPIN_CENTER;
    private double tiltPosition = 0.2;
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

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        Limelightspin = hardwareMap.get(Servo.class, "Limelightspin");
        Limelighttilt = hardwareMap.get(Servo.class, "Limelighttilt");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // REVERSED SHOOTER DIRECTIONS
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        // Set drive motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set drive motors to run without encoders for direct control
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior to brake for better control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelightspin.setPosition(SPIN_CENTER);
        Limelighttilt.setPosition(tiltPosition);

        // Initialize vision
        initAprilTag();

        telemetry.addData("Status", "Initialized - AprilTag Simple Tracking");
        telemetry.addData("Tracking Mode", "Simple Step (0.01 per move)");
        telemetry.addData("Controls", "L-Stick: Drive | R-Stick: Turn");
        telemetry.addData("X", "Manual Shooter | Y: Auto-Aim");
        telemetry.addData("Bumpers", "Speed ±0.05 | DPad L/R: ±0.01");
        telemetry.addData("DPad Up", "Rapid Shoot | DPad Down: Human Player");
        telemetry.addData("A/B", "Intake Control");
        telemetry.addData("Turret", "Steps 0.01 toward AprilTag");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update battery voltage every loop
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            // ========================================
            // DRIVE CONTROLS (Robot-Centric Mecanum)
            // ========================================
            double drive = -applyDeadzone(gamepad1.left_stick_y);    // Forward/backward
            double strafe = applyDeadzone(gamepad1.left_stick_x);    // Left/right
            double turn = applyDeadzone(gamepad1.right_stick_x);     // Rotation

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
            if (gamepad1.left_bumper && !lastLeftBumperPress && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= POWER_ADJUST_LARGE;
                shooterRightPower -= POWER_ADJUST_LARGE;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
            }
            lastLeftBumperPress = gamepad1.left_bumper;

            // RIGHT BUMPER: Increase shooter power by 0.05
            if (gamepad1.right_bumper && !lastRightBumperPress && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += POWER_ADJUST_LARGE;
                shooterRightPower += POWER_ADJUST_LARGE;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }
            lastRightBumperPress = gamepad1.right_bumper;

            // DPAD RIGHT: Increase shooter power by 0.01
            if (gamepad1.dpad_right && !lastDpadRightPress && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower += POWER_ADJUST_SMALL;
                shooterRightPower += POWER_ADJUST_SMALL;
                shooterLeftPower = Math.min(1.0, shooterLeftPower);
                shooterRightPower = Math.min(1.0, shooterRightPower);
            }
            lastDpadRightPress = gamepad1.dpad_right;

            // DPAD LEFT: Decrease shooter power by 0.01
            if (gamepad1.dpad_left && !lastDpadLeftPress && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= POWER_ADJUST_SMALL;
                shooterRightPower -= POWER_ADJUST_SMALL;
                shooterLeftPower = Math.max(0.0, shooterLeftPower);
                shooterRightPower = Math.max(0.0, shooterRightPower);
            }
            lastDpadLeftPress = gamepad1.dpad_left;

            // X Button for manual shooter mode
            if (gamepad1.x && !lastXPress && !humanPlayerMode && !rapidShootMode) {
                shooterMode = (shooterMode == 1) ? 0 : 1;
                autoAimEnabled = false;
                limelightTrackingEnabled = false;
            }
            lastXPress = gamepad1.x;

            // Y Button for auto-aim mode
            if (gamepad1.y && !lastYPress && !humanPlayerMode && !rapidShootMode) {
                if (autoAimEnabled) {
                    autoAimEnabled = false;
                    limelightTrackingEnabled = false;
                    shooterMode = 0;
                    Limelightspin.setPosition(SPIN_CENTER);
                } else {
                    autoAimEnabled = true;
                    limelightTrackingEnabled = true;
                    shooterMode = 2;
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
                    autoAimEnabled = false;
                    limelightTrackingEnabled = false;
                    rapidShootMode = false;
                    Limelightspin.setPosition(SPIN_CENTER);
                } else {
                    shooterRightPower = DEFAULT_SHOOTER_POWER;
                    shooterLeftPower = DEFAULT_SHOOTER_POWER;
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    intakeWheels.setPower(0);
                }
            }
            lastDpadDownPress = gamepad1.dpad_down;

            // ========================================
            // SIMPLE APRILTAG TRACKING
            // ========================================
            if (!humanPlayerMode) {
                simpleTrackAprilTag();
            }

            // --- Shooter Logic ---
            if (humanPlayerMode) {
                // Human player mode - shooter and intake already set when mode activated
                shooterRight.setPower(HUMAN_PLAYER_SHOOTER_POWER);
                shooterLeft.setPower(HUMAN_PLAYER_SHOOTER_POWER);
                intakeWheels.setPower(HUMAN_PLAYER_INTAKE_POWER);
            } else if (rapidShootMode) {
                // Rapid shoot mode - maintain shooter mode, pulse intake
                if (autoAimEnabled) {
                    double calculatedPower = calculateShooterPower();
                    if (calculatedPower > 0) {
                        shooterLeftPower = calculatedPower;
                        shooterRightPower = calculatedPower;
                        shooterRight.setPower(shooterRightPower);
                        shooterLeft.setPower(shooterLeftPower);
                    } else {
                        shooterRight.setPower(0);
                        shooterLeft.setPower(0);
                    }
                } else if (shooterMode == 1) {
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
            } else if (autoAimEnabled) {
                double calculatedPower = calculateShooterPower();

                if (calculatedPower > 0) {
                    shooterLeftPower = calculatedPower;
                    shooterRightPower = calculatedPower;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
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

            // --- Ball Counting ---
            int totalColor = colorSensor.red() + colorSensor.green() + colorSensor.blue();
            if (totalColor > COLOR_THRESHOLD && !ballDetected) {
                ballDetected = true;
                ballCount++;
            } else if (totalColor <= COLOR_THRESHOLD && ballDetected) {
                ballDetected = false;
            }

            // ========================================
            // TELEMETRY
            // ========================================
            boolean batteryBoostActive = ENABLE_BATTERY_BOOST && currentVoltage < LOW_BATTERY_THRESHOLD;

            // Show AprilTag detection info
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int detectedTagCount = (currentDetections != null) ? currentDetections.size() : 0;

            telemetry.addData("=== APRILTAG VISION ===", "");
            telemetry.addData("Tags Detected", detectedTagCount);
            if (detectedTagCount > 0 && currentDetections != null) {
                AprilTagDetection detection = currentDetections.get(0);
                telemetry.addData("Active Tag ID", detection.id);
                telemetry.addData("Tag Center", "X:%.1f Y:%.1f", detection.center.x, detection.center.y);
                telemetry.addData("Tag Position", "X:%.1f Y:%.1f Z:%.1f cm",
                        detection.ftcPose.x * 100,
                        detection.ftcPose.y * 100,
                        detection.ftcPose.z * 100);
                telemetry.addData("Tag Angles", "P:%.1f R:%.1f Y:%.1f deg",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw);
            }

            telemetry.addData("=== APRILTAG TRACKING ===", "");
            telemetry.addData("Tags Detected", detectedTagCount);
            if (detectedTagCount > 0 && currentDetections != null) {
                AprilTagDetection detection = currentDetections.get(0);
                telemetry.addData("Active Tag ID", detection.id);
                telemetry.addData("Tag Center", "X:%.1f Y:%.1f", detection.center.x, detection.center.y);
            }
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("FL/FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL/BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.addData("=== DISTANCE ===", "");
            telemetry.addData("Raw", "%.3f m", rawDistance);
            telemetry.addData("Corrected", "%.3f m", targetDistance);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Power", "%.2f", shooterRightPower);
            telemetry.addData("Battery", "%.2f V %s", currentVoltage, batteryBoostActive ? "(BOOST ON)" : "");
            telemetry.addData("Mode", getShooterModeName());
            telemetry.addData("=== INTAKE ===", "");
            telemetry.addData("Power", "%.2f", intakeWheelsPower);
            telemetry.addData("Running", intakeRunning);
            telemetry.addData("Balls", ballCount);
            telemetry.addData("=== TURRET (SIMPLE TRACKING) ===", "");
            telemetry.addData("Spin", "%.3f", spinPosition);
            telemetry.addData("Tilt", "%.3f", tiltPosition);
            telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
            telemetry.update();
        }

        visionPortal.close();
    }

    private String getShooterModeName() {
        if (humanPlayerMode) {
            return "Human Player";
        } else if (rapidShootMode) {
            return "Rapid Shoot (" + (shooterModeBeforeRapid == 2 ? "Auto" : shooterModeBeforeRapid == 1 ? "Manual" : "Off") + ")";
        }

        switch (shooterMode) {
            case 0:
                return "Off";
            case 1:
                return "Manual";
            case 2:
                return "Auto-Aim";
            default:
                return "Unknown";
        }
    }

    private void initAprilTag() {
        // Create custom tag library with current season tags
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        double tagSize = APRILTAG_SIZE;

        // Add all common FTC season tags (adjust IDs as needed for your field setup)
        tagLibraryBuilder.addTag(20, "Blue Goal Tag", tagSize, DistanceUnit.METER);
        tagLibraryBuilder.addTag(21, "Center Obelisk Tag", tagSize, DistanceUnit.METER);
        tagLibraryBuilder.addTag(24, "Red Goal Tag", tagSize, DistanceUnit.METER);

        AprilTagLibrary customTagLibrary = tagLibraryBuilder.build();

        // Configure AprilTag processor with visual feedback options
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(customTagLibrary)
                .setDrawTagID(true)              // Draw tag ID on camera stream
                .setDrawTagOutline(true)         // Draw outline around detected tags
                .setDrawAxes(true)               // Draw X, Y, Z axes
                .setDrawCubeProjection(true)     // Draw 3D cube projection
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // Build vision portal with recommended resolution (640x480)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
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

    private void simpleTrackAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            // No target detected - hold current position
            telemetry.addData("=== TRACKING ===", "");
            telemetry.addData("Status", "❌ No AprilTag detected");
            telemetry.addData("Spin", "%.3f (holding)", spinPosition);
            telemetry.addData("Tilt", "%.3f (holding)", tiltPosition);
            return;
        }

        AprilTagDetection detection = currentDetections.get(0);

        int imageWidth = 640;
        int imageHeight = 480;

        // Calculate pixel error from center
        double centerX = imageWidth / 2.0;
        double centerY = imageHeight / 2.0;
        double pixelErrorX = detection.center.x - centerX;
        double pixelErrorY = detection.center.y - centerY;

        telemetry.addData("=== TRACKING ===", "");
        telemetry.addData("Tag Center", "X:%.0f Y:%.0f", detection.center.x, detection.center.y);
        telemetry.addData("Screen Center", "X:%.0f Y:%.0f", centerX, centerY);
        telemetry.addData("Pixel Error", "X:%.0f Y:%.0f", pixelErrorX, pixelErrorY);

        // HORIZONTAL TRACKING (Spin servo)
        if (Math.abs(pixelErrorX) > PIXEL_DEADZONE) {
            // Tag is more than 50 pixels from center horizontally
            double oldSpin = spinPosition;

            if (pixelErrorX > 0) {
                // Tag is to the right, move servo right
                spinPosition += SERVO_STEP;
            } else {
                // Tag is to the left, move servo left
                spinPosition -= SERVO_STEP;
            }

            // Handle wraparound for continuous rotation servo
            if (SPIN_WRAPAROUND_ENABLED) {
                if (spinPosition < SPIN_MIN) {
                    spinPosition = SPIN_MAX - (SPIN_MIN - spinPosition);
                    telemetry.addData("Spin", "%.3f → %.3f (wrapped left)", oldSpin, spinPosition);
                } else if (spinPosition > SPIN_MAX) {
                    spinPosition = SPIN_MIN + (spinPosition - SPIN_MAX);
                    telemetry.addData("Spin", "%.3f → %.3f (wrapped right)", oldSpin, spinPosition);
                } else {
                    telemetry.addData("Spin", "%.3f → %.3f", oldSpin, spinPosition);
                }
            } else {
                spinPosition = Math.max(SPIN_MIN, Math.min(SPIN_MAX, spinPosition));
                telemetry.addData("Spin", "%.3f → %.3f", oldSpin, spinPosition);
            }

            Limelightspin.setPosition(spinPosition);
            telemetry.addData("Status", "🎯 Tracking X (%.0fpx off)", pixelErrorX);
        } else {
            telemetry.addData("Spin", "%.3f (centered X)", spinPosition);
        }

        // VERTICAL TRACKING (Tilt servo)
        if (Math.abs(pixelErrorY) > PIXEL_DEADZONE) {
            // Tag is more than 50 pixels from center vertically
            double oldTilt = tiltPosition;

            if (pixelErrorY > 0) {
                // Tag is below center, tilt down
                tiltPosition += SERVO_STEP;
            } else {
                // Tag is above center, tilt up
                tiltPosition -= SERVO_STEP;
            }

            tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));

            telemetry.addData("Tilt", "%.3f → %.3f", oldTilt, tiltPosition);
            Limelighttilt.setPosition(tiltPosition);
            telemetry.addData("Status", "🎯 Tracking Y (%.0fpx off)", pixelErrorY);
        } else {
            telemetry.addData("Tilt", "%.3f (centered Y)", tiltPosition);
        }

        // Check if fully centered
        if (Math.abs(pixelErrorX) <= PIXEL_DEADZONE && Math.abs(pixelErrorY) <= PIXEL_DEADZONE) {
            telemetry.addData("Status", "🔒 LOCKED ON TARGET");
        }
    }

    private void trackAprilTagWithTurret() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            // No target detected - hold current position
            smoothedXError = 0.0;
            smoothedYError = 0.0;
            telemetry.addData("TRACKING", "❌ No target - holding position");
            telemetry.addData("Spin", "%.3f (holding)", spinPosition);
            telemetry.addData("Tilt", "%.3f (holding)", tiltPosition);
            return;
        }

        AprilTagDetection detection = currentDetections.get(0);

        int imageWidth = 640;
        int imageHeight = 480;

        // Calculate raw pixel error from center
        double centerX = imageWidth / 2.0;
        double centerY = imageHeight / 2.0;
        double pixelErrorX = detection.center.x - centerX;
        double pixelErrorY = detection.center.y - centerY;

        // Calculate total pixel distance from center
        double pixelDistance = Math.sqrt(pixelErrorX * pixelErrorX + pixelErrorY * pixelErrorY);

        telemetry.addData("=== TRACKING STATUS ===", "");
        telemetry.addData("Tag Center", "%.0f, %.0f", detection.center.x, detection.center.y);
        telemetry.addData("Pixel Offset", "X:%.0f Y:%.0f", pixelErrorX, pixelErrorY);
        telemetry.addData("Distance from Center", "%.0f px", pixelDistance);

        // ONLY TRACK IF MORE THAN DEADZONE PIXELS FROM CENTER
        if (pixelDistance <= PIXEL_DEADZONE) {
            // Target is close enough - HOLD POSITION and reset smoothing
            telemetry.addData("Status", "🔒 LOCKED (within %.0fpx)", PIXEL_DEADZONE);
            smoothedXError = 0.0;
            smoothedYError = 0.0;
            // DON'T update servo positions - keep them where they are
            telemetry.addData("Spin", "%.3f (locked)", spinPosition);
            telemetry.addData("Tilt", "%.3f (locked)", tiltPosition);
            return;
        }

        // Target is far enough - start tracking
        telemetry.addData("Status", "🎯 TRACKING (%.0fpx away)", pixelDistance);

        // Calculate normalized error (-1.0 to 1.0)
        double xError = pixelErrorX / centerX;
        double yError = pixelErrorY / centerY;

        // Apply exponential smoothing - REVERSED FACTOR (0.15 means keep 15% new, 85% old)
        // This makes movement smoother and less reactive
        smoothedXError = (1 - SMOOTHING_FACTOR) * xError + SMOOTHING_FACTOR * smoothedXError;
        smoothedYError = (1 - SMOOTHING_FACTOR) * yError + SMOOTHING_FACTOR * smoothedYError;

        telemetry.addData("Raw Error", "X:%.3f Y:%.3f", xError, yError);
        telemetry.addData("Smoothed Error", "X:%.3f Y:%.3f", smoothedXError, smoothedYError);

        // Calculate adjustments
        double spinAdjustment = -smoothedXError * SPIN_GAIN;
        double tiltAdjustment = -smoothedYError * TILT_GAIN;

        // HORIZONTAL TRACKING (Spin servo) with wraparound
        // Only move if adjustment is significant enough
        if (Math.abs(spinAdjustment) >= MIN_SERVO_MOVEMENT) {
            double oldSpin = spinPosition;
            spinPosition += spinAdjustment;

            // Handle wraparound for continuous rotation servo
            if (SPIN_WRAPAROUND_ENABLED) {
                if (spinPosition < SPIN_MIN) {
                    // Wrap from 0.1 to 1.0
                    spinPosition = SPIN_MAX - (SPIN_MIN - spinPosition);
                    telemetry.addData("Spin", "%.3f → %.3f (wrapped left)", oldSpin, spinPosition);
                } else if (spinPosition > SPIN_MAX) {
                    // Wrap from 1.0 to 0.1
                    spinPosition = SPIN_MIN + (spinPosition - SPIN_MAX);
                    telemetry.addData("Spin", "%.3f → %.3f (wrapped right)", oldSpin, spinPosition);
                } else {
                    telemetry.addData("Spin", "%.3f → %.3f (Δ%.3f)", oldSpin, spinPosition, spinAdjustment);
                }
            } else {
                // Standard clamping (no wraparound)
                spinPosition = Math.max(SPIN_MIN, Math.min(SPIN_MAX, spinPosition));
                telemetry.addData("Spin", "%.3f → %.3f (Δ%.3f)", oldSpin, spinPosition, spinAdjustment);
            }

            Limelightspin.setPosition(spinPosition);
        } else {
            telemetry.addData("Spin", "%.3f (settled)", spinPosition);
        }

        // VERTICAL TRACKING (Tilt servo)
        // Only move if adjustment is significant enough
        if (Math.abs(tiltAdjustment) >= MIN_SERVO_MOVEMENT) {
            double oldTilt = tiltPosition;
            tiltPosition += tiltAdjustment;
            tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));

            telemetry.addData("Tilt", "%.3f → %.3f (Δ%.3f)", oldTilt, tiltPosition, tiltAdjustment);
            Limelighttilt.setPosition(tiltPosition);
        } else {
            telemetry.addData("Tilt", "%.3f (settled)", tiltPosition);
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