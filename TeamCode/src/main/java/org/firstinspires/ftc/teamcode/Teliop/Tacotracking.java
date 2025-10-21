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

@TeleOp(name = "Taco tracking", group = "test")
public class Tacotracking extends LinearOpMode {
    public DcMotorEx frontLeft = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;

    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public Servo Limelightspin = null;
    public Servo Limelighttilt = null;

    double shooterLeftPower = 0.0;
    double shooterRightPower = 0.0;
    
    // Limelight servo tracking variables
    private double spinPosition = 0.5;
    private double tiltPosition = 0.1;
    private double smoothedXError = 0.0;
    private double smoothedYError = 0.0;
    private static final double SMOOTHING_FACTOR = 0.3;
    private static final double SPIN_GAIN = 0.008;
    private static final double TILT_GAIN = 0.005;
    private static final double DEADBAND = 0.15;
    private static final double MIN_SPEED = 0.02;
    private static final double MAX_SPEED = 0.25;
    private static final double TILT_MIN = 0.0;
    private static final double TILT_MAX = 0.2;
    
    boolean limelightTrackingEnabled = false;

    public DcMotorEx intakeWheels = null;
    public ColorSensor colorSensor = null;

    double intakeWheelsPower = 0.0;

    boolean intakeRunning = false;
    boolean lastAPress = false;
    boolean lastBPress = false;

    boolean lastYPress = false;
    boolean lastXPress = false;
    int shooterMode = 0;
    // Mode 0: Off
    // Mode 1: Fixed 60%
    // Mode 2: Manual adjustment (X button toggle)
    // Mode 3: Auto-aim with tracking (Y button)

    boolean humanPlayerMode = false;

    // Variables for non-blocking intake sequence
    private long intakeSequenceStartTime = 0;
    private int intakeSequenceStep = 0;
    private boolean intakeSequenceRunning = false;

    // Variables for ball counting
    private int ballCount = 0;
    private boolean ballDetected = false;
    private int colorThreshold = 100;

    // AprilTag Vision variables
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean autoAimEnabled = false;
    private double targetDistance = 0.0;
    
    // Constants for shooter power calculation
    private static final double TARGET_HEIGHT_ABOVE_TAG = 0.30; // 30 cm in meters
    private static final double SHOOTER_HEIGHT = 0.45;
    private static final double APRILTAG_HEIGHT = 0.15;
    
    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelightspin = hardwareMap.get(Servo.class, "Limelightspin");
        Limelighttilt = hardwareMap.get(Servo.class, "Limelighttilt");
        
        // Set initial Limelight positions
        Limelighttilt.setPosition(tiltPosition);
        Limelightspin.setPosition(0.5);

        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setPower(shooterRightPower);
        shooterLeft.setPower(shooterLeftPower);

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addData("Status", "Initialized - Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Manual Limelight position control with back/start buttons
            if (gamepad1.back) {
                limelightTrackingEnabled = false;
                Limelightspin.setPosition(0);
                Limelighttilt.setPosition(0.05);
                spinPosition = 0.5;
                tiltPosition = 0.05;
            }

            if (gamepad1.start) {
                limelightTrackingEnabled = false;
                Limelightspin.setPosition(1);
                Limelighttilt.setPosition(0.2);
                spinPosition = 0.5;
                tiltPosition = 0.2;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.leftBumperWasPressed() && !humanPlayerMode && !autoAimEnabled) {
                shooterLeftPower -= 0.05;
                shooterRightPower -= 0.05;

                if (shooterLeftPower < 0.0) shooterLeftPower = 1.0;
                if (shooterRightPower < 0.0) shooterRightPower = 1.0;
            }

            if (gamepad1.rightBumperWasPressed() && !humanPlayerMode && !autoAimEnabled) {
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

            // X button - Manual shooter mode (original behavior)
            if (gamepad1.x && !lastXPress && !humanPlayerMode) {
                if (shooterMode == 2) {
                    shooterMode = 0;
                } else {
                    shooterMode = 2;
                    autoAimEnabled = false;
                    limelightTrackingEnabled = false;
                    Limelightspin.setPosition(0.5);
                }
            }
            lastXPress = gamepad1.x;

            // Y button - Auto-aim with Limelight tracking
            if (gamepad1.y && !lastYPress && !humanPlayerMode) {
                if (autoAimEnabled) {
                    autoAimEnabled = false;
                    limelightTrackingEnabled = false;
                    shooterMode = 0;
                    Limelightspin.setPosition(0.5);
                } else {
                    autoAimEnabled = true;
                    limelightTrackingEnabled = true;
                    shooterMode = 3;
                }
            }
            lastYPress = gamepad1.y;

            // Handle shooter modes
            if (autoAimEnabled && shooterMode == 3) {
                // Auto-aim mode with tracking
                double calculatedPower = calculateShooterPower();
                
                if (calculatedPower > 0) {
                    shooterLeftPower = calculatedPower;
                    shooterRightPower = calculatedPower;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                    
                    // Track AprilTag with Limelight servos
                    if (limelightTrackingEnabled) {
                        trackAprilTagWithLimelight();
                    }
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                    if (limelightTrackingEnabled) {
                        Limelightspin.setPosition(0.5);
                    }
                }
            } else if (shooterMode == 2) {
                // Manual mode - use bumpers/dpad to adjust
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else if (shooterMode == 1) {
                // Fixed 60% mode
                shooterRightPower = 0.60;
                shooterLeftPower = 0.60;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            if (gamepad1.a && !lastAPress && !humanPlayerMode) {
                if (intakeRunning && intakeWheelsPower == -1.0) {
                    intakeRunning = false;
                } else {
                    intakeRunning = true;
                    intakeWheelsPower = -1.0;
                }
            }
            lastAPress = gamepad1.a;

            if (gamepad1.b && !lastBPress && !humanPlayerMode) {
                if (intakeRunning && intakeWheelsPower == 1.0) {
                    intakeRunning = false;
                } else {
                    intakeRunning = true;
                    intakeWheelsPower = 1.0;
                }
            }
            lastBPress = gamepad1.b;

            // Non-blocking intake sequence
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

                    if (cycleTime < 1000) {
                        intakeWheels.setPower(0);
                    } else if (cycleTime < 1500) {
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

            // Ball counting logic
            int totalColor = colorSensor.red() + colorSensor.green() + colorSensor.blue();

            if (totalColor > colorThreshold && !ballDetected) {
                ballDetected = true;
                ballCount++;
            } else if (totalColor <= colorThreshold && ballDetected) {
                ballDetected = false;
            }

            if (gamepad1.dpadDownWasPressed()) {
                humanPlayerMode = !humanPlayerMode;
                shooterRightPower = 0;
                shooterLeftPower = 0;
                autoAimEnabled = false;
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

            // Telemetry
            telemetry.addData("shooter counts/s", "%.2f", shooterRight.getVelocity());
            telemetry.addData("shooter", "%.2f", shooterRightPower);
            telemetry.addData("shooter actual", "%.2f", shooterRight.getPower());
            telemetry.addData("Limelight Tracking", "%b", limelightTrackingEnabled);
            telemetry.addData("Limelight Spin Pos", "%.3f", spinPosition);
            telemetry.addData("Limelight Tilt Pos", "%.3f", tiltPosition);
            telemetry.addData("intake power", "%.2f", intakeWheelsPower);
            telemetry.addData("intake power actual", "%.2f", intakeWheels.getPower());
            telemetry.addData("Human Player Mode", "%b", humanPlayerMode);
            telemetry.addData("Shooter Mode", "%d", shooterMode);
            telemetry.addData("Auto-Aim Enabled", "%b", autoAimEnabled);
            telemetry.addData("Target Distance", "%.2f m", targetDistance);
            telemetry.addData("Intake Sequence Running", "%b", intakeSequenceRunning);
            telemetry.addData("Sequence Step", "%d", intakeSequenceStep);
            telemetry.addData("Ball Count", "%d", ballCount);
            telemetry.addData("Color Total", "%d", colorSensor.red() + colorSensor.green() + colorSensor.blue());
            telemetry.update();
        }
        
        visionPortal.close();
    }

    /**
     * Initialize AprilTag detection processor
     */
    private void initAprilTag() {
        // Create custom AprilTag library for 210mm x 210mm tags
        double tagSize = 0.21; // 210mm in meters
        
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        tagLibraryBuilder.addTag(20, "Target Tag 20", tagSize, DistanceUnit.METER);
        tagLibraryBuilder.addTag(24, "Target Tag 24", tagSize, DistanceUnit.METER);
        
        AprilTagLibrary customTagLibrary = tagLibraryBuilder.build();
        
        // Create the AprilTag processor with custom tag library
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(customTagLibrary)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // Create the vision portal using the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Calculate shooter power based on distance to AprilTag
     * Aims 30 cm above the AprilTag
     * Uses a simple linear interpolation based on distance
     * @return calculated shooter power (0.0 to 1.0), or 0 if no tag detected
     */
    private double calculateShooterPower() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        if (currentDetections == null || currentDetections.isEmpty()) {
            telemetry.addData("AprilTag", "No tags detected");
            targetDistance = 0.0;
            return 0.0;
        }

        // Use the first detected AprilTag
        AprilTagDetection detection = currentDetections.get(0);
        
        // Get range (distance) from robot to tag in meters
        targetDistance = detection.ftcPose.range;
        
        // Calculate height difference
        double targetHeight = APRILTAG_HEIGHT + TARGET_HEIGHT_ABOVE_TAG;
        double heightDifference = targetHeight - SHOOTER_HEIGHT;
        
        // Simple distance-based power mapping
        double[][] powerMap = {
            {0.5, 0.35},
            {1.0, 0.45},
            {1.5, 0.55},
            {2.0, 0.65},
            {2.5, 0.75},
            {3.0, 0.85},
            {3.5, 0.95},
            {4.0, 1.00}
        };
        
        double calculatedPower = 0.5;
        
        // Linear interpolation between calibration points
        if (targetDistance <= powerMap[0][0]) {
            calculatedPower = powerMap[0][1];
        } else if (targetDistance >= powerMap[powerMap.length - 1][0]) {
            calculatedPower = powerMap[powerMap.length - 1][1];
        } else {
            for (int i = 0; i < powerMap.length - 1; i++) {
                if (targetDistance >= powerMap[i][0] && targetDistance <= powerMap[i + 1][0]) {
                    double d1 = powerMap[i][0];
                    double p1 = powerMap[i][1];
                    double d2 = powerMap[i + 1][0];
                    double p2 = powerMap[i + 1][1];
                    
                    calculatedPower = p1 + (targetDistance - d1) * (p2 - p1) / (d2 - d1);
                    break;
                }
            }
        }
        
        // Add slight adjustment for height difference
        if (heightDifference > 0.1) {
            calculatedPower += heightDifference * 0.1;
        } else if (heightDifference < -0.1) {
            calculatedPower -= Math.abs(heightDifference) * 0.05;
        }
        
        // Clamp power between safe limits
        calculatedPower = Math.max(0.30, Math.min(1.0, calculatedPower));
        
        telemetry.addData("AprilTag ID", detection.id);
        telemetry.addData("Distance", "%.2f m", targetDistance);
        telemetry.addData("Height Diff", "%.2f m", heightDifference);
        telemetry.addData("Calculated Power", "%.2f", calculatedPower);
        
        return calculatedPower;
    }

    /**
     * Track AprilTag using Limelight servos with smooth control
     */
    private void trackAprilTagWithLimelight() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        if (currentDetections == null || currentDetections.isEmpty()) {
            // No tag detected - stop spinning
            Limelightspin.setPosition(0.5);
            return;
        }

        // Use the first detected AprilTag
        AprilTagDetection detection = currentDetections.get(0);
        
        // Get image dimensions (adjust to your camera resolution)
        int imageWidth = 1280;
        int imageHeight = 720;

        // Calculate error from center (normalized -1 to 1)
        double xError = (detection.center.x - imageWidth / 2.0) / (imageWidth / 2.0);
        double yError = (detection.center.y - imageHeight / 2.0) / (imageHeight / 2.0);

        // Apply exponential moving average for smoothing
        smoothedXError = SMOOTHING_FACTOR * xError + (1 - SMOOTHING_FACTOR) * smoothedXError;
        smoothedYError = SMOOTHING_FACTOR * yError + (1 - SMOOTHING_FACTOR) * smoothedYError;

        // Apply deadband to prevent jitter
        double finalXError = Math.abs(smoothedXError) < DEADBAND ? 0 : smoothedXError;
        double finalYError = Math.abs(smoothedYError) < DEADBAND ? 0 : smoothedYError;

        // Control spin (continuous rotation servo)
        if (Math.abs(finalXError) > 0.3) {
            double spinSpeed = finalXError * SPIN_GAIN;

            // Apply minimum speed threshold to overcome friction
            if (Math.abs(spinSpeed) > 0 && Math.abs(spinSpeed) < MIN_SPEED) {
                spinSpeed = Math.signum(spinSpeed) * MIN_SPEED;
            }

            // Limit maximum speed
            spinSpeed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, spinSpeed));
            Limelightspin.setPosition(0.5 + spinSpeed);
            spinPosition = 0.5 + spinSpeed;
        } else {
            Limelightspin.setPosition(0.5); // Stop at center
            spinPosition = 0.5;
            smoothedXError = 0.0;  // Reset smoothing when stopped
        }

        // Control tilt (standard servo with limited range)
        if (Math.abs(finalYError) > 0.3) {
            double tiltAdjustment = -finalYError * TILT_GAIN;
            tiltPosition += tiltAdjustment;

            // Clamp tilt to allowed range (0.0 to 0.2)
            tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));
            Limelighttilt.setPosition(tiltPosition);
        }
        
        telemetry.addData("X Error", "%.2f", xError);
        telemetry.addData("Y Error", "%.2f", yError);
        telemetry.addData("Tag Center X", "%.0f", detection.center.x);
        telemetry.addData("Tag Center Y", "%.0f", detection.center.y);
    }
}

//hi luke