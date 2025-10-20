package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Taco Decode - AprilTag Auto")
public class tacoDecode extends LinearOpMode {
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

    public DcMotorEx intakeWheels = null;
    public ColorSensor colorSensor = null;

    double intakeWheelsPower = 0.0;

    boolean intakeRunning = false;
    boolean lastAPress = false;
    boolean lastBPress = false;

    boolean lastYPress = false;
    boolean lastXPress = false;
    int shooterMode = 0;

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
    private static final double TARGET_HEIGHT_ABOVE_TAG = 0.10; // 30 cm in meters
    private static final double GRAVITY = 9.81; // m/s^2
    private static final double SHOOTER_HEIGHT = 0.30; // Adjust based on your robot's shooter height in meters
    private static final double APRILTAG_HEIGHT = 0.70x; // Standard AprilTag height in meters

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

            if (gamepad1.back) {
                Limelightspin.setPosition(0);
                Limelighttilt.setPosition(0.05);
            }

            if (gamepad1.start) {
                Limelightspin.setPosition(1);
                Limelighttilt.setPosition(0.2);
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

            if (gamepad1.x && !lastXPress && !humanPlayerMode) {
                if (shooterMode == 2) {
                    shooterMode = 0;
                    autoAimEnabled = false;
                } else {
                    shooterMode = 2;
                    autoAimEnabled = false;
                }
            }
            lastXPress = gamepad1.x;

            // Y button now enables AprilTag auto-aim mode
            if (gamepad1.y && !lastYPress && !humanPlayerMode) {
                if (autoAimEnabled) {
                    // Turn off auto-aim
                    autoAimEnabled = false;
                    shooterMode = 0;
                } else {
                    // Enable auto-aim mode
                    autoAimEnabled = true;
                    shooterMode = 3; // New mode for auto-aim
                }
            }
            lastYPress = gamepad1.y;

            // Handle shooter modes
            if (autoAimEnabled && shooterMode == 3) {
                // Auto-aim mode - calculate shooter power based on AprilTag distance
                double calculatedPower = calculateShooterPower();

                if (calculatedPower > 0) {
                    shooterLeftPower = calculatedPower;
                    shooterRightPower = calculatedPower;
                    shooterRight.setPower(shooterRightPower);
                    shooterLeft.setPower(shooterLeftPower);
                } else {
                    // No AprilTag detected, maintain current power or stop
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                }
            } else if (shooterMode == 2) {
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else if (shooterMode == 1) {
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

        // Close vision portal when done
        visionPortal.close();
    }

    /**
     * Initialize AprilTag detection processor
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
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

        // Get range (distance) from robot to tag
        targetDistance = detection.ftcPose.range; // Distance in meters
        double bearing = Math.toRadians(detection.ftcPose.bearing); // Angle to tag
        double elevation = Math.toRadians(detection.ftcPose.elevation); // Vertical angle

        // Calculate actual horizontal distance (accounting for angle)
        double horizontalDistance = targetDistance * Math.cos(elevation);

        // Calculate height difference
        // Target is 30cm above the AprilTag
        double targetHeight = APRILTAG_HEIGHT + TARGET_HEIGHT_ABOVE_TAG;
        double heightDifference = targetHeight - SHOOTER_HEIGHT;

        // Physics calculation for projectile motion
        // Using: v = sqrt((g * d^2) / (2 * cos^2(θ) * (d * tan(θ) - h)))
        // Simplified for horizontal launch: v = sqrt(g * d^2 / (2 * h))

        double requiredVelocity;
        if (heightDifference > 0) {
            // Shooting upward
            double angle = Math.atan2(heightDifference, horizontalDistance);
            requiredVelocity = Math.sqrt(
                    (GRAVITY * horizontalDistance * horizontalDistance) /
                            (2 * Math.cos(angle) * Math.cos(angle) *
                                    (horizontalDistance * Math.tan(angle) - heightDifference))
            );
        } else {
            // Shooting downward or level
            requiredVelocity = Math.sqrt(
                    (GRAVITY * horizontalDistance * horizontalDistance) /
                            (2 * Math.abs(heightDifference))
            );
        }

        // Convert velocity to motor power
        // This is a rough estimation - you'll need to calibrate this for your robot
        // Assuming max velocity at power 1.0 is about 5 m/s (adjust based on testing)
        double maxVelocity = 5.0; // meters per second at full power
        double calculatedPower = requiredVelocity / maxVelocity;

        // Clamp power between reasonable limits
        calculatedPower = Math.max(0.3, Math.min(1.0, calculatedPower));

        telemetry.addData("AprilTag ID", detection.id);
        telemetry.addData("Distance", "%.2f m", targetDistance);
        telemetry.addData("Horizontal Dist", "%.2f m", horizontalDistance);
        telemetry.addData("Height Diff", "%.2f m", heightDifference);
        telemetry.addData("Required Velocity", "%.2f m/s", requiredVelocity);
        telemetry.addData("Calculated Power", "%.2f", calculatedPower);

        return calculatedPower;
    }
}

//hi luke