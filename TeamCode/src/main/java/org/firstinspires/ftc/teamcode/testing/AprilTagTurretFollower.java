package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Turret Follower")
public class AprilTagTurretFollower extends LinearOpMode {

    // Servo hardware
    private Servo spinServo;
    private Servo tiltServo;

    // Vision processing
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Turret control parameters
    private double spinPosition = 0.5;  // Center position for continuous servo (0-1)
    private double tiltPosition = 0.1;  // Middle of 0-0.2 range

    // Exponential moving average for smoothing
    private double smoothedXError = 0.0;
    private double smoothedYError = 0.0;
    private static final double SMOOTHING_FACTOR = 0.3;  // 0-1, lower = more smoothing

    // PID-like constants for smooth tracking
    private static final double SPIN_GAIN = 0.008;  // Reduced for smoother tracking
    private static final double TILT_GAIN = 0.005;  // Reduced for smoother tracking
    private static final double DEADBAND = 0.15;    // Increased to prevent jitter
    private static final double MIN_SPEED = 0.02;   // Minimum movement speed
    private static final double MAX_SPEED = 0.25;   // Maximum movement speed

    // Tilt servo limits
    private static final double TILT_MIN = 0.0;
    private static final double TILT_MAX = 0.2;

    // Camera field of view (degrees) - adjust for your camera
    private static final double CAMERA_FOV_HORIZONTAL = 60.0;
    private static final double CAMERA_FOV_VERTICAL = 45.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        spinServo = hardwareMap.get(Servo.class, "Limelightspin");
        tiltServo = hardwareMap.get(Servo.class, "Limelighttilt");

        // Set initial positions
        tiltServo.setPosition(tiltPosition);
        spinServo.setPosition(0);  // Stop position for continuous servo

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Info", "Turret will track detected AprilTags");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get AprilTag detections
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                // Track the first detected tag
                AprilTagDetection detection = detections.get(0);
                trackAprilTag(detection);

                // Display tracking info
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Spin Position", "%.3f", spinPosition);
                telemetry.addData("Tilt Position", "%.3f", tiltPosition);
                telemetry.addData("X Center", "%.2f", detection.center.x);
                telemetry.addData("Y Center", "%.2f", detection.center.y);
            } else {
                // No tag detected - stop spinning
                spinServo.setPosition(0);
                telemetry.addData("Status", "No AprilTag detected");
                telemetry.addData("Tilt Position", "%.3f", tiltPosition);
            }

            telemetry.update();
            sleep(20);  // Update at ~50Hz
        }

        // Cleanup
        visionPortal.close();
    }

    /**
     * Track an AprilTag by adjusting turret position
     */
    private void trackAprilTag(AprilTagDetection detection) {
        // Get image dimensions
        int imageWidth = 1280;   // Adjust to your camera resolution
        int imageHeight = 720;  // Adjust to your camera resolution

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
            spinServo.setPosition(0.5 + spinSpeed);
        } else {
            spinServo.setPosition(0); // Stop
            smoothedXError = 0.2;  // Reset smoothing when stopped
        }

        // Control tilt (standard servo with limited range)
        if (Math.abs(finalYError) > 0.3) {
            double tiltAdjustment = -finalYError * TILT_GAIN;
            tiltPosition += tiltAdjustment;

            // Clamp tilt to allowed range
            tiltPosition = Math.max(TILT_MIN, Math.min(TILT_MAX, tiltPosition));
            tiltServo.setPosition(tiltPosition);
        }
    }

    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}