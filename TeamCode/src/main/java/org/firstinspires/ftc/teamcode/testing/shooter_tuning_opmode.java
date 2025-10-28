package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Shooter Speed Tuning", group = "Tuning")
public class shooter_tuning_opmode extends OpMode {

    // ========================================
    // TUNING CONFIGURATION
    // ========================================
    
    // APRILTAG SIZE - Must match your actual tag size
    private static final double APRILTAG_SIZE = 0.166;  // 166mm
    
    // TARGET APRILTAG IDs (match your field setup)
    private static final int TARGET_TAG_ID_1 = 20;  // Blue goal
    private static final int TARGET_TAG_ID_2 = 24;  // Alternative target
    
    // SPEED INCREMENT
    private static final double SPEED_INCREMENT = 0.01;  // A button increment
    
    // ========================================
    // END OF TUNING AREA
    // ========================================

    // AprilTag Webcam Class
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Hardware
    private DcMotorEx shooterLeft, shooterRight;

    // State Variables
    private boolean lastAPress = false;
    private boolean lastBPress = false;
    private boolean lastXPress = false;
    
    private double shooterSpeed = 0.0;
    private double currentVoltage = 12.5;
    
    // Data collection
    private int dataPointCount = 0;

    @Override
    public void init() {
        // Initialize shooter motors
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // Match your teleop motor directions
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize AprilTag vision system
        initAprilTag();

        telemetry.addData("Status", "Initialized - Shooter Speed Tuning");
        telemetry.addData("Controls", "A: +0.01 | B: -0.01 | X: Reset");
        telemetry.addData("Target Tags", "ID %d & ID %d", TARGET_TAG_ID_1, TARGET_TAG_ID_2);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update battery voltage
        currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Update AprilTag detections
        updateAprilTags();

        // ========================================
        // SPEED ADJUSTMENT CONTROLS
        // ========================================
        
        // A Button: Increase speed by 0.01
        if (gamepad1.a && !lastAPress) {
            shooterSpeed += SPEED_INCREMENT;
            if (shooterSpeed > 1.0) {
                shooterSpeed = 1.0;
            }
            dataPointCount++;
        }
        lastAPress = gamepad1.a;

        // B Button: Decrease speed by 0.01
        if (gamepad1.b && !lastBPress) {
            shooterSpeed -= SPEED_INCREMENT;
            if (shooterSpeed < 0.0) {
                shooterSpeed = 0.0;
            }
        }
        lastBPress = gamepad1.b;

        // X Button: Reset speed to 0
        if (gamepad1.x && !lastXPress) {
            shooterSpeed = 0.0;
        }
        lastXPress = gamepad1.x;

        // Set shooter motor powers
        shooterLeft.setPower(shooterSpeed);
        shooterRight.setPower(shooterSpeed);

        // ========================================
        // TELEMETRY DISPLAY
        // ========================================
        telemetry.addData("=== SHOOTER TUNING ===", "");
        telemetry.addData("Speed", "%.3f", shooterSpeed);
        telemetry.addData("Left RPM", "%.0f", shooterLeft.getVelocity() * 60.0 / 28.0);
        telemetry.addData("Right RPM", "%.0f", shooterRight.getVelocity() * 60.0 / 28.0);
        telemetry.addData("Battery", "%.2f V", currentVoltage);
        telemetry.addData("Data Points", dataPointCount);
        
        telemetry.addData("", ""); // Spacer
        
        // Display AprilTag detection info for target tag 1
        AprilTagDetection tag1 = getTagBySpecificID(TARGET_TAG_ID_1);
        if (tag1 != null) {
            displayDetectionTelemetry(tag1);
        } else {
            telemetry.addData("Tag ID %d", "Not Detected", TARGET_TAG_ID_1);
        }
        
        telemetry.addData("", ""); // Spacer
        
        // Display AprilTag detection info for target tag 2
        AprilTagDetection tag2 = getTagBySpecificID(TARGET_TAG_ID_2);
        if (tag2 != null) {
            displayDetectionTelemetry(tag2);
        } else {
            telemetry.addData("Tag ID %d", "Not Detected", TARGET_TAG_ID_2);
        }
        
        telemetry.addData("", ""); // Spacer
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Speed +0.01");
        telemetry.addData("B", "Speed -0.01");
        telemetry.addData("X", "Reset to 0");
        
        // Dashboard data
        sendToDashboard(tag1);
        
        telemetry.update();
    }

    private void initAprilTag() {
        // Build AprilTag Library with your specific tags
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        double tagSize = APRILTAG_SIZE;
        tagLibraryBuilder.addTag(TARGET_TAG_ID_1, "Target Tag " + TARGET_TAG_ID_1, tagSize, DistanceUnit.METER);
        tagLibraryBuilder.addTag(TARGET_TAG_ID_2, "Target Tag " + TARGET_TAG_ID_2, tagSize, DistanceUnit.METER);
        AprilTagLibrary customTagLibrary = tagLibraryBuilder.build();

        // Create the AprilTag processor with visual aids
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(customTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        
        visionPortal = builder.build();
    }

    private void updateAprilTags() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    private List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    private AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    private void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {
            return;
        }

        // Display detailed AprilTag information (following video format)
        telemetry.addLine(String.format("\n==== AprilTag ID %d ====", detectedID.id));
        telemetry.addLine(String.format("XYZ: %.2f / %.2f / %.2f (m)", 
            detectedID.ftcPose.x, detectedID.ftcPose.y, detectedID.ftcPose.z));
        telemetry.addLine(String.format("PRY: %.1f / %.1f / %.1f (deg)", 
            detectedID.ftcPose.pitch, detectedID.ftcPose.roll, detectedID.ftcPose.yaw));
        telemetry.addLine(String.format("RBE: %.2f / %.1f / %.1f (m, deg, deg)", 
            detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));
        
        // Power map format for easy copy-paste
        telemetry.addData("Power Map Entry", "{%.2f, %.2f}", 
            detectedID.ftcPose.range, shooterSpeed);
    }

    private void sendToDashboard(AprilTagDetection primaryTag) {
        // FTC Dashboard telemetry
        telemetry.addData("dashboard_speed", shooterSpeed);
        telemetry.addData("dashboard_voltage", currentVoltage);
        telemetry.addData("dashboard_left_rpm", shooterLeft.getVelocity() * 60.0 / 28.0);
        telemetry.addData("dashboard_right_rpm", shooterRight.getVelocity() * 60.0 / 28.0);
        telemetry.addData("dashboard_data_points", dataPointCount);
        
        if (primaryTag != null) {
            telemetry.addData("dashboard_tag_id", primaryTag.id);
            telemetry.addData("dashboard_range", primaryTag.ftcPose.range);
            telemetry.addData("dashboard_bearing", primaryTag.ftcPose.bearing);
            telemetry.addData("dashboard_elevation", primaryTag.ftcPose.elevation);
            telemetry.addData("dashboard_x", primaryTag.ftcPose.x);
            telemetry.addData("dashboard_y", primaryTag.ftcPose.y);
            telemetry.addData("dashboard_z", primaryTag.ftcPose.z);
            
            // Formatted for power map
            telemetry.addData("dashboard_map_entry", 
                String.format("{%.2f, %.2f}", primaryTag.ftcPose.range, shooterSpeed));
        }
    }

    @Override
    public void stop() {
        // Stop shooter motors
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        
        // Close vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}