package org.firstinspires.ftc.teamcode.pedroPathing.OldAutos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Other.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Autonomous(name = "Motif Red" , group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Motif_red extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses - MIRRORED FOR RED SIDE
    private final Pose startPose = new Pose(72, 24, Math.toRadians(270)); // Start Pose mirrored (Y: 144-120=24, Heading: 270)
    private final Pose scorePose = new Pose(60, 114, Math.toRadians(20)); // Scoring Pose mirrored (Y: 144-30=114, Heading: 20)
    private final Pose PPGPose = new Pose(100, 60.5, Math.toRadians(0)); // Highest (First Set) mirrored (Y: 144-83.5=60.5)
    private final Pose PGPPose = new Pose(100, 84.5, Math.toRadians(0)); // Middle (Second Set) mirrored (Y: 144-59.5=84.5)
    private final Pose GPPPose = new Pose(100, 108.5, Math.toRadians(0)); // Lowest (Third Set) mirrored (Y: 144-35.5=108.5)

    // Initialize variables for paths
    private PathChain grabPPG;
    private PathChain scorePPG;
    private PathChain grabPGP;
    private PathChain scorePGP;
    private PathChain grabGPP;
    private PathChain scoreGPP;

    //set April Tag values to specific patterns - RED SIDE TAGS
    private static final int PPG_TAG_ID = 13;
    private static final int PGP_TAG_ID = 12;
    private static final int GPP_TAG_ID = 11;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value

    private int foundID; // Current state machine value, dictates which one to run

    // Declare motors (but don't initialize yet)
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public DcMotorEx frontLeft = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;


    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    // a place to put your intake and shooting functions
    public void intakeArtifacts() {
        intakeWheels.setPower(-1);
        sleep(3000); // spin for 3 seconds
        intakeWheels.setPower(0); // stop intake
    }

    public void shootArtifacts() {
        //intakeWheels.setPower(1);
        //sleep(500);
        shooterLeft.setPower(0.57);
        shooterRight.setPower(0.57);
        sleep(3000);
        for (int i = 0; i < 6; i++) {  // repeat 6 times
            // Spin intake
            intakeWheels.setPower(-1.0);
            sleep(200); // spin for 0.2 seconds

            // Stop intake
            intakeWheels.setPower(0);
            sleep(1500); // stop for 1.5 seconds
        }
        // Stop shooters after done
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize hardware here (inside runOpMode)
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        initAprilTag();

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == PPG_TAG_ID) {
                        // call lines for the PPG pattern
                        buildPathsPPG();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = PPG_TAG_ID;
                        break;  // don't look any further.
                    } else if (detection.id == PGP_TAG_ID) {
                        // call lines for the PGP pattern
                        buildPathsPGP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = PGP_TAG_ID;
                        break;  // don't look any further.
                    } else if (detection.id == GPP_TAG_ID) {
                        // call lines for the GPP pattern
                        buildPathsGPP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = GPP_TAG_ID;
                        break;  // don't look any further.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Update the state machine
            if (foundID == PPG_TAG_ID) {
                updateStateMachinePPG();
            } else if (foundID == PGP_TAG_ID) {
                updateStateMachinePGP();
            } else if (foundID == GPP_TAG_ID) {
                updateStateMachineGPP();
            }

            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void buildPathsPPG() {
        // basically just plotting the points for the lines that score the PPG pattern
        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PPGPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, scorePose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsPGP() {
        // basically just plotting the points for the lines that score the PGP pattern
        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PGPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, scorePose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsGPP() {
        // basically just plotting the points for the lines that score the GPP pattern
        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    //below is the state machine for each pattern
    public void updateStateMachinePPG() {
        switch (pathStatePPG) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabPPG);
                setpathStatePPG(1);
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Run intake while moving
                    intakeArtifacts();
                    // Move to the scoring location
                    follower.followPath(scorePPG);
                    setpathStatePPG(2);
                }
                break;
            case 2:
                // Wait until at scoring position
                if (!follower.isBusy()) {
                    // Shoot the artifacts
                    shootArtifacts();
                    setpathStatePPG(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }

    public void updateStateMachinePGP() {
        switch (pathStatePGP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabPGP);
                setpathStatePGP(1);
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Run intake while moving
                    intakeArtifacts();
                    // Move to the scoring location
                    follower.followPath(scorePGP);
                    setpathStatePGP(2);
                }
                break;
            case 2:
                // Wait until at scoring position
                if (!follower.isBusy()) {
                    // Shoot the artifacts
                    shootArtifacts();
                    setpathStatePGP(-1);
                }
                break;
        }
    }

    public void updateStateMachineGPP() {
        switch (pathStateGPP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabGPP);
                setpathStateGPP(1);
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    // Run intake while moving
                    intakeArtifacts();
                    // Move to the scoring location
                    follower.followPath(scoreGPP);
                    setpathStateGPP(2);
                }
                break;
            case 2:
                // Wait until at scoring position
                if (!follower.isBusy()) {
                    // Shoot the artifacts
                    shootArtifacts();
                    setpathStateGPP(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }

    // Setter methods for pathState variables
    void setpathStatePPG(int newPathState) {
        this.pathStatePPG = newPathState;
    }

    void setpathStatePGP(int newPathState) {
        this.pathStatePGP = newPathState;
    }

    void setpathStateGPP(int newPathState) {
        this.pathStateGPP = newPathState;
    }

    /**
     * start the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        //     exposureControl.setMode(ExposureControl.Mode.Manual);
        //     sleep(50);
        // }
        // exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        // sleep(20);
        // GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        // gainControl.setGain(gain);
        // sleep(20);
    }
}