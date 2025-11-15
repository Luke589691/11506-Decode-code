package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Six Ball Red Far", group = "Autonomous")
@Configurable
public class Six_ball_Red_far extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;

    // Non-blocking timers
    private ElapsedTime shooterTimer = new ElapsedTime();
    private ElapsedTime generalTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;
    private boolean isSecondShot = false;
    private int shootCount = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Red far starting position (mirrored from blue: 72, 8 -> 72, 136)
        follower.setStartingPose(new Pose(72, 136, Math.toRadians(270)));

        paths = new Paths(follower);
        initializeIntakeShooter();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", "RED FAR");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shot Count", shootCount);
        panelsTelemetry.update(telemetry);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.0f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shots", shootCount);
        telemetry.update();
    }

    private void initializeIntakeShooter() {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private boolean updateShooter() {
        if (shooterSpinningUp) {
            double elapsed = shooterTimer.milliseconds();

            // For shots after first: reverse intake for first 100ms during ramp-up
            if (isSecondShot && elapsed < 100) {
                intakeWheels.setPower(1.0);
            } else if (isSecondShot && elapsed >= 100) {
                intakeWheels.setPower(0);
            }

            if (elapsed >= 6000) {
                shooterSpinningUp = false;
                shooterPulsing = true;
                shooterPulseCount = 0;
                shooterTimer.reset();
                intakeWheels.setPower(-1.0);
            }
            return false;
        }

        if (shooterPulsing) {
            double elapsed = shooterTimer.milliseconds();

            if (elapsed < 500) {
                // Pulse phase
                intakeWheels.setPower(-0.8);
            } else if (elapsed < 2200) {
                // Wait phase
                intakeWheels.setPower(0);
            } else {
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 2) {
                    // After 2 pulses, switch to 3 seconds full power
                    shooterPulsing = false;
                    intakeWheels.setPower(-1.0);
                    shooterTimer.reset();
                    generalTimer.reset();
                } else {
                    // Continue pulsing
                    intakeWheels.setPower(-1.0);
                }
            }
            return false;
        }

        // Full power phase after pulsing (only when shooterPulsing is false)
        if (shooterPulseCount >= 2 && !shooterSpinningUp) {
            double elapsed = generalTimer.milliseconds();

            if (elapsed >= 3000) {
                // Done with shooting sequence
                intakeWheels.setPower(0);
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                shooterPulseCount = 0; // Reset for next shot
                return true;
            }
            return false;
        }

        return true;
    }

    private void startShooting(boolean afterFirst) {
        shooterLeft.setPower(0.70);
        shooterRight.setPower(0.70);
        shooterSpinningUp = true;
        shooterPulsing = false;
        isSecondShot = afterFirst;
        shooterTimer.reset();
        shootCount++;
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower) {
            // Path1: Starting position to first scoring position
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(64.662, 143.149),
                                    new Pose(61.046, 126.984)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                    .build();

            // Path2: First scoring to sample pickup area
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.046, 126.984),
                                    new Pose(50.411, 120.603),
                                    new Pose(46.157, 113.371)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))
                    .build();

            // Path3: Pick up sample and go to observation zone
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.157, 113.371),
                                    new Pose(10.635, 113.371)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // Path4: Return from observation zone to sample area
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.635, 113.371),
                                    new Pose(46.157, 114.009)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // Path5: Sample area back to scoring position
            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(46.157, 114.009),
                                    new Pose(50.198, 120.815),
                                    new Pose(61.046, 126.771)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(245))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addData("State", "Starting Path1 - Move to scoring");
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                telemetry.addData("State", "Path1 - Moving");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path1 complete, shooting preload");
                    startShooting(false);
                    pathState = 2;
                }
                break;

            case 2:
                telemetry.addData("State", "Shooting (1st - Preload)");
                if (updateShooter()) {
                    pathState = 3;
                }
                break;

            case 3:
                telemetry.addData("State", "Starting Path2 - Move to samples");
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                telemetry.addData("State", "Path2 - Moving to samples");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path2 complete, starting Path3 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.65);
                    follower.followPath(paths.Path3, false);
                    pathState = 5;
                }
                break;

            case 5:
                telemetry.addData("State", "Path3 - Intake ON (to observation zone)");
                if (!follower.isBusy()) {
                    // Stop intake, we pushed sample into observation zone
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    telemetry.addData("State", "Path3 complete, starting Path4");
                    pathState = 6;
                }
                break;

            case 6:
                telemetry.addData("State", "Starting Path4 - Return to samples");
                follower.followPath(paths.Path4);
                pathState = 9;
                break;

            case 9:
                telemetry.addData("State", "Path4 - Returning");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path4 complete, starting Path5");
                    follower.followPath(paths.Path5);
                    pathState = 10;
                }
                break;

            case 10:
                telemetry.addData("State", "Path5 - Moving to scoring");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path5 complete, shooting final sample");
                    startShooting(true);
                    pathState = 11;
                }
                break;

            case 11:
                telemetry.addData("State", "Shooting (2nd - Final)");
                if (updateShooter()) {
                    pathState = 12;
                }
                break;

            case 12:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                telemetry.addData("Total Shots", shootCount);
                break;

            default:
                telemetry.addData("State", "Unknown state!");
                break;
        }
    }
}