package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Nine ball Red V2", group = "Autonomous")
@Configurable
public class nine_ball_v2_red extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public Servo stop = null;

    // Non-blocking timers
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterPulsing = false;
    private boolean isFirstShot = true;
    private int shootCount = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.971, 131.050, Math.toRadians(35)));

        paths = new Paths(follower);
        initializeIntakeShooter();

        // Start shooter immediately - using blue V2 tuning
        shooterLeft.setPower(0.55);
        shooterRight.setPower(0.55);

        panelsTelemetry.debug("Status", "Initialized");
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
        panelsTelemetry.update(telemetry);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.0f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void initializeIntakeShooter() {
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        stop = hardwareMap.get(Servo.class, "stop");

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop.setPosition(0.9); // Start with stopper closed
    }

    private boolean updateShooter() {
        double elapsed = shooterTimer.milliseconds();

        // First shot only: wait for spin up - using blue V2 timing
        if (isFirstShot && elapsed < 5000) {
            if (!isFirstShot && elapsed < 100) {
                intakeWheels.setPower(1.0);
            } else if (!isFirstShot && elapsed >= 100) {
                intakeWheels.setPower(0);
            }
            return false;
        }

        // After spin-up, start pulsing
        if (!shooterPulsing) {
            shooterPulsing = true;
            shooterPulseCount = 0;
            shooterTimer.reset();
            intakeWheels.setPower(-1.0);
            stop.setPosition(0); // Open stopper
        }

        if (shooterPulsing) {
            // Using blue V2 pulse timing: 400ms on, 1800ms wait
            if (elapsed < 400) {
                intakeWheels.setPower(-0.8);
            } else if (elapsed < 2200) {
                intakeWheels.setPower(0);
            } else {
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 3) {
                    shooterPulsing = false;
                    intakeWheels.setPower(0);
                    stop.setPosition(0.9); // Close stopper
                    return true;
                } else {
                    intakeWheels.setPower(-1.0);
                }
            }
            return false;
        }

        return true;
    }

    private void startShooting() {
        shooterPulsing = false;
        shooterTimer.reset();
        shootCount++;

        if (isFirstShot) {
            isFirstShot = false;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(117.971, 131.050),
                            new Pose(88.374, 93.757)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(30))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.353, 87.281),
                            new Pose(79.000, 93.000),
                            new Pose(99.712, 84.691)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(99.712, 84.691),
                            new Pose(125.353, 84.043)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.353, 84.043),
                            new Pose(71.612, 87.410)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.612, 87.410),
                            new Pose(53.000, 53.000),
                            new Pose(100.101, 60.086)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(100.101, 60.086),
                            new Pose(125.741, 60.216)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(125.741, 60.216),
                            new Pose(70.000, 53.000),
                            new Pose(71.741, 87.410)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.871, 87.410),
                            new Pose(57.000, 63.000),
                            new Pose(105.669, 33.410)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addData("State", "Starting Path1");
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path1 complete, shooting");
                    startShooting();
                    pathState = 2;
                }
                break;

            case 2:
                telemetry.addData("State", "Shooting (1st)");
                if (updateShooter()) {
                    pathState = 3;
                }
                break;

            case 3:
                telemetry.addData("State", "Starting Path2");
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path2 complete, starting Path3 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.50);
                    follower.followPath(paths.Path3, false);
                    pathState = 5;
                }
                break;

            case 5:
                telemetry.addData("State", "Path3 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 6;
                }
                break;

            case 6:
                telemetry.addData("State", "Starting Path4 with intake");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.80);
                follower.followPath(paths.Path4);
                pathState = 7;
                break;

            case 7:
                telemetry.addData("State", "Path4 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    telemetry.addData("State", "Path4 complete, shooting");
                    startShooting();
                    pathState = 8;
                }
                break;

            case 8:
                telemetry.addData("State", "Shooting (2nd)");
                if (updateShooter()) {
                    pathState = 9;
                }
                break;

            case 9:
                telemetry.addData("State", "Starting Path5");
                follower.followPath(paths.Path5);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path5 complete, starting Path6 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.50);
                    follower.followPath(paths.Path6, false);
                    pathState = 11;
                }
                break;

            case 11:
                telemetry.addData("State", "Path6 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 12;
                }
                break;

            case 12:
                telemetry.addData("State", "Starting Path7 with intake");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.80);
                follower.followPath(paths.Path7);
                pathState = 13;
                break;

            case 13:
                telemetry.addData("State", "Path7 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    telemetry.addData("State", "Path7 complete, shooting");
                    startShooting();
                    pathState = 14;
                }
                break;

            case 14:
                telemetry.addData("State", "Shooting (3rd)");
                if (updateShooter()) {
                    pathState = 15;
                }
                break;

            case 15:
                telemetry.addData("State", "Starting Path11 (final)");
                follower.followPath(paths.Path11);
                pathState = 16;
                break;

            case 16:
                telemetry.addData("State", "Path11 - Moving to final position");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path11 complete");
                    pathState = 17;
                }
                break;

            case 17:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                break;

            default:
                telemetry.addData("State", "Unknown state!");
                break;
        }
    }
}