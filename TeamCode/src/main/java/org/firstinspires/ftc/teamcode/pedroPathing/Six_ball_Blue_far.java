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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Six Ball Blue Far", group = "Autonomous")
@Configurable
public class Six_ball_Blue_far extends OpMode {

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
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;
    private boolean firstShot = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);
        initializeHardware();

        // Start shooter spinning in init
        shooterLeft.setPower(0.66);
        shooterRight.setPower(0.66);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void initializeHardware() {
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

        // Initialize servo closed
        stop.setPosition(0.9);
    }

    private boolean updateShooter() {
        if (shooterSpinningUp) {
            double elapsed = shooterTimer.milliseconds();

            // Only wait for spin-up on first shot
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
                intakeWheels.setPower(-0.8);
            } else if (elapsed < 2200) {
                intakeWheels.setPower(0);
            } else {
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 5) {
                    shooterPulsing = false;
                    intakeWheels.setPower(0);
                    stop.setPosition(0.9); // Close servo
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
        stop.setPosition(0); // Open servo

        if (firstShot) {
            shooterSpinningUp = true;
            shooterPulsing = false;
            firstShot = false;
        } else {
            // Skip spin-up for subsequent shots
            shooterSpinningUp = false;
            shooterPulsing = true;
            shooterPulseCount = 0;
            intakeWheels.setPower(-1.0);
        }

        shooterTimer.reset();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(64.662, 0.851), new Pose(61.046, 17.016)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.046, 17.016),
                            new Pose(50.411, 23.397),
                            new Pose(46.157, 30.629)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(46.157, 30.629), new Pose(10.635, 30.629)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(10.635, 30.629), new Pose(46.157, 29.991)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(46.157, 29.991),
                            new Pose(50.198, 23.185),
                            new Pose(61.046, 17.229)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 2;
                }
                break;

            case 2:
                if (updateShooter()) {
                    pathState = 3;
                }
                break;

            case 3:
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(-1.0);
                    follower.followPath(paths.Path3, false);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    pathState = 6;
                }
                break;

            case 6:
                follower.followPath(paths.Path4);
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    pathState = 8;
                }
                break;

            case 8:
                follower.followPath(paths.Path5);
                pathState = 9;
                break;

            case 9:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 10;
                }
                break;

            case 10:
                if (updateShooter()) {
                    pathState = 11;
                }
                break;

            case 11:
                telemetry.addData("State", "✓ COMPLETE ✓");
                break;
        }
    }
}