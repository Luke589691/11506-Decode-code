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

@Autonomous(name = "Twelve Ball Red V2", group = "Autonomous")
@Configurable
public class twelve_ball_v2_red extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public Servo stop = null;

    // Non-blocking timers for shooter
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;
    private boolean isFirstShot = true;

    // Track when intake started for half-line timing
    private ElapsedTime intakeTimer = new ElapsedTime();
    private boolean intakeHalfLineDone = false;

    // Configurable shooting heading (in degrees)
    private double shootingHeadingDegrees = 26.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.536, 129.411, Math.toRadians(36)));
        paths = new Paths(follower, shootingHeadingDegrees);

        initializeHardware();

        // Start shooter spinning in init
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
        stop.setPosition(0.5);
    }
    private boolean updateShooter() {
        double elapsed = shooterTimer.milliseconds();

        if (shooterSpinningUp) {
            if (elapsed >= 2000) {  // Reduced from 4000ms to 2000ms
                shooterSpinningUp = false;
                shooterPulsing = true;
                shooterTimer.reset();
            }
            return false;
        }

        if (shooterPulsing) {
            // 4 shots: 800ms intake + 400ms wait each = 4800ms total
            int cycle = (int)(elapsed / 1200);  // Each cycle is 1200ms (800 + 400)
            int phaseTime = (int)(elapsed % 1200);

            if (cycle < 4) {
                intakeWheels.setPower(phaseTime < 800 ? -0.8 : 0);
            } else {
                shooterPulsing = false;
                intakeWheels.setPower(0);
                stop.setPosition(0.5);
                return true;
            }
            return false;
        }

        return true;
    }

    private void startShooting() {
        stop.setPosition(0.9); // Open servo

        if (isFirstShot) {
            // First shot: do spin-up
            shooterSpinningUp = true;
            shooterPulsing = false;
            isFirstShot = false;
        } else {
            // Subsequent shots: V2 standard 100ms intake reversal, then shoot immediately
            intakeWheels.setPower(1.0);
            try { Thread.sleep(100); } catch (InterruptedException e) {}
            intakeWheels.setPower(0);

            shooterSpinningUp = false;
            shooterPulsing = true;
            shooterPulseCount = 0;
            intakeWheels.setPower(-1.0);
        }

        shooterTimer.reset();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower, double shootingHeadingDegrees) {
            double shootingHeadingRad = Math.toRadians(shootingHeadingDegrees);

            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.536, 129.411),
                            new Pose(87.890, 105.397)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(36), shootingHeadingRad)
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 105.397),
                            new Pose(87.666, 84.299)
                    ))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.666, 84.299),
                            new Pose(128.738, 84.075)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(128.738, 84.075),
                            new Pose(87.000, 74.000),
                            new Pose(87.890, 106.070)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootingHeadingRad)
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 106.070),
                            new Pose(88.564, 59.611)
                    ))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564, 59.611),
                            new Pose(133.900, 60.060)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.900, 60.060),
                            new Pose(75.000, 51.000),
                            new Pose(88.115, 106.070)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootingHeadingRad)
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.115, 106.070),
                            new Pose(88.564, 36.269)
                    ))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564, 36.269),
                            new Pose(133.227, 35.596)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.227, 35.596),
                            new Pose(81.000, 78.000),
                            new Pose(68.000, 36.000),
                            new Pose(88.115, 105.845)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
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
                    follower.setMaxPower(0.60);  // Increased from 0.50
                    follower.followPath(paths.Path3, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 5;
                }
                break;

            case 5:
                // Stop intake halfway through the line
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 6;
                }
                break;

            case 6:
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.90);  // Increased from 0.80
                follower.followPath(paths.Path4);
                intakeTimer.reset();
                intakeHalfLineDone = false;
                pathState = 7;
                break;

            case 7:
                // Stop intake halfway through the curve
                if (!intakeHalfLineDone && intakeTimer.milliseconds() > 400) {  // Reduced from 500ms
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    startShooting();
                    pathState = 8;
                }
                break;

            case 8:
                if (updateShooter()) {
                    pathState = 9;
                }
                break;

            case 9:
                follower.followPath(paths.Path5);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.60);  // Increased from 0.50
                    follower.followPath(paths.Path6, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 11;
                }
                break;

            case 11:
                // Stop intake halfway through the line
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 12;
                }
                break;

            case 12:
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.90);  // Increased from 0.80
                follower.followPath(paths.Path7);
                intakeTimer.reset();
                intakeHalfLineDone = false;
                pathState = 13;
                break;

            case 13:
                // Stop intake halfway through the curve
                if (!intakeHalfLineDone && intakeTimer.milliseconds() > 400) {  // Reduced from 500ms
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    startShooting();
                    pathState = 14;
                }
                break;

            case 14:
                if (updateShooter()) {
                    pathState = 15;
                }
                break;

            case 15:
                follower.followPath(paths.Path8);
                pathState = 16;
                break;

            case 16:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.60);  // Increased from 0.50
                    follower.followPath(paths.Path9, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 17;
                }
                break;

            case 17:
                // Stop intake halfway through the line
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 18;
                }
                break;

            case 18:
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.90);  // Increased from 0.80
                follower.followPath(paths.Path10);
                intakeTimer.reset();
                intakeHalfLineDone = false;
                pathState = 19;
                break;

            case 19:
                // Stop intake halfway through the curve
                if (!intakeHalfLineDone && intakeTimer.milliseconds() > 400) {  // Reduced from 500ms
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    startShooting();
                    pathState = 20;
                }
                break;

            case 20:
                if (updateShooter()) {
                    pathState = 21;
                }
                break;

            case 21:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                break;
        }
    }
}