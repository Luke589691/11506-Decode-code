package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Six Ball Blue V2", group = "Autonomous")
@Configurable
public class SixBall_v2_blue extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public Servo stop = null;

    // Timers for non-blocking actions
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterPulsing = false;
    private boolean isFirstShot = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.654, 124.262, Math.toRadians(145)));
        paths = new Paths(follower);

        initializeIntakeShooter();

        // Start shooter immediately
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

        // First shot only: wait for spin up
        if (isFirstShot && elapsed < 5000) {
            // For subsequent shots: reverse intake for first 100ms
            if (!isFirstShot && elapsed < 100) {
                intakeWheels.setPower(1.0);
            } else if (!isFirstShot && elapsed >= 100) {
                intakeWheels.setPower(0);
            }
            return false;
        }

        // After spin-up (or immediately for subsequent shots), start pulsing
        if (!shooterPulsing) {
            shooterPulsing = true;
            shooterPulseCount = 0;
            shooterTimer.reset();
            intakeWheels.setPower(-1.0);
            stop.setPosition(0); // Open stopper
        }

        if (shooterPulsing) {
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

        if (isFirstShot) {
            isFirstShot = false;
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.654, 124.262),
                            new Pose(55.626, 93.757)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(110))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.626, 93.757),
                            new Pose(55.850, 78.215)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.850, 78.215),
                            new Pose(25.477, 79.112)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(25.477, 79.112),
                            new Pose(56.075, 93.533)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
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
                    telemetry.addData("State", "Path1 complete, starting shooter");
                    startShooting();
                    pathState = 2;
                }
                break;

            case 2:
                telemetry.addData("State", "Shooting");
                if (updateShooter()) {
                    telemetry.addData("State", "Shooting complete");
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
                    telemetry.addData("State", "Path2 complete");
                    pathState = 5;
                }
                break;

            case 5:
                telemetry.addData("State", "Starting Path3 - Intake FULL");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.50);
                follower.followPath(paths.Path3, false);
                pathState = 6;
                break;

            case 6:
                telemetry.addData("State", "Following Path3 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    telemetry.addData("State", "Path3 complete - Intake OFF");
                    pathState = 7;
                }
                break;

            case 7:
                telemetry.addData("State", "Starting Path4 - Intake Full");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.80);
                follower.followPath(paths.Path4);
                pathState = 8;
                break;

            case 8:
                telemetry.addData("State", "Following Path4 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1);
                    telemetry.addData("State", "Path4 complete, starting shooter");
                    startShooting();
                    pathState = 9;
                }
                break;

            case 9:
                telemetry.addData("State", "Final shooting");
                if (updateShooter()) {
                    telemetry.addData("State", "Autonomous complete!");
                    pathState = 10;
                }
                break;

            case 10:
                telemetry.addData("State", "DONE");
                break;
        }
    }
}