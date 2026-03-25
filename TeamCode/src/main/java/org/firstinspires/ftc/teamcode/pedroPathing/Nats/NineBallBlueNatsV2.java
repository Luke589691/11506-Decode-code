package org.firstinspires.ftc.teamcode.pedroPathing.Nats;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Other.Constants;

@Autonomous(name = "Nine Ball Nats Blue Dual Shooter", group = "Autonomous")
@Configurable
public class NineBallBlueNatsV2 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware — dual shooter from TacoNats
    public DcMotorEx intakeWheels  = null;
    public DcMotorEx shooterLeft   = null;
    public DcMotorEx shooterRight  = null;
    public Servo stop              = null;

    // Shooter power (mirrors TacoNats DEFAULT_SHOOTER_POWER — adjust to taste)
    private static final double SHOOTER_POWER = 0.56;

    // Non-blocking timers for shooter
    private ElapsedTime shooterTimer    = new ElapsedTime();
    private ElapsedTime servoOpenTimer  = new ElapsedTime();
    private boolean shooterSpinningUp   = false;
    private boolean shooterPulsing      = false;
    private boolean waitingForServo     = false;
    private boolean isFirstShot         = true;
    private boolean isLastCycle         = false;  // NEW: flag for last shooting cycle

    // Intake half-line timing
    private ElapsedTime intakeTimer       = new ElapsedTime();
    private boolean intakeHalfLineDone    = false;

    // Configurable shooting heading (degrees)
    private double shootingHeadingDegrees = 335.0;

    // -----------------------------------------------------------------------
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.536, 14.589, Math.toRadians(324)));
        paths = new Paths(follower, shootingHeadingDegrees);

        initializeHardware();

        // Pre-spin both shooters during init (same as NineBall original)
        shooterLeft.setPower(SHOOTER_POWER);
        shooterRight.setPower(SHOOTER_POWER);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // -----------------------------------------------------------------------
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State",    pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X",       follower.getPose().getX());
        panelsTelemetry.debug("Y",       follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);

        telemetry.addData("Path State",    pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.0f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // -----------------------------------------------------------------------
    private void initializeHardware() {
        // Motor names — update these strings to match your hardware config
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        stop         = hardwareMap.get(Servo.class,     "stop");

        // Directions from TacoNats
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        intakeWheels.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo starts closed
        stop.setPosition(0.5);
    }

    // -----------------------------------------------------------------------
    /** Sets power on both shooter motors identically. */
    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    // -----------------------------------------------------------------------
    /**
     * Non-blocking shooter state machine.
     * Returns true once the full shooting sequence is complete.
     */
    private boolean updateShooter() {
        // Wait after servo opens — longer delay on the last cycle
        if (waitingForServo) {
            double servoDelay = isLastCycle ? 2500 : 1000;  // 2.5s last cycle, 1s otherwise
            if (servoOpenTimer.milliseconds() < servoDelay) return false;
            waitingForServo = false;

            // After delay: if not spinning up (i.e. not first shot), begin pulsing now
            if (!shooterSpinningUp) {
                shooterPulsing = true;
                shooterTimer.reset();
            }
        }

        double elapsed = shooterTimer.milliseconds();

        if (shooterSpinningUp) {
            if (elapsed >= 2000) {
                shooterSpinningUp = false;
                shooterPulsing    = true;
                shooterTimer.reset();
            }
            return false;
        }

        if (shooterPulsing) {
            // 4 shots: 800 ms intake ON + 400 ms wait = 1200 ms per cycle
            int cycle     = (int)(elapsed / 1200);
            int phaseTime = (int)(elapsed % 1200);

            if (cycle < 4) {
                intakeWheels.setPower(phaseTime < 800 ? -0.8 : 0);
            } else {
                shooterPulsing = false;
                intakeWheels.setPower(0);
                stop.setPosition(0.5); // Close servo after shooting
                return true;
            }
            return false;
        }

        return true; // Already idle
    }

    // -----------------------------------------------------------------------
    private void startShooting() {
        stop.setPosition(0.9); // Open servo
        servoOpenTimer.reset();
        waitingForServo = true; // Always pause after servo opens

        if (isFirstShot) {
            // First shot needs spin-up time (runs in parallel with servo delay)
            shooterSpinningUp = true;
            shooterPulsing    = false;
            isFirstShot       = false;
        } else {
            // Subsequent shots: pulsing starts after the servo delay
            shooterSpinningUp = false;
            shooterPulsing    = false;
        }

        shooterTimer.reset();
    }

    // -----------------------------------------------------------------------
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5,
                Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower, double shootingHeadingDegrees) {
            double shootingHeadingRad = Math.toRadians(shootingHeadingDegrees);

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.536, 14.589),
                            new Pose(87.890,  38.603)))
                    .setLinearHeadingInterpolation(Math.toRadians(324), shootingHeadingRad)
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 38.603),
                            new Pose(87.666, 59.701)))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.666,  59.701),
                            new Pose(128.738, 59.925)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(128.738, 59.925),
                            new Pose(87.000,  70.000),
                            new Pose(87.890,  37.930)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootingHeadingRad)
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 37.930),
                            new Pose(88.564, 84.389)))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564,  84.389),
                            new Pose(133.900, 83.940)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.900, 83.940),
                            new Pose(75.000,  93.000),
                            new Pose(88.115,  37.930)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootingHeadingRad)
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.115, 37.930),
                            new Pose(88.564, 107.731)))
                    .setLinearHeadingInterpolation(shootingHeadingRad, Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564,  107.731),
                            new Pose(133.227, 108.404)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.227, 108.404),
                            new Pose(81.000,  66.000),
                            new Pose(68.000,  108.000),
                            new Pose(88.115,  38.155)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(325))
                    .build();
        }
    }

    // -----------------------------------------------------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // --- Drive to first shooting position ---
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

            // --- Row 1 intake sweep ---
            case 3:
                follower.followPath(paths.Path2);
                intakeWheels.setPower(-1.0);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path3, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 5;
                }
                break;

            case 5:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 6;
                }
                break;

            // --- Return to shoot row 1 balls ---
            case 6:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path4);
                intakeWheels.setPower(-1.0);
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
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

            // --- Row 2 intake sweep ---
            case 9:
                follower.followPath(paths.Path5);
                intakeWheels.setPower(-1.0);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path6, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 11;
                }
                break;

            case 11:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 12;
                }
                break;

            // --- Return to shoot row 2 balls ---
            case 12:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path7);
                intakeWheels.setPower(-1.0);
                pathState = 13;
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
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

            // --- Row 3 intake sweep ---
            case 15:
                follower.followPath(paths.Path8);
                intakeWheels.setPower(-1.0);
                pathState = 16;
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path9, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 17;
                }
                break;

            case 17:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 18;
                }
                break;

            // --- Return to shoot row 3 balls (last cycle) ---
            case 18:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path10);
                intakeWheels.setPower(-1.0);
                pathState = 19;
                break;

            case 19:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    isLastCycle = true;  // Extended servo delay for final shot
                    startShooting();
                    pathState = 20;
                }
                break;

            case 20:
                if (updateShooter()) {
                    // All done — spin down shooters
                    setShooterPower(0);
                    pathState = 21;
                }
                break;

            case 21:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                break;
        }
    }
}