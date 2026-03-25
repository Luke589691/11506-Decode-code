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

@Autonomous(name = "12 Ball Nats Blue", group = "Autonomous")
@Configurable

public class TwelveBallBlueNatsV2 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware — dual shooter (TacoNats directions)
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft  = null;
    public DcMotorEx shooterRight = null;
    public Servo stop             = null;

    // Shooter power — tune as needed
    private static final double SHOOTER_POWER = 0.50;

    // Non-blocking shooter state
    private ElapsedTime shooterTimer  = new ElapsedTime();
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing    = false;
    private boolean isFirstShot       = true;

    // Half-line intake timing
    private ElapsedTime intakeTimer    = new ElapsedTime();
    private boolean intakeHalfLineDone = false;

    // Configurable shooting heading (degrees)
    private double shootingHeadingDegrees = 330.0;

    // -----------------------------------------------------------------------
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(119.536, 14.589, Math.toRadians(324)));
        paths = new Paths(follower, shootingHeadingDegrees);

        initializeHardware();

        // Pre-spin both shooters during init
        setShooterPower(SHOOTER_POWER);

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

        stop.setPosition(0.5); // Servo closed at start
    }

    // -----------------------------------------------------------------------
    /** Sets both shooter motors to the same power. */
    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    // -----------------------------------------------------------------------
    /**
     * Non-blocking shooter state machine.
     * Returns true when the full shooting sequence is complete.
     */
    private boolean updateShooter() {
        double elapsed = shooterTimer.milliseconds();

        if (shooterSpinningUp) {
            if (elapsed >= 3000) {
                shooterSpinningUp = false;
                shooterPulsing    = true;
                shooterTimer.reset();
            }
            return false;
        }

        if (shooterPulsing) {
            // 4 shots: 800 ms intake ON + 400 ms wait = 1200 ms per cycle
            int cycle     = (int)(elapsed / 800);
            int phaseTime = (int)(elapsed % 800);

            if (cycle < 4) {
                intakeWheels.setPower(phaseTime < 300 ? -0.8 : 0);
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

        if (isFirstShot) {
            shooterSpinningUp = true;
            shooterPulsing    = false;
            isFirstShot       = false;
        } else {
            // Brief intake reversal to clear any jam, then pulse immediately
            intakeWheels.setPower(1.0);
            try { Thread.sleep(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            intakeWheels.setPower(0);

            shooterSpinningUp = false;
            shooterPulsing    = true;
            intakeWheels.setPower(-1.0);
        }

        shooterTimer.reset();
    }

    // -----------------------------------------------------------------------
    public static class Paths {
        // 12 balls = 4 rows × 3 balls each, so we need 4 sweep+return cycles.
        // That gives us 12 paths total (intake out + sweep + return per row,
        // plus a final return-to-shoot).
        // TODO: Replace all Pose coordinates below with your actual field positions.
        public PathChain Path1,  Path2,  Path3;  // Row 1: drive to shoot → sweep → return
        public PathChain Path4,  Path5,  Path6;  // Row 2: drive to shoot → sweep → return
        public PathChain Path7,  Path8,  Path9;  // Row 3: drive to shoot → sweep → return
        public PathChain Path10, Path11, Path12; // Row 4: drive to shoot → sweep → return

        public Paths(Follower follower, double shootingHeadingDegrees) {
            double shootRad = Math.toRadians(shootingHeadingDegrees);

            // ----------------------------------------------------------------
            // ROW 1
            // ----------------------------------------------------------------
            // Path1 — Starting position → Shooting position
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.536, 14.589),
                            new Pose(87.890,  38.603)))
                    .setLinearHeadingInterpolation(Math.toRadians(324), shootRad)
                    .build();

            // Path2 — Shooting position → far end of Row 1 (intake sweep out)
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 38.603),
                            new Pose(133.900, 59.925)))
                    .setLinearHeadingInterpolation(shootRad, Math.toRadians(0))
                    .build();

            // Path3 — Far end of Row 1 → Shooting position (return)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.900, 59.925),
                            new Pose(87.000,  70.000),
                            new Pose(87.890,  38.603)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootRad)
                    .build();

            // ----------------------------------------------------------------
            // ROW 2
            // ----------------------------------------------------------------
            // Path4 — Shooting position → Row 2 entry (intake sweep out)
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 38.603),
                            new Pose(88.564, 84.389)))
                    .setLinearHeadingInterpolation(shootRad, Math.toRadians(0))
                    .build();

            // Path5 — Row 2 entry → far end of Row 2
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564,  84.389),
                            new Pose(133.900, 83.940)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // Path6 — Far end of Row 2 → Shooting position (return)
            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.900, 83.940),
                            new Pose(75.000,  93.000),
                            new Pose(88.115,  38.603)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootRad)
                    .build();

            // ----------------------------------------------------------------
            // ROW 3
            // ----------------------------------------------------------------
            // Path7 — Shooting position → Row 3 entry (intake sweep out)
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.115, 38.603),
                            new Pose(88.564, 107.731)))
                    .setLinearHeadingInterpolation(shootRad, Math.toRadians(0))
                    .build();

            // Path8 — Row 3 entry → far end of Row 3
            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564,  107.731),
                            new Pose(133.227, 108.404)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // Path9 — Far end of Row 3 → Shooting position (return)
            Path9 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.227, 108.404),
                            new Pose(81.000,  66.000),
                            new Pose(68.000,  108.000),
                            new Pose(88.115,  38.155)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootRad)
                    .build();

            // ----------------------------------------------------------------
            // ROW 4  — TODO: fill in real field coordinates for the 4th row
            // ----------------------------------------------------------------
            // Path10 — Shooting position → Row 4 entry (intake sweep out)
            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.115, 38.155),   // TODO: adjust start
                            new Pose(88.564, 130.000))) // TODO: Row 4 entry Y
                    .setLinearHeadingInterpolation(shootRad, Math.toRadians(0))
                    .build();

            // Path11 — Row 4 entry → far end of Row 4
            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564,  130.000), // TODO: Row 4 entry
                            new Pose(133.000, 130.000))) // TODO: Row 4 far end
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // Path12 — Far end of Row 4 → Final shooting position (return)
            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.000, 130.000), // TODO: Row 4 far end
                            new Pose(81.000,  90.000),  // TODO: curve control point
                            new Pose(88.115,  38.155))) // TODO: final shoot position
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(325))
                    .build();
        }
    }

    // -----------------------------------------------------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ================================================================
            // INITIAL DRIVE TO SHOOT
            // ================================================================
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

            // ================================================================
            // ROW 1 — Sweep out → return to shoot
            // ================================================================
            case 3:
                follower.followPath(paths.Path2);
                intakeWheels.setPower(-1.0);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.followPath(paths.Path3);
                    intakeWheels.setPower(-1.0);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 6;
                }
                break;

            case 6:
                if (updateShooter()) {
                    pathState = 7;
                }
                break;

            // ================================================================
            // ROW 2 — Sweep to entry → sweep far end → return to shoot
            // ================================================================
            case 7:
                follower.followPath(paths.Path4);
                intakeWheels.setPower(-1.0);
                pathState = 8;
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path5, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 9;
                }
                break;

            case 9:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 10;
                }
                break;

            case 10:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path6);
                intakeWheels.setPower(-1.0);
                pathState = 11;
                break;

            case 11:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 12;
                }
                break;

            case 12:
                if (updateShooter()) {
                    pathState = 13;
                }
                break;

            // ================================================================
            // ROW 3 — Sweep to entry → sweep far end → return to shoot
            // ================================================================
            case 13:
                follower.followPath(paths.Path7);
                intakeWheels.setPower(-1.0);
                pathState = 14;
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path8, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 15;
                }
                break;

            case 15:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 16;
                }
                break;

            case 16:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path9);
                intakeWheels.setPower(-1.0);
                pathState = 17;
                break;

            case 17:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 18;
                }
                break;

            case 18:
                if (updateShooter()) {
                    pathState = 19;
                }
                break;

            // ================================================================
            // ROW 4 — Sweep to entry → sweep far end → return to shoot
            // ================================================================
            case 19:
                follower.followPath(paths.Path10);
                intakeWheels.setPower(-1.0);
                pathState = 20;
                break;

            case 20:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.Path11, false);
                    intakeTimer.reset();
                    intakeHalfLineDone = false;
                    pathState = 21;
                }
                break;

            case 21:
                if (!intakeHalfLineDone && !follower.isBusy()) {
                    intakeWheels.setPower(0);
                    intakeHalfLineDone = true;
                }
                if (!follower.isBusy() && intakeHalfLineDone) {
                    follower.setMaxPower(1.0);
                    pathState = 22;
                }
                break;

            case 22:
                follower.setMaxPower(1.0);
                follower.followPath(paths.Path12);
                intakeWheels.setPower(-1.0);
                pathState = 23;
                break;

            case 23:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 24;
                }
                break;

            case 24:
                if (updateShooter()) {
                    // All done — spin down both shooters
                    setShooterPower(0);
                    pathState = 25;
                }
                break;

            // ================================================================
            // COMPLETE
            // ================================================================
            case 25:
                telemetry.addData("State", "✓ 12 BALL AUTONOMOUS COMPLETE ✓");
                break;
        }
    }
}