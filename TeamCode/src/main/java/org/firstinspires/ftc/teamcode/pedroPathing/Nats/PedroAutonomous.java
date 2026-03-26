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
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // -----------------------------------------------------------------------
    // Hardware — mirrors NineBallBlueNatsV2 dual shooter setup
    // Update these hardware map names to match your robot configuration
    // -----------------------------------------------------------------------
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft  = null;
    public DcMotorEx shooterRight = null;
    public Servo stop             = null;

    private static final double SHOOTER_POWER = 0.54;

    // Non-blocking shooter timers
    private ElapsedTime shooterTimer   = new ElapsedTime();
    private ElapsedTime servoOpenTimer = new ElapsedTime();
    private boolean shooterSpinningUp  = false;
    private boolean shooterPulsing     = false;
    private boolean waitingForServo    = false;
    private boolean isFirstShot        = true;
    private boolean isLastCycle        = false;

    // -----------------------------------------------------------------------
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25.346, 129.421, Math.toRadians(145)));

        paths = new Paths(follower);

        initializeHardware();

        // Pre-spin shooters during init
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
    }

    // -----------------------------------------------------------------------
    private void initializeHardware() {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        stop         = hardwareMap.get(Servo.class,     "stop");

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
    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    // -----------------------------------------------------------------------
    /** Opens the servo and kicks off the shooter sequence. */
    private void startShooting() {
        stop.setPosition(0.9); // Open servo
        servoOpenTimer.reset();
        waitingForServo = true;

        if (isFirstShot) {
            shooterSpinningUp = true;
            shooterPulsing    = false;
            isFirstShot       = false;
        } else {
            shooterSpinningUp = false;
            shooterPulsing    = false;
        }
        shooterTimer.reset();
    }

    // -----------------------------------------------------------------------
    /**
     * Non-blocking shooter state machine.
     * Call every loop() tick — returns true once the shooting sequence is done.
     */
    private boolean updateShooter() {
        // Wait after servo opens; last cycle gets an extended delay
        if (waitingForServo) {
            double servoDelay = isLastCycle ? 2500 : 1000;
            if (servoOpenTimer.milliseconds() < servoDelay) return false;
            waitingForServo = false;

            if (!shooterSpinningUp) {
                shooterPulsing = true;
                shooterTimer.reset();
            }
        }

        double elapsed = shooterTimer.milliseconds();

        if (shooterSpinningUp) {
            if (elapsed >= 0) {
                shooterSpinningUp = false;
                shooterPulsing    = true;
                shooterTimer.reset();
            }
            return false;
        }

        if (shooterPulsing) {
            // 2 pulses: 800 ms intake ON + 400 ms wait = 1200 ms per cycle
            int cycle     = (int)(elapsed / 1200);
            int phaseTime = (int)(elapsed % 1200);

            if (cycle < 2) {
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
    // Paths — names preserved exactly as you defined them
    // -----------------------------------------------------------------------
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain intake1;   // intake sweep row 1  (was "intake")
        public PathChain Path4;
        public PathChain Path5;
        public PathChain intake2;   // intake sweep row 2  (was "Intake" #1)
        public PathChain Path7;
        public PathChain Path8;
        public PathChain intake3;   // intake sweep row 3  (was "Intake" #2 — duplicate fixed)
        public PathChain Path10;

        public Paths(Follower follower) {

            // --- Drive to first shooting position ---
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.346, 129.421),
                                    new Pose(58.916, 103.290)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(155))
                    .build();

            // --- Move down toward row 1 ---
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.916, 103.290),
                                    new Pose(58.252, 84.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            // --- Row 1 intake sweep ---
            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.252, 84.776),
                                    new Pose(15.252, 84.561)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // --- Return to shoot row 1 balls ---
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.252, 84.561),
                                    new Pose(59.654, 103.009)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            // --- Move down toward row 2 ---
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.654, 103.009),
                                    new Pose(59.813, 60.963)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            // --- Row 2 intake sweep ---
            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.813, 60.963),
                                    new Pose(15.738, 60.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // --- Return to shoot row 2 balls ---
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.738, 60.178),
                                    new Pose(59.196, 102.879)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            // --- Move down toward row 3 ---
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.196, 102.879),
                                    new Pose(60.206, 36.047)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            // --- Row 3 intake sweep ---
            intake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.206, 36.047),
                                    new Pose(14.822, 35.551)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // --- Return to shoot row 3 balls (final) ---
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.822, 35.551),
                                    new Pose(59.093, 101.794)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();
        }
    }

    // -----------------------------------------------------------------------
    // State machine — mirrors NineBallBlueNatsV2 structure exactly
    //
    // Flow:
    //   0-2   Drive to shoot position → shoot preloaded balls
    //   3-5   Descend to row 1 → intake sweep
    //   6-8   Return → shoot row 1 balls
    //   9-11  Descend to row 2 → intake sweep
    //  12-14  Return → shoot row 2 balls
    //  15-17  Descend to row 3 → intake sweep
    //  18-20  Return → shoot row 3 balls (last cycle — extended delay)
    //  21     Complete
    // -----------------------------------------------------------------------
    public void autonomousPathUpdate() {
        switch (pathState) {

            // === Drive to first shooting position ===
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

            // === Row 1: descend then intake sweep ===
            case 3:
                follower.followPath(paths.Path2);
                intakeWheels.setPower(-1.0);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40); // Slow for intake sweep
                    follower.followPath(paths.intake1, false);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 6;
                }
                break;

            // === Row 1: return and shoot ===
            case 6:
                follower.followPath(paths.Path4);
                intakeWheels.setPower(-1.0); // Keep feeding during return
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 8;
                }
                break;

            case 8:
                if (updateShooter()) {
                    pathState = 9;
                }
                break;

            // === Row 2: descend then intake sweep ===
            case 9:
                follower.followPath(paths.Path5);
                intakeWheels.setPower(-1.0);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.intake2, false);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 12;
                }
                break;

            // === Row 2: return and shoot ===
            case 12:
                follower.followPath(paths.Path7);
                intakeWheels.setPower(-1.0);
                pathState = 13;
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    startShooting();
                    pathState = 14;
                }
                break;

            case 14:
                if (updateShooter()) {
                    pathState = 15;
                }
                break;

            // === Row 3: descend then intake sweep ===
            case 15:
                follower.followPath(paths.Path8);
                intakeWheels.setPower(-1.0);
                pathState = 16;
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.intake3, false);
                    pathState = 17;
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 18;
                }
                break;

            // === Row 3: return and shoot (final cycle) ===
            case 18:
                follower.followPath(paths.Path10);
                intakeWheels.setPower(-1.0);
                pathState = 19;
                break;

            case 19:
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    isLastCycle = true; // Extended 2.5s servo delay for final shot
                    startShooting();
                    pathState = 20;
                }
                break;

            case 20:
                if (updateShooter()) {
                    setShooterPower(0); // Spin down
                    pathState = 21;
                }
                break;

            case 21:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                telemetry.update();
                break;
        }
    }
}