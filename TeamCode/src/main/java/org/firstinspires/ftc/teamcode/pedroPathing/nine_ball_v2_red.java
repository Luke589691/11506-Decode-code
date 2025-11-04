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

    // Non-blocking timers
    private ElapsedTime actionTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Mirrored starting pose: X = 144 - 26.029, heading = 180 - 145 = 35°
        follower.setStartingPose(new Pose(117.971, 131.050, Math.toRadians(35)));

        paths = new Paths(follower);

        // Initialize hardware
        initializeIntakeShooter();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // CRITICAL: Must be called every loop
        autonomousPathUpdate();

        // Telemetry
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

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * NON-BLOCKING shooter state machine
     * Returns true when shooting is complete
     */
    private boolean updateShooter() {
        if (shooterSpinningUp) {
            if (shooterTimer.milliseconds() >= 4000) {
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

            if (elapsed < 700) {
                intakeWheels.setPower(-1.0);
            } else if (elapsed < 2200) {
                intakeWheels.setPower(0);
            } else {
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 4) {
                    shooterPulsing = false;
                    intakeWheels.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    return true;
                } else {
                    intakeWheels.setPower(-1.0);
                }
            }
            return false;
        }

        return true;
    }

    /**
     * Start shooting sequence (non-blocking)
     */
    private void startShooting() {
        shooterLeft.setPower(-0.60);
        shooterRight.setPower(-0.60);
        shooterSpinningUp = true;
        shooterPulsing = false;
        shooterTimer.reset();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            // Path1: Mirror coordinates
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(117.971, 131.050),  // 144 - 26.029
                            new Pose(71.353, 87.281)     // 144 - 72.647
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(35))  // 180-145
                    .build();

            // Path2: Mirror BezierCurve control points and headings
            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.353, 87.281),    // 144 - 72.647
                            new Pose(79.000, 93.000),    // 144 - 65.000
                            new Pose(99.712, 84.691)     // 144 - 44.288
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();

            // Path3: Mirror X, constant heading at 0°
            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(99.712, 84.691),    // 144 - 44.288
                            new Pose(125.353, 84.043)    // 144 - 18.647
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))  // 180-180
                    .build();

            // Path4: Mirror coordinates
            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.353, 84.043),   // 144 - 18.647
                            new Pose(71.612, 87.410)     // 144 - 72.388
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))   // 180-180, 180-145
                    .build();

            // Path5: Mirror BezierCurve
            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.612, 87.410),    // 144 - 72.388
                            new Pose(53.000, 53.000),    // 144 - 91.000
                            new Pose(100.101, 60.086)    // 144 - 43.899
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();

            // Path6: Mirror X
            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(100.101, 60.086),   // 144 - 43.899
                            new Pose(125.741, 60.216)    // 144 - 18.259
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))  // 180-180
                    .build();

            // Path7: Mirror BezierCurve
            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(125.741, 60.216),   // 144 - 18.259
                            new Pose(70.000, 53.000),    // 144 - 74.000
                            new Pose(71.741, 87.410)     // 144 - 72.259
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))   // 180-180, 180-145
                    .build();


            // Path11: Mirror BezierCurve
            Path11 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.871, 87.410),    // 144 - 72.129
                            new Pose(57.000, 63.000),    // 144 - 87.000
                            new Pose(105.669, 33.410)    // 144 - 38.331
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ===== PATH 1 → SHOOT =====
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

            // ===== PATH 2 → PATH 3 (INTAKE) =====
            case 3:
                telemetry.addData("State", "Starting Path2");
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path2 complete, starting Path3 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.Path3, false);
                    pathState = 5;
                }
                break;

            case 5:
                telemetry.addData("State", "Path3 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    pathState = 6;
                }
                break;

            // ===== PATH 4 (INTAKE HALF) → SHOOT =====
            case 6:
                telemetry.addData("State", "Starting Path4 with half intake");
                intakeWheels.setPower(-0.5);
                follower.followPath(paths.Path4);
                pathState = 7;
                break;

            case 7:
                telemetry.addData("State", "Path4 - Intake Half");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
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

            // ===== PATH 5 → PATH 6 (INTAKE) =====
            case 9:
                telemetry.addData("State", "Starting Path5");
                follower.followPath(paths.Path5);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path5 complete, starting Path6 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.Path6, false);
                    pathState = 11;
                }
                break;

            case 11:
                telemetry.addData("State", "Path6 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    pathState = 12;
                }
                break;

            // ===== PATH 7 (INTAKE HALF) → SHOOT =====
            case 12:
                telemetry.addData("State", "Starting Path7 with half intake");
                intakeWheels.setPower(-0.5);
                follower.followPath(paths.Path7);
                pathState = 13;
                break;

            case 13:
                telemetry.addData("State", "Path7 - Intake Half");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
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

            // ===== PATH 8 → PATH 9 (INTAKE) =====
            case 15:
                telemetry.addData("State", "Starting Path8");
                follower.followPath(paths.Path8);
                pathState = 16;
                break;

            case 16:
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path8 complete, starting Path9 with intake");
                    intakeWheels.setPower(-1.0);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.Path9, false);
                    pathState = 17;
                }
                break;

            case 17:
                telemetry.addData("State", "Path9 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    pathState = 18;
                }
                break;

            // ===== PATH 10 (INTAKE HALF) → SHOOT =====
            case 18:
                telemetry.addData("State", "Starting Path10 with half intake");
                intakeWheels.setPower(-0.5);
                follower.followPath(paths.Path10);
                pathState = 19;
                break;

            case 19:
                telemetry.addData("State", "Path10 - Intake Half");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    telemetry.addData("State", "Path10 complete, shooting");
                    startShooting();
                    pathState = 20;
                }
                break;

            case 20:
                telemetry.addData("State", "Shooting (4th)");
                if (updateShooter()) {
                    pathState = 21;
                }
                break;

            // ===== PATH 11 (OPTIONAL FINAL PATH) =====
            case 21:
                telemetry.addData("State", "Starting Path11 (final)");
                follower.followPath(paths.Path11);
                pathState = 22;
                break;

            case 22:
                telemetry.addData("State", "Path11 - Moving to final position");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path11 complete");
                    pathState = 23;
                }
                break;

            case 23:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                break;

            default:
                telemetry.addData("State", "Unknown state!");
                break;
        }
    }
}

/*
 * NINE BALL AUTONOMOUS SEQUENCE (RED SIDE)
 *
 * Mirrored from blue side using formula: new_x = 144 - old_x
 * Headings converted: new_heading = 180° - old_heading
 *
 * The robot follows this pattern:
 * 1. Path1 → Shoot (preloaded samples)
 * 2. Path2+Path3 → Collect samples (intake full speed)
 * 3. Path4 → Return and shoot (intake half speed for alignment)
 * 4. Path5+Path6 → Collect more samples
 * 5. Path7 → Return and shoot
 * 6. Path8+Path9 → Collect final samples
 * 7. Path10 → Return and shoot
 * 8. Path11 → Move to parking/final position
 *
 * INTAKE STRATEGY:
 * - Full speed (-1.0) during collection paths (Path3, Path6, Path9)
 * - Half speed (-0.5) during return paths (Path4, Path7, Path10) for better control
 * - Stopped during shooting sequences
 */