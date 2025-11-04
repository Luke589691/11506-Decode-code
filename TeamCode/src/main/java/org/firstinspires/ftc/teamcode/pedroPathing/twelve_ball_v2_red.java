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

    // Non-blocking timers for shooter
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        // Mirrored starting pose: X = 144 - 24.464, heading = 180 - 144 = 36°
        follower.setStartingPose(new Pose(119.536, 129.411, Math.toRadians(36)));
        paths = new Paths(follower);

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
            if (shooterTimer.milliseconds() >= 5000) {
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
                // Feed artifact
                intakeWheels.setPower(-1.0);
            } else if (elapsed < 2200) {
                // Wait between shots
                intakeWheels.setPower(0);
            } else {
                // Move to next pulse
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 4) {
                    // Shooting complete
                    shooterPulsing = false;
                    intakeWheels.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    return true;
                } else {
                    // Start next pulse
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
        shooterLeft.setPower(0.57);
        shooterRight.setPower(0.57);
        shooterSpinningUp = true;
        shooterPulsing = false;
        shooterTimer.reset();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {
            // Path1: Mirror X coordinates and headings
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.536, 129.411),  // 144 - 24.464
                            new Pose(87.890, 105.397)    // 144 - 56.110
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(35))  // 180-144, 180-145
                    .build();

            // Path2: Mirror X
            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 105.397),   // 144 - 56.110
                            new Pose(87.666, 84.299)     // 144 - 56.334
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();

            // Path3: Mirror X
            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.666, 84.299),    // 144 - 56.334
                            new Pose(128.738, 84.075)    // 144 - 15.262
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))    // 180-180
                    .build();

            // Path4: Mirror BezierCurve control points
            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(128.738, 84.075),   // 144 - 15.262
                            new Pose(87.000, 74.000),    // 144 - 57.000
                            new Pose(87.890, 106.070)    // 144 - 56.110
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))   // 180-180, 180-145
                    .build();

            // Path5: Mirror X
            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.890, 106.070),   // 144 - 56.110
                            new Pose(88.564, 59.611)     // 144 - 55.436
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();

            // Path6: Mirror X
            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564, 59.611),    // 144 - 55.436
                            new Pose(133.900, 60.060)    // 144 - 10.100
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))    // 180-180
                    .build();

            // Path7: Mirror BezierCurve
            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.900, 60.060),   // 144 - 10.100
                            new Pose(75.000, 51.000),    // 144 - 69.000
                            new Pose(88.115, 106.070)    // 144 - 55.885
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))   // 180-180, 180-145
                    .build();

            // Path8: Mirror X
            Path8 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.115, 106.070),   // 144 - 55.885
                            new Pose(88.564, 36.269)     // 144 - 55.436
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))   // 180-145, 180-180
                    .build();

            // Path9: Mirror X
            Path9 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.564, 36.269),    // 144 - 55.436
                            new Pose(133.227, 35.596)    // 144 - 10.773
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))    // 180-180
                    .build();

            // Path10: Mirror BezierCurve (4-point curve)
            Path10 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(133.227, 35.596),   // 144 - 10.773
                            new Pose(81.000, 78.000),    // 144 - 63.000
                            new Pose(68.000, 36.000),    // 144 - 76.000
                            new Pose(88.115, 105.845)    // 144 - 55.885
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))   // 180-180, 180-145
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
                telemetry.addData("State", "Path1 - Moving to shoot position");
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

            // ===== PATH 2 (TRANSITION) =====
            case 3:
                telemetry.addData("State", "Starting Path2");
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                telemetry.addData("State", "Path2 - Transition");
                if (!follower.isBusy()) {
                    pathState = 5;
                }
                break;

            // ===== PATH 3 (INTAKE AT 1/4 SPEED) =====
            case 5:
                telemetry.addData("State", "Starting Path3 with intake (1/4 speed)");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.25);
                follower.followPath(paths.Path3, false);
                pathState = 6;
                break;

            case 6:
                telemetry.addData("State", "Path3 - Intake ON (1/4 speed)");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 7;
                }
                break;

            // ===== PATH 4 → SHOOT =====
            case 7:
                telemetry.addData("State", "Starting Path4");
                follower.followPath(paths.Path4);
                pathState = 8;
                break;

            case 8:
                telemetry.addData("State", "Path4 - Returning to shoot");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path4 complete, shooting");
                    startShooting();
                    pathState = 9;
                }
                break;

            case 9:
                telemetry.addData("State", "Shooting (2nd)");
                if (updateShooter()) {
                    pathState = 10;
                }
                break;

            // ===== PATH 5 (TRANSITION) =====
            case 10:
                telemetry.addData("State", "Starting Path5");
                follower.followPath(paths.Path5);
                pathState = 11;
                break;

            case 11:
                telemetry.addData("State", "Path5 - Transition");
                if (!follower.isBusy()) {
                    pathState = 12;
                }
                break;

            // ===== PATH 6 (INTAKE AT 1/4 SPEED) =====
            case 12:
                telemetry.addData("State", "Starting Path6 with intake (1/4 speed)");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.25);
                follower.followPath(paths.Path6, false);
                pathState = 13;
                break;

            case 13:
                telemetry.addData("State", "Path6 - Intake ON (1/4 speed)");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 14;
                }
                break;

            // ===== PATH 7 → SHOOT =====
            case 14:
                telemetry.addData("State", "Starting Path7");
                follower.followPath(paths.Path7);
                pathState = 15;
                break;

            case 15:
                telemetry.addData("State", "Path7 - Returning to shoot");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path7 complete, shooting");
                    startShooting();
                    pathState = 16;
                }
                break;

            case 16:
                telemetry.addData("State", "Shooting (3rd)");
                if (updateShooter()) {
                    pathState = 17;
                }
                break;

            // ===== PATH 8 (TRANSITION) =====
            case 17:
                telemetry.addData("State", "Starting Path8");
                follower.followPath(paths.Path8);
                pathState = 18;
                break;

            case 18:
                telemetry.addData("State", "Path8 - Transition");
                if (!follower.isBusy()) {
                    pathState = 19;
                }
                break;

            // ===== PATH 9 (INTAKE AT 1/4 SPEED) =====
            case 19:
                telemetry.addData("State", "Starting Path9 with intake (1/4 speed)");
                intakeWheels.setPower(-1.0);
                follower.setMaxPower(0.25);
                follower.followPath(paths.Path9, false);
                pathState = 20;
                break;

            case 20:
                telemetry.addData("State", "Path9 - Intake ON (1/4 speed)");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    follower.setMaxPower(1.0);
                    pathState = 21;
                }
                break;

            // ===== PATH 10 → FINAL SHOOT =====
            case 21:
                telemetry.addData("State", "Starting Path10");
                follower.followPath(paths.Path10);
                pathState = 22;
                break;

            case 22:
                telemetry.addData("State", "Path10 - Final return to shoot");
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path10 complete, final shooting");
                    startShooting();
                    pathState = 23;
                }
                break;

            case 23:
                telemetry.addData("State", "Shooting (4th - Final)");
                if (updateShooter()) {
                    pathState = 24;
                }
                break;

            case 24:
                telemetry.addData("State", "✓ AUTONOMOUS COMPLETE ✓");
                break;

            default:
                telemetry.addData("State", "Unknown state!");
                break;
        }
    }
}

/*
 * TWELVE BALL AUTONOMOUS (RED SIDE - NON-BLOCKING VERSION)
 *
 * Mirrored from blue side using formula: new_x = 144 - old_x
 * Headings converted: new_heading = 180° - old_heading
 *
 * SEQUENCE:
 * - Path1 → Shoot (preloaded)
 * - Path2 (transition)
 * - Path3 (intake at 1/4 speed)
 * - Path4 → Shoot
 * - Path5 (transition)
 * - Path6 (intake at 1/4 speed)
 * - Path7 → Shoot
 * - Path8 (transition)
 * - Path9 (intake at 1/4 speed)
 * - Path10 → Final Shoot
 *
 * TOTAL: 25 states (0-24), 4 shooting sequences
 */