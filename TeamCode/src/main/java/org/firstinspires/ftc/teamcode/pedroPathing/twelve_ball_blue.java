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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Twelve Ball Blue", group = "Autonomous")
@Configurable // Panels
public class twelve_ball_blue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // ===== INTAKE AND SHOOTER MOTORS =====
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        // Starting pose matches Path1's starting point
        follower.setStartingPose(new Pose(24.464, 129.411, Math.toRadians(144)));
        paths = new Paths(follower); // Build paths

        // ===== INITIALIZE INTAKE AND SHOOTER HARDWARE =====
        initializeIntakeShooter();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        // Also send to telemetry for Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    // ===== HARDWARE INITIALIZATION =====
    /**
     * Initialize intake and shooter motors with proper configuration
     */
    private void initializeIntakeShooter() {
        // Get motors from hardware map
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");

        // Set motor directions
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure shooter motors
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ===== INTAKE FUNCTION =====
    /**
     * Runs the intake wheels to collect artifacts
     * Spins for 3 seconds then stops
     */
    public void intakeArtifacts() {
        intakeWheels.setPower(-1);  // Full power intake
        sleep(5000);                // Spin for 3 seconds
        intakeWheels.setPower(0);   // Stop intake
    }

    // ===== SHOOTER FUNCTION =====
    /**
     * Shoots artifacts into the goal
     * Spins up flywheels, then pulses intake to feed artifacts
     */
    public void shootArtifacts() {
        // Spin up shooter flywheels
        shooterLeft.setPower(0.57);
        shooterRight.setPower(0.57);
        sleep(5000);  // Give flywheels time to reach speed

        // Pulse intake 4 times to feed artifacts into shooter
        for (int i = 0; i < 4; i++) {
            // Spin intake to feed artifact
            intakeWheels.setPower(-1.0);
            sleep(400);  // Feed for 0.4 seconds

            // Stop intake briefly
            intakeWheels.setPower(0);
            sleep(1500);  // Wait 1.5 seconds between shots
        }

        // Stop shooters after done
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.464, 129.411), new Pose(56.110, 105.397))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(145))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.110, 105.397), new Pose(56.334, 84.299))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.334, 84.299), new Pose(15.262, 84.075))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.262, 84.075),
                                    new Pose(57.000, 74.000),
                                    new Pose(56.110, 106.070)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.110, 106.070), new Pose(55.436, 59.611))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.436, 59.611), new Pose(10.100, 60.060))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.100, 60.060),
                                    new Pose(69.000, 51.000),
                                    new Pose(55.885, 106.070)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.885, 106.070), new Pose(55.436, 36.269))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.436, 36.269), new Pose(10.773, 35.596))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.773, 35.596),
                                    new Pose(63.000, 78.000),
                                    new Pose(76.000, 36.000),
                                    new Pose(55.885, 105.845)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // State machine for autonomous routine
        switch (pathState) {
            case 0:
                // Follow Path1 to shooting position
                telemetry.addData("State 0", "Following Path1");
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                // Wait for Path1, then shoot
                telemetry.addData("State 1", "Waiting for Path1 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 1", "Path1 complete - Shooting");
                    shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                // Follow Path2
                telemetry.addData("State 2", "Following Path2");
                follower.followPath(paths.Path2);
                setPathState(3);
                break;
            case 3:
                // Wait for Path2 to complete
                telemetry.addData("State 3", "Waiting for Path2 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 3", "Path2 complete");
                    setPathState(4);
                }
                break;
            case 4:
                // Follow Path3 with full intake at 1/4 speed
                telemetry.addData("State 4", "Following Path3 - Intake ON (1/4 Speed)");
                intakeWheels.setPower(-1);  // Full intake
                follower.setMaxPower(0.25);  // Slow down to 1/4 speed
                follower.followPath(paths.Path3, false);
                setPathState(5);
                break;
            case 5:
                // Wait for Path3, then stop intake and restore speed
                telemetry.addData("State 5", "Waiting for Path3 to complete");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);  // Stop intake
                    follower.setMaxPower(1.0);  // Restore full speed
                    telemetry.addData("State 5", "Path3 complete");
                    setPathState(6);
                }
                break;
            case 6:
                // Follow Path4
                telemetry.addData("State 6", "Following Path4");
                follower.followPath(paths.Path4);
                setPathState(7);
                break;
            case 7:
                // Wait for Path4, then shoot
                telemetry.addData("State 7", "Waiting for Path4 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 7", "Path4 complete - Shooting");
                    shootArtifacts();
                    setPathState(8);
                }
                break;
            case 8:
                // Follow Path5
                telemetry.addData("State 8", "Following Path5");
                follower.followPath(paths.Path5);
                setPathState(9);
                break;
            case 9:
                // Wait for Path5 to complete
                telemetry.addData("State 9", "Waiting for Path5 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 9", "Path5 complete");
                    setPathState(10);
                }
                break;
            case 10:
                // Follow Path6 with full intake at 1/4 speed
                telemetry.addData("State 10", "Following Path6 - Intake ON (1/4 Speed)");
                intakeWheels.setPower(-1);  // Full intake
                follower.setMaxPower(0.25);  // Slow down to 1/4 speed
                follower.followPath(paths.Path6, false);
                setPathState(11);
                break;
            case 11:
                // Wait for Path6, then stop intake and restore speed
                telemetry.addData("State 11", "Waiting for Path6 to complete");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);  // Stop intake
                    follower.setMaxPower(1.0);  // Restore full speed
                    telemetry.addData("State 11", "Path6 complete");
                    setPathState(12);
                }
                break;
            case 12:
                // Follow Path7
                telemetry.addData("State 12", "Following Path7");
                follower.followPath(paths.Path7);
                setPathState(13);
                break;
            case 13:
                // Wait for Path7, then shoot
                telemetry.addData("State 13", "Waiting for Path7 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 13", "Path7 complete - Shooting");
                    shootArtifacts();
                    setPathState(14);
                }
                break;
            case 14:
                // Follow Path8
                telemetry.addData("State 14", "Following Path8");
                follower.followPath(paths.Path8);
                setPathState(15);
                break;
            case 15:
                // Wait for Path8 to complete
                telemetry.addData("State 15", "Waiting for Path8 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 15", "Path8 complete");
                    setPathState(16);
                }
                break;
            case 16:
                // Follow Path9 with full intake at 1/4 speed
                telemetry.addData("State 16", "Following Path9 - Intake ON (1/4 Speed)");
                intakeWheels.setPower(-1);  // Full intake
                follower.setMaxPower(0.25);  // Slow down to 1/4 speed
                follower.followPath(paths.Path9, false);
                setPathState(17);
                break;
            case 17:
                // Wait for Path9, then stop intake and restore speed
                telemetry.addData("State 17", "Waiting for Path9 to complete");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);  // Stop intake
                    follower.setMaxPower(1.0);  // Restore full speed
                    telemetry.addData("State 17", "Path9 complete");
                    setPathState(18);
                }
                break;
            case 18:
                // Follow Path10
                telemetry.addData("State 18", "Following Path10");
                follower.followPath(paths.Path10);
                setPathState(19);
                break;
            case 19:
                // Wait for Path10, then shoot
                telemetry.addData("State 19", "Waiting for Path10 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 19", "Path10 complete - Final Shooting");
                    shootArtifacts();
                    setPathState(20);
                }
                break;
            case 20:
                telemetry.addData("State 20", "Autonomous Complete");
                break;

            default:
                // Autonomous complete
                telemetry.addData("Default", "Unknown state");
                break;
        }
        return pathState;
    }

    /**
     * Setter method for pathState to ensure proper state transitions
     */
    private void setPathState(int newState) {
        pathState = newState;
    }

    /**
     * Helper method for sleep in OpMode (since OpMode doesn't have sleep built-in)
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

/*
 * USAGE NOTES:
 *
 * TEN BALL AUTONOMOUS ROUTINE:
 * This autonomous follows 10 paths and scores 4 times (after Path 1, 4, 7, and 10)
 *
 * SHOOTING SCHEDULE:
 * - After Path1: First scoring
 * - After Path4: Second scoring
 * - After Path7: Third scoring
 * - After Path10: Final scoring
 *
 * INTAKE SCHEDULE:
 * - Path3: Full intake to collect game pieces (DRIVEBASE AT 1/4 SPEED)
 * - Path6: Full intake to collect game pieces (DRIVEBASE AT 1/4 SPEED)
 * - Path9: Full intake to collect game pieces (DRIVEBASE AT 1/4 SPEED)
 * - All other paths: Intake OFF, drivebase at full speed
 *
 * SPEED CONTROL:
 * - During intake paths (3, 6, 9): Robot moves at 25% speed for controlled collection
 * - All other paths: Robot moves at 100% speed
 * - Speed automatically restored after each intake path completes
 *
 * PATTERN:
 * 1. Path1 (full speed) → Shoot
 * 2. Path2 (full speed, transition)
 * 3. Path3 (1/4 speed, INTAKE ON - collect pieces)
 * 4. Path4 (full speed) → Shoot
 * 5. Path5 (full speed, transition)
 * 6. Path6 (1/4 speed, INTAKE ON - collect pieces)
 * 7. Path7 (full speed) → Shoot
 * 8. Path8 (full speed, transition)
 * 9. Path9 (1/4 speed, INTAKE ON - collect pieces)
 * 10. Path10 (full speed) → Shoot
 *
 * SHOOTER:
 * - 0.57 power on flywheels (adjust if needed)
 * - 4 artifacts per shooting cycle
 * - Adjust timing in shootArtifacts() method as needed
 *
 * STATE MACHINE FLOW:
 * States 0-1:   Path1 → Shoot
 * States 2-3:   Path2 (transition)
 * States 4-5:   Path3 (intake at 1/4 speed)
 * States 6-7:   Path4 → Shoot
 * States 8-9:   Path5 (transition)
 * States 10-11: Path6 (intake at 1/4 speed)
 * States 12-13: Path7 → Shoot
 * States 14-15: Path8 (transition)
 * States 16-17: Path9 (intake at 1/4 speed)
 * States 18-19: Path10 → Shoot
 * State 20:     Complete
 */