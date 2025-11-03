package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Six ball blue", group = "Autonomous")
@Configurable // Panels
public class SixBall_blue extends OpMode {
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
        // FIXED: Starting pose now matches Path1's starting point
        follower.setStartingPose(new Pose(22.654, 124.262, Math.toRadians(145)));
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
        shooterLeft.setPower(-0.60);
        shooterRight.setPower(-0.60);
        sleep(4000);  // Give flywheels time to reach speed

        // Pulse intake 6 times to feed artifacts into shooter
        for (int i = 0; i < 4; i++) {
            // Spin intake to feed artifact
            intakeWheels.setPower(-1.0);
            sleep(700);  // Feed for 0.2 seconds

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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.654, 124.262), new Pose(55.626, 93.757))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.626, 93.757), new Pose(55.850, 83.215))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.850, 83.215), new Pose(30.477, 84.112))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.477, 84.112), new Pose(56.075, 93.533))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // State machine for autonomous routine
        switch (pathState) {
            case 0:
                // Follow first path
                telemetry.addData("State 0", "Following Path1");
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                // Wait for path to complete
                telemetry.addData("State 1", "Waiting for Path1 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 1", "Path1 complete, shooting");
                    // Shoot artifacts
                    shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                // Follow next path
                telemetry.addData("State 2", "Following Path2");
                follower.followPath(paths.Path2);
                setPathState(3);
                break;
            case 3:
                // Wait for path to complete
                telemetry.addData("State 3", "Waiting for Path2 to complete");
                if (!follower.isBusy()) {
                    telemetry.addData("State 3", "Path2 complete");
                    setPathState(4);
                }
                break;
            case 4:
                // Follow Path3 (combined Path3+Path4) with intake running at full speed, robot moving dead slow
                telemetry.addData("State 4", "Following Path3 - Intake FULL (Drive Dead Slow)");
                intakeWheels.setPower(-1);  // Full intake speed
                follower.followPath(paths.Path3, /* holdEnd = */ false);
                setPathState(5);
                break;
            case 5:
                // Wait for Path3 to complete, then stop intake
                telemetry.addData("State 5", "Waiting for Path3 to complete - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);  // Stop intake
                    telemetry.addData("State 5", "Path3 complete - Intake OFF");
                    setPathState(6);
                }
                break;
            case 6:
                // Follow Path4, run intake at half speed
                telemetry.addData("State 6", "Following Path4 - Intake Half Speed");
                intakeWheels.setPower(-0.5);  // Half speed intake during Path4
                follower.followPath(paths.Path4);
                setPathState(7);
                break;
            case 7:
                // Wait for Path4 to complete, then stop intake and shoot
                telemetry.addData("State 7", "Waiting for Path4 to complete - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);  // Stop intake
                    telemetry.addData("State 7", "Path4 complete - Shooting");
                    shootArtifacts();  // Shoot after Path4 completes
                    setPathState(8);
                }
                break;
            case 8:
                telemetry.addData("State 8", "Autonomous complete");
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
 * INTAKE:
 * - Call intakeArtifacts() when you want to collect game pieces
 * - Runs at full power for 3 seconds
 * - Adjust timing/power in the function as needed
 *
 * SHOOTER:
 * - Call shootArtifacts() when ready to score
 * - Spins up flywheels, then feeds 4 artifacts
 * - Adjust flywheel power (0.57) if shots are too weak/strong
 * - Adjust loop count (4) for number of artifacts
 *
 * STATE MACHINE:
 * - State 0-1: Follow Path1, then shoot
 * - State 2-3: Follow Path2
 * - State 4-5: Follow Path3 with intake at full speed (-1.0)
 * - State 6-7: Follow Path4 with intake at half speed (-0.5), then shoot
 * - State 8: Complete
 */