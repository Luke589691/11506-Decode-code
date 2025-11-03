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

    // Timers for non-blocking actions
    private ElapsedTime actionTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.654, 124.262, Math.toRadians(145)));
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
     * Call this every loop to advance the shooting sequence
     */
    private boolean updateShooter() {
        if (shooterSpinningUp) {
            // Wait for flywheels to spin up
            if (shooterTimer.milliseconds() >= 4000) {
                shooterSpinningUp = false;
                shooterPulsing = true;
                shooterPulseCount = 0;
                shooterTimer.reset();
                intakeWheels.setPower(-1.0); // Start first pulse
            }
            return false; // Still busy
        }

        if (shooterPulsing) {
            double elapsed = shooterTimer.milliseconds();

            // Pulse pattern: 700ms on, 1500ms off
            if (elapsed < 700) {
                // Intake feeding
                intakeWheels.setPower(-1.0);
            } else if (elapsed < 2200) {  // 700 + 1500
                // Waiting between shots
                intakeWheels.setPower(0);
            } else {
                // Pulse complete, start next one
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 4) {
                    // All pulses complete
                    shooterPulsing = false;
                    intakeWheels.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    return true; // Shooting complete
                } else {
                    intakeWheels.setPower(-1.0); // Start next pulse
                }
            }
            return false; // Still busy
        }

        return true; // Not shooting
    }

    /**
     * Start the shooting sequence (non-blocking)
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.654, 124.262),
                            new Pose(55.626, 93.757)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.626, 93.757),
                            new Pose(55.850, 83.215)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.850, 83.215),
                            new Pose(30.477, 84.112)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(30.477, 84.112),
                            new Pose(56.075, 93.533)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start Path1
                telemetry.addData("State", "Starting Path1");
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                // Wait for Path1
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path1 complete, starting shooter");
                    startShooting();
                    pathState = 2;
                }
                break;

            case 2:
                // Shooting after Path1
                telemetry.addData("State", "Shooting");
                if (updateShooter()) {
                    telemetry.addData("State", "Shooting complete");
                    pathState = 3;
                }
                break;

            case 3:
                // Start Path2
                telemetry.addData("State", "Starting Path2");
                follower.followPath(paths.Path2);
                pathState = 4;
                break;

            case 4:
                // Wait for Path2
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path2 complete");
                    pathState = 5;
                }
                break;

            case 5:
                // Start Path3 with full intake
                telemetry.addData("State", "Starting Path3 - Intake FULL");
                intakeWheels.setPower(-1.0);
                follower.followPath(paths.Path3, false);
                pathState = 6;
                break;

            case 6:
                // Wait for Path3
                telemetry.addData("State", "Following Path3 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    telemetry.addData("State", "Path3 complete - Intake OFF");
                    pathState = 7;
                }
                break;

            case 7:
                // Start Path4 with half intake
                telemetry.addData("State", "Starting Path4 - Intake Half");
                intakeWheels.setPower(-0.5);
                follower.followPath(paths.Path4);
                pathState = 8;
                break;

            case 8:
                // Wait for Path4
                telemetry.addData("State", "Following Path4 - Intake ON");
                if (!follower.isBusy()) {
                    intakeWheels.setPower(0);
                    telemetry.addData("State", "Path4 complete, starting shooter");
                    startShooting();
                    pathState = 9;
                }
                break;

            case 9:
                // Final shooting
                telemetry.addData("State", "Final shooting");
                if (updateShooter()) {
                    telemetry.addData("State", "Autonomous complete!");
                    pathState = 10;
                }
                break;

            case 10:
                // Complete
                telemetry.addData("State", "DONE");
                break;
        }
    }
}