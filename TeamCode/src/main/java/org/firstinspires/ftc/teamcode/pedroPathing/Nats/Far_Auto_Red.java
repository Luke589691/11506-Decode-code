package org.firstinspires.ftc.teamcode.pedroPathing.Nats;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Other.Constants;

@Autonomous(name = "Far Red", group = "Autonomous")
@Configurable
public class Far_Auto_Red extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    // Hardware
    public DcMotorEx intakeWheels = null;
    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;

    // Timers for non-blocking actions
    private ElapsedTime shooterTimer = new ElapsedTime();
    private int shooterPulseCount = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterPulsing = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Mirrored starting pose: Y = 144 - 8 = 136, Heading = -90°
        follower.setStartingPose(new Pose(72, 136, Math.toRadians(-90)));

        paths = new Paths(follower);

        initializeIntakeShooter();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // CRITICAL: Must be called every loop
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
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
            double elapsed = shooterTimer.milliseconds();

            // Wait for flywheels to spin up
            if (elapsed >= 5000) {
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

            // Pulse pattern: 400ms on, 1800ms off
            if (elapsed < 700) {
                // Intake feeding
                intakeWheels.setPower(-0.8);
            } else if (elapsed < 2200) {  // 400 + 1800
                // Waiting between shots
                intakeWheels.setPower(0);
            } else {
                // Pulse complete, start next one
                shooterPulseCount++;
                shooterTimer.reset();

                if (shooterPulseCount >= 5) {
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
        shooterLeft.setPower(0.725);
        shooterRight.setPower(0.725);
        shooterSpinningUp = true;
        shooterPulsing = false;
        shooterTimer.reset();
    }

    public static class Paths {
        public PathChain Path6;
        public PathChain Path2;

        public Paths(Follower follower) {
            // All Y coordinates mirrored: Y_red = 144 - Y_blue
            // All headings mirrored: H_red = -H_blue

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54.665, 136.343),  // 144 - 7.657
                                    new Pose(62.960, 129.962)   // 144 - 14.038
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-110))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(62.960, 129.962),  // 144 - 14.038
                                    new Pose(62.960, 109.968)   // 144 - 34.032
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-90))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start Path6
                telemetry.addData("State", "Starting Path6");
                follower.followPath(paths.Path6);
                pathState = 1;
                break;

            case 1:
                // Wait for Path6
                if (!follower.isBusy()) {
                    telemetry.addData("State", "Path6 complete, starting shooter");
                    startShooting();
                    pathState = 2;
                }
                break;

            case 2:
                // Shooting after Path6
                telemetry.addData("State", "Shooting");
                if (updateShooter()) {
                    telemetry.addData("State", "Shooting complete, starting Path2");
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
                // Complete
                telemetry.addData("State", "Autonomous DONE");
                break;
        }
    }
}