package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Taco Decode")
public class tacoDecode extends LinearOpMode {
    public DcMotorEx frontLeft = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;

    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;
    public Servo Limelightspin = null;
    public Servo Limelighttilt = null;

    double shooterLeftPower = 0.0;
    double shooterRightPower = 0.0;

    public DcMotorEx intakeWheels = null;
    public ColorSensor colorSensor = null;

    double intakeWheelsPower = 0.0;

    boolean intakeRunning = false;
    boolean lastAPress = false;
    boolean lastBPress = false;

    boolean lastYPress = false;
    boolean lastXPress = false;
    int shooterMode = 0;

    boolean humanPlayerMode = false;

    // Variables for non-blocking intake sequence
    private long intakeSequenceStartTime = 0;
    private int intakeSequenceStep = 0;
    private boolean intakeSequenceRunning = false;

    // Variables for ball counting
    private int ballCount = 0;
    private boolean ballDetected = false;
    private int colorThreshold = 100; // Adjust based on your sensor

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelightspin = hardwareMap.get(Servo.class, "Limelightspin");
        Limelighttilt = hardwareMap.get(Servo.class, "Limelighttilt");

        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setPower(shooterRightPower);
        shooterLeft.setPower(shooterLeftPower);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.back) {
                Limelightspin.setPosition(0);
                Limelighttilt.setPosition(0.05);
            }

            if (gamepad1.start) {
                Limelightspin.setPosition(1);
                Limelighttilt.setPosition(0.2);
            }

            double y = -gamepad1.left_stick_y; //foward and back
            double x = gamepad1.left_stick_x; //strafe
            double rx = gamepad1.right_stick_x; //spinny

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.leftBumperWasPressed() && !humanPlayerMode) {
                shooterLeftPower -= 0.05;
                shooterRightPower -= 0.05;

                if (shooterLeftPower < 0.0) shooterLeftPower = 1.0;
                if (shooterRightPower < 0.0) shooterRightPower = 1.0;
            }

            if (gamepad1.rightBumperWasPressed() && !humanPlayerMode) {
                shooterLeftPower += 0.05;
                shooterRightPower += 0.05;

                if (shooterLeftPower > 1.0) shooterLeftPower = 0.0;
                if (shooterRightPower > 1.0) shooterRightPower = 0.0;
            }

            if (gamepad1.dpadLeftWasPressed() && !humanPlayerMode) {
                shooterLeftPower -= 0.01;
                shooterRightPower -= 0.01;

                if (shooterLeftPower < 0.0) shooterLeftPower = 1.0;
                if (shooterRightPower < 0.0) shooterRightPower = 1.0;
            }

            if (gamepad1.dpadRightWasPressed() && !humanPlayerMode) {
                shooterLeftPower += 0.01;
                shooterRightPower += 0.01;

                if (shooterLeftPower > 1.0) shooterLeftPower = 0.0;
                if (shooterRightPower > 1.0) shooterRightPower = 0.0;
            }

            if (gamepad1.x && !lastXPress && !humanPlayerMode) {
                if (shooterMode == 2) {
                    shooterMode = 0;
                } else {
                    shooterMode = 2;
                }
            }
            lastXPress = gamepad1.x;

            if (shooterMode == 2) {
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            if (gamepad1.y && !lastYPress && !humanPlayerMode) {
                if (shooterMode == 1) {
                    shooterMode = 0;
                } else {
                    shooterMode = 1;
                }
            }

            lastYPress = gamepad1.y;
            if (shooterMode == 1) {
                shooterRightPower = 0.60;
                shooterLeftPower = 0.60;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            if (gamepad1.a && !lastAPress && !humanPlayerMode) {
                if (intakeRunning && intakeWheelsPower == -1.0) {
                    intakeRunning = false;
                } else {
                    intakeRunning = true;
                    intakeWheelsPower = -1.0; // negative is intake
                }
            }
            lastAPress = gamepad1.a;

            if (gamepad1.b && !lastBPress && !humanPlayerMode) {
                if (intakeRunning && intakeWheelsPower == 1.0) {
                    intakeRunning = false;
                } else {
                    intakeRunning = true;
                    intakeWheelsPower = 1.0; // positive is outtake
                }
            }
            lastBPress = gamepad1.b;

            // Non-blocking intake sequence triggered by dpad_up
            if (gamepad1.dpad_up && !intakeSequenceRunning) {
                intakeSequenceRunning = true;
                intakeSequenceStartTime = System.currentTimeMillis();
                intakeSequenceStep = 0;
                shooterMode = 1; // Automatically enable shooter mode 1
                ballCount = 0; // Reset ball count when sequence starts
            }

            // Handle the intake sequence without blocking
            if (intakeSequenceRunning) {
                long elapsed = System.currentTimeMillis() - intakeSequenceStartTime;

                if (intakeSequenceStep < 4) {
                    long cycleTime = elapsed % 2500; // Each cycle is 2.5 seconds

                    if (elapsed >= (intakeSequenceStep + 1) * 2500) {
                        intakeSequenceStep++;
                    }

                    if (cycleTime < 1000) {
                        // First 1 second: do nothing
                        intakeWheels.setPower(0);
                    } else if (cycleTime < 1500) {
                        // Next 0.5 seconds: spin
                        intakeWheels.setPower(-1.0);
                    } else {
                        // Final 1 second: stop
                        intakeWheels.setPower(0);
                    }
                } else {
                    intakeSequenceRunning = false;
                    intakeWheels.setPower(0);
                    // Optionally turn off shooter mode after sequence completes
                    // shooterMode = 0; // Uncomment this if you want shooter to stop after sequence
                }
            } else if (intakeRunning) {
                // Normal intake control when sequence is not running
                intakeWheels.setPower(intakeWheelsPower);
            } else {
                intakeWheels.setPower(0);
            }

            // Ball counting logic using color sensor
            int totalColor = colorSensor.red() + colorSensor.green() + colorSensor.blue();

            if (totalColor > colorThreshold && !ballDetected) {
                // Ball just entered sensor view
                ballDetected = true;
                ballCount++;
            } else if (totalColor <= colorThreshold && ballDetected) {
                // Ball left sensor view
                ballDetected = false;
            }

            if (gamepad1.dpadDownWasPressed()) {
                humanPlayerMode = !humanPlayerMode;
                shooterRightPower = 0;
                shooterLeftPower = 0;
            }

            if (gamepad1.a && humanPlayerMode) {
                shooterRightPower = -0.5;
                shooterLeftPower = -0.5;

                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
                intakeWheelsPower = 0.5; //outake
                intakeWheels.setPower(intakeWheelsPower);
            }

            //telemetry.addData("FL", "%.2f", frontLeftPower);
            //telemetry.addData("BL", "%.2f", backLeftPower);
            //telemetry.addData("FR", "%.2f", frontRightPower);
            //telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addData("shooter counts/s", "%.2f", shooterRight.getVelocity());
            telemetry.addData("shooter", "%.2f", shooterRightPower);
            telemetry.addData("shooter actual", "%.2f", shooterRight.getPower());
            telemetry.addData("intake power", "%.2f", intakeWheelsPower);
            telemetry.addData("intake power actual", "%.2f", intakeWheels.getPower());
            telemetry.addData("Human Player Mode", "%b", humanPlayerMode);
            telemetry.addData("Shooter Mode", "%d", shooterMode);
            telemetry.addData("Intake Sequence Running", "%b", intakeSequenceRunning);
            telemetry.addData("Sequence Step", "%d", intakeSequenceStep);
            telemetry.addData("Ball Count", "%d", ballCount);
            telemetry.addData("Color Total", "%d", colorSensor.red() + colorSensor.green() + colorSensor.blue());
            telemetry.update();
        }
    }
}

//hi luke