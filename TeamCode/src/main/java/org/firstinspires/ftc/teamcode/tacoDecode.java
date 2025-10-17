package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Taco Decode")
public class tacoDecode extends LinearOpMode {
    public DcMotorEx frontLeft = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;

    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;

    double shooterLeftPower = 0.0;
    double shooterRightPower = 0.0;

    public DcMotorEx intakeWheels = null;

    double intakeWheelsPower = 0.0;

    boolean intakeRunning = false;
    boolean lastAPress = false;
    boolean lastBPress = false;

    boolean lastYPress = false;
    boolean lastXPress = false;
//    boolean shooterRunning = false;
    int shooterMode = 0;

    boolean humanPlayerMode = false;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeWheels = hardwareMap.get(DcMotorEx.class, "intakeWheels");

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

            if (gamepad1.dpadRightWasPressed() && !humanPlayerMode){
                shooterLeftPower += 0.01;
                shooterRightPower += 0.01;

                if (shooterLeftPower > 1.0) shooterLeftPower = 0.0;
                if (shooterRightPower > 1.0) shooterRightPower = 0.0;
            }

            if (gamepad1.x && !lastXPress && !humanPlayerMode) { //todo fix toggle
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
                shooterRightPower = 0.62;
                shooterLeftPower = 0.62;
                shooterRight.setPower(shooterRightPower);
                shooterLeft.setPower(shooterLeftPower);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }


            if (gamepad1.a && !lastAPress && !humanPlayerMode) { //todo make dpad left on for 1 sec off for 1 sec 3x for intake 3 balls
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

            if (intakeRunning) {
                intakeWheels.setPower(intakeWheelsPower);
            } else {
                intakeWheels.setPower(0);
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

            telemetry.addData("FL", "%.2f", frontLeftPower)
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addData("shooter counts/s", "%.2f", shooterRight.getVelocity());
            telemetry.addData("shooter", "%.2f", shooterRightPower);
            telemetry.addData("shooter actual", "%.2f", shooterRight.getPower());
            telemetry.addData("intake power", "%.2f", intakeWheelsPower);
            telemetry.addData("intake power actual", "%.2f", intakeWheels.getPower());
            telemetry.addData("Human Player Mode", "%b", humanPlayerMode);
            telemetry.addData("Shooter Mode", "%d", shooterMode);

            telemetry.update();
        }
    }
}
