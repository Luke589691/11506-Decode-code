package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "mecanumTest")
    public class MecanumWheelTest extends LinearOpMode {
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;



        @Override
        public void runOpMode() {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            waitForStart();

            while (opModeIsActive()) {
                double frontLeftPower = 0.0;
                double backLeftPower = 0.0;
                double frontRightPower = 0.0;
                double backRightPower = 0.0;
                if (gamepad1.a) {
                    frontLeftPower = 0.5;
                }


                if (gamepad1.b) {
                    backLeftPower = 0.5;
                }

                if (gamepad1.x) {
                    frontRightPower = 0.5;
                }

                if (gamepad1.y) {
                    backRightPower = 0.5;
                }

                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);

                telemetry.addData("FL", "%.2f", frontLeftPower);
                telemetry.addData("BL", "%.2f", backLeftPower);
                telemetry.addData("FR", "%.2f", frontRightPower);
                telemetry.addData("BR", "%.2f", backRightPower);
                telemetry.update();
            }
        }
}
