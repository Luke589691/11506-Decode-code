package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.config.TuningConfig

/**
 * ============================================================
 *  TUNING OPMODE  —  Turret PID + Hood Calibration
 * ============================================================
 *  Run this op-mode to:
 *    • Find your turret ticks-per-degree
 *    • Tune turret PID gains
 *    • Calibrate hood servo min/max positions
 *    • Verify shooter speed
 *
 *  GAMEPAD 1:
 *    A           → Send turret to 0 ticks (home)
 *    B           → Send turret to 180 ° position
 *    Y           → Send turret to 360 ° position
 *    X           → Toggle shooter
 *    D-Pad UP    → Hood angle + 1 °
 *    D-Pad DOWN  → Hood angle - 1 °
 *    Right stick X → Manual turret spin
 * ============================================================
 */
@TeleOp(name = "TUNING – Turret & Hood", group = "DECODE Tuning")
class TuningOpMode : LinearOpMode() {

    private lateinit var turret : TurretSubsystem
    private lateinit var shooter: ShooterSubsystem

    private var prevA = false; private var prevB = false
    private var prevY = false; private var prevX = false
    private var prevUp = false; private var prevDown = false

    override fun runOpMode() {
        turret  = TurretSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)

        telemetry.addLine("Tuning OpMode ready. Press PLAY.")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {

            // Turret positions
            val curA = gamepad1.a; if (curA && !prevA) turret.goToPosition(0); prevA = curA
            val curB = gamepad1.b
            if (curB && !prevB) turret.goToPosition((TuningConfig.TURRET_TICKS_PER_DEGREE * 180).toInt())
            prevB = curB
            val curY = gamepad1.y
            if (curY && !prevY) turret.goToPosition(TuningConfig.TURRET_MAX_TICKS)
            prevY = curY

            // Manual spin (when not in position mode)
            if (turret.mode == TurretSubsystem.Mode.MANUAL) {
                turret.spinManual(gamepad1.right_stick_x.toDouble())
            }

            // Hood
            val curUp   = gamepad1.dpad_up;   if (curUp   && !prevUp)   turret.nudgeHood(+1.0); prevUp   = curUp
            val curDown = gamepad1.dpad_down;  if (curDown && !prevDown) turret.nudgeHood(-1.0); prevDown = curDown

            // Shooter
            val curX = gamepad1.x; if (curX && !prevX) shooter.toggle(); prevX = curX

            turret.update()

            with(telemetry) {
                addLine("══ TURRET TUNING ══")
                addData("Mode",              turret.mode)
                addData("Current ticks",     turret.currentTicks)
                addData("Ticks/deg (calc)",  TuningConfig.TURRET_TICKS_PER_DEGREE)
                addData("Max ticks (calc)",  TuningConfig.TURRET_MAX_TICKS)
                addLine("")
                addLine("══ HOOD ══")
                addData("Hood angle (°)",    turret.hoodAngleDeg)
                addData("Servo MIN pos",     TuningConfig.HOOD_SERVO_MIN)
                addData("Servo MAX pos",     TuningConfig.HOOD_SERVO_MAX)
                addLine("")
                addLine("══ SHOOTER ══")
                addData("State",             shooter.state)
                addData("At speed",          shooter.isAtSpeed)
                addData("Power",             shooter.currentPower)
                update()
            }
        }

        turret.stop()
        shooter.stop()
    }
}
