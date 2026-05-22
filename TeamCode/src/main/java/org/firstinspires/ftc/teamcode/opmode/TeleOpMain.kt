package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.config.TuningConfig

/**
 * ============================================================
 *  TELEOP  —  APOC Bot  (DECODE Game)
 * ============================================================
 *
 *  ── GAMEPAD 1 (Driver) ──────────────────────────────────
 *  Left  stick  X/Y   →  Drive (forward / strafe)
 *  Right stick  X     →  Rotate
 *  Left  bumper       →  Slow mode (hold)
 *
 *  ── GAMEPAD 2 (Operator) ────────────────────────────────
 *  A                  →  Intake toggle
 *  B                  →  Intake reverse (hold)
 *  X                  →  Shooter toggle
 *  Y                  →  Shoot (when at speed)
 *
 *  Right stick X      →  Turret manual spin
 *  Right bumper       →  Enable HuskyLens auto-track
 *  Left  bumper       →  Disable auto-track
 *
 *  D-Pad UP / DOWN    →  Hood angle +1° / -1°
 * ============================================================
 */
@TeleOp(name = "APOC TeleOp", group = "DECODE")
class TeleOpMain : LinearOpMode() {

    private lateinit var drive  : DriveSubsystem
    private lateinit var intake : IntakeSubsystem
    private lateinit var turret : TurretSubsystem
    private lateinit var shooter: ShooterSubsystem

    // Button edge-detection state
    private var prevIntakeA    = false
    private var prevShooterX   = false
    private var prevTrackOn    = false
    private var prevTrackOff   = false
    private var prevDpadUp     = false
    private var prevDpadDown   = false

    override fun runOpMode() {

        // ── Init ──────────────────────────────────────────────
        drive   = DriveSubsystem(hardwareMap)
        intake  = IntakeSubsystem(hardwareMap)
        turret  = TurretSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)

        telemetry.addLine("APOC Bot ready. Press PLAY.")
        telemetry.update()
        waitForStart()

        // ── Main loop ─────────────────────────────────────────
        while (opModeIsActive()) {

            // ─── GAMEPAD 1  (Driver) ───────────────────────
            val slowMode = gamepad1.left_bumper
            drive.slowMode = slowMode

            drive.drive(
                y  = -gamepad1.left_stick_y.toDouble(),   // FTC Y-axis is inverted
                x  =  gamepad1.left_stick_x.toDouble(),
                rx =  gamepad1.right_stick_x.toDouble()
            )

            // ─── GAMEPAD 2  (Operator) ─────────────────────

            // Intake toggle (A)
            val currA = gamepad2.a
            if (currA && !prevIntakeA) intake.toggle()
            prevIntakeA = currA

            // Intake reverse (hold B)
            if (gamepad2.b) intake.reverse()
            else if (!gamepad2.a && intake.state == IntakeSubsystem.State.REVERSING) intake.stop()

            // Shooter toggle (X)
            val currX = gamepad2.x
            if (currX && !prevShooterX) shooter.toggle()
            prevShooterX = currX

            // Turret manual spin (right stick X) — only when not auto-tracking
            if (turret.mode == TurretSubsystem.Mode.MANUAL) {
                turret.spinManual(gamepad2.right_stick_x.toDouble())
            }

            // Auto-track toggle (bumpers)
            val currTrackOn  = gamepad2.right_bumper
            val currTrackOff = gamepad2.left_bumper
            if (currTrackOn  && !prevTrackOn)  turret.enableAutoTrack()
            if (currTrackOff && !prevTrackOff) turret.disableAutoTrack()
            prevTrackOn  = currTrackOn
            prevTrackOff = currTrackOff

            // Hood angle (D-Pad up / down)
            val currDpadUp   = gamepad2.dpad_up
            val currDpadDown = gamepad2.dpad_down
            if (currDpadUp   && !prevDpadUp)   turret.nudgeHood(+1.0)
            if (currDpadDown && !prevDpadDown)  turret.nudgeHood(-1.0)
            prevDpadUp   = currDpadUp
            prevDpadDown = currDpadDown

            // ─── Turret update (PID / tracking loop) ───────
            turret.update()

            // ─── Telemetry ──────────────────────────────────
            with(telemetry) {
                addLine("=== APOC DECODE TeleOp ===")
                addData("Drive slow mode",    drive.slowMode)
                addData("Intake",             intake.state)
                addData("Shooter",            shooter.state)
                addData("Shooter at speed",   shooter.isAtSpeed)
                addData("Turret mode",        turret.mode)
                addData("Turret ticks",       turret.currentTicks)
                addData("Hood angle (°)",     turret.hoodAngleDeg)
                update()
            }
        }

        // ── On stop ───────────────────────────────────────────
        drive.stop()
        intake.stop()
        turret.stop()
        shooter.stop()
    }
}
