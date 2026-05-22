package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

/**
 * ============================================================
 *  AUTONOMOUS  —  APOC Bot  (DECODE Game)
 * ============================================================
 *  Skeleton auto routine — fill in your game-specific steps.
 *
 *  Pattern: driveFor() and strafeFor() use timed dead-reckoning.
 *  Replace with Road Runner or odometry when ready.
 * ============================================================
 */
@Autonomous(name = "APOC Auto", group = "DECODE")
class AutoMain : LinearOpMode() {

    private lateinit var drive  : DriveSubsystem
    private lateinit var intake : IntakeSubsystem
    private lateinit var turret : TurretSubsystem
    private lateinit var shooter: ShooterSubsystem

    private val timer = ElapsedTime()

    override fun runOpMode() {

        drive   = DriveSubsystem(hardwareMap)
        intake  = IntakeSubsystem(hardwareMap)
        turret  = TurretSubsystem(hardwareMap)
        shooter = ShooterSubsystem(hardwareMap)

        telemetry.addLine("APOC Auto ready. Press PLAY.")
        telemetry.update()
        waitForStart()
        timer.reset()

        // ── Auto routine (edit below) ─────────────────────────

        // 1. Spin up shooter while driving forward
        shooter.spinUp()
        driveFor(seconds = 1.5, y = 0.5)

        // 2. Enable turret auto-track and wait for lock-on
        turret.enableAutoTrack()
        waitSeconds(1.0) {
            turret.update()
            telemetry.addData("Turret mode", turret.mode)
            telemetry.update()
        }

        // 3. Fire when shooter is at speed
        if (shooter.isAtSpeed) {
            // TODO: trigger your feeding mechanism here
            waitSeconds(1.0) { turret.update() }
        }

        // 4. Stop everything
        shooter.stop()
        turret.stop()
        drive.stop()
    }

    // ════════════════════════════════════════════════════════
    //  UTILITY HELPERS
    // ════════════════════════════════════════════════════════

    /** Drive in a direction for a set time (seconds). */
    private fun driveFor(seconds: Double, y: Double = 0.0, x: Double = 0.0, rx: Double = 0.0) {
        timer.reset()
        while (opModeIsActive() && timer.seconds() < seconds) {
            drive.drive(y, x, rx)
            turret.update()
        }
        drive.stop()
    }

    /**
     * Wait for [seconds] while executing [block] every loop tick.
     * Block receives elapsed seconds.
     */
    private fun waitSeconds(seconds: Double, block: () -> Unit = {}) {
        timer.reset()
        while (opModeIsActive() && timer.seconds() < seconds) {
            block()
        }
    }
}
