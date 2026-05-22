package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.HardwareConfig
import org.firstinspires.ftc.teamcode.config.TuningConfig
import kotlin.math.abs
import kotlin.math.max

/**
 * ============================================================
 *  DRIVE SUBSYSTEM  —  Meconium / Swift V2 Mecanum Drive
 * ============================================================
 *
 *  Coordinate convention (robot-centric):
 *    +x  = strafe right
 *    +y  = drive forward
 *    +rx = rotate clockwise
 *
 *  Power mixing:
 *    FL = y + x + rx
 *    FR = y - x - rx
 *    BL = y - x + rx
 *    BR = y + x - rx
 * ============================================================
 */
class DriveSubsystem(hardwareMap: HardwareMap) {

    private val frontLeft  : DcMotor = hardwareMap.get(DcMotor::class.java, HardwareConfig.DRIVE_FRONT_LEFT)
    private val frontRight : DcMotor = hardwareMap.get(DcMotor::class.java, HardwareConfig.DRIVE_FRONT_RIGHT)
    private val backLeft   : DcMotor = hardwareMap.get(DcMotor::class.java, HardwareConfig.DRIVE_BACK_LEFT)
    private val backRight  : DcMotor = hardwareMap.get(DcMotor::class.java, HardwareConfig.DRIVE_BACK_RIGHT)

    var slowMode: Boolean = false

    init {
        // ── Motor directions (flip if wheels spin backwards) ──
        frontLeft .direction = DcMotorSimple.Direction.REVERSE
        backLeft  .direction = DcMotorSimple.Direction.REVERSE
        frontRight.direction = DcMotorSimple.Direction.FORWARD
        backRight .direction = DcMotorSimple.Direction.FORWARD

        // ── Brake on stop ─────────────────────────────────────
        listOf(frontLeft, frontRight, backLeft, backRight).forEach { motor ->
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    /**
     * Drive the robot using field-relative or robot-centric inputs.
     *
     * @param y    Forward/back  (-1 = back,  +1 = forward)
     * @param x    Strafe        (-1 = left,  +1 = right)
     * @param rx   Rotation      (-1 = CCW,   +1 = CW)
     */
    fun drive(y: Double, x: Double, rx: Double) {
        val speedScale = if (slowMode) TuningConfig.DRIVE_SLOW_SPEED
                         else          TuningConfig.DRIVE_MAX_SPEED

        val cx = x * TuningConfig.DRIVE_STRAFE_CORRECT

        val fl = (y + cx + rx) * speedScale
        val fr = (y - cx - rx) * speedScale
        val bl = (y - cx + rx) * speedScale
        val br = (y + cx - rx) * speedScale

        // Normalise so no value exceeds ±1
        val maxMag = max(1.0, max(max(abs(fl), abs(fr)), max(abs(bl), abs(br))))

        frontLeft .power = fl / maxMag
        frontRight.power = fr / maxMag
        backLeft  .power = bl / maxMag
        backRight .power = br / maxMag
    }

    /** Stop all drive motors immediately. */
    fun stop() {
        frontLeft.power  = 0.0
        frontRight.power = 0.0
        backLeft.power   = 0.0
        backRight.power  = 0.0
    }
}
