package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.config.HardwareConfig
import org.firstinspires.ftc.teamcode.config.TuningConfig
import com.dfrobot.HuskyLens          // DFRobot HuskyLens SDK class
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * ============================================================
 *  TURRET SUBSYSTEM
 * ============================================================
 *  • Spin motor  — 360 ° travel, encoder position control
 *  • Hood servo  — 20 ° travel (pivot), mapped to servo range
 *  • HuskyLens   — AprilTag tracking for auto-aim
 *
 *  Call update() every loop tick when auto-tracking is active.
 * ============================================================
 *
 *  NOTE on HuskyLens import:
 *  Add the DFRobot HuskyLens FTC library to your build.gradle:
 *    implementation 'com.dfrobot:huskylenslibrary:1.0.0'
 *  Or copy the HuskyLens.java file from DFRobot's GitHub into
 *  your project and adjust the import above.
 * ============================================================
 */
class TurretSubsystem(hardwareMap: HardwareMap) {

    // ── Hardware ─────────────────────────────────────────────
    private val spinMotor  : DcMotor = hardwareMap.get(DcMotor::class.java, HardwareConfig.TURRET_SPIN_MOTOR)
    private val hoodServo  : Servo   = hardwareMap.get(Servo::class.java,   HardwareConfig.TURRET_HOOD_SERVO)
    private val huskyLens  : HuskyLens = hardwareMap.get(HuskyLens::class.java, HardwareConfig.HUSKY_LENS_DEVICE)

    // ── PID state ────────────────────────────────────────────
    private var targetTicks    = 0
    private var lastError      = 0.0
    private var integralSum    = 0.0
    private val pidTimer       = ElapsedTime()

    // ── Tracking mode ────────────────────────────────────────
    enum class Mode { MANUAL, AUTO_TRACK, POSITION }
    var mode: Mode = Mode.MANUAL
        private set

    // ── Hood angle (degrees 0–20) ─────────────────────────────
    var hoodAngleDeg: Double = 0.0
        private set

    init {
        // Spin motor setup
        spinMotor.direction        = DcMotorSimple.Direction.FORWARD
        spinMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        spinMotor.mode             = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        spinMotor.mode             = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // HuskyLens: switch to AprilTag recognition mode
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION)

        // Hood to safe start position
        setHoodAngle(0.0)
        pidTimer.reset()
    }

    // ════════════════════════════════════════════════════════
    //  PUBLIC API
    // ════════════════════════════════════════════════════════

    /**
     * Manual spin control — call every loop tick while driving.
     * @param power  Joystick value (-1 = CCW, +1 = CW)
     */
    fun spinManual(power: Double) {
        mode = Mode.MANUAL
        val safePower = clampToLimits(power)
        spinMotor.power = safePower
    }

    /**
     * Command turret to an absolute encoder tick position.
     * Call update() each loop to execute the move.
     */
    fun goToPosition(ticks: Int) {
        targetTicks = ticks.coerceIn(TuningConfig.TURRET_MIN_TICKS, TuningConfig.TURRET_MAX_TICKS)
        integralSum = 0.0
        lastError   = 0.0
        pidTimer.reset()
        mode = Mode.POSITION
    }

    /**
     * Enable HuskyLens AprilTag auto-tracking.
     * Call update() each loop to run the tracker.
     */
    fun enableAutoTrack() {
        mode = Mode.AUTO_TRACK
        integralSum = 0.0
        lastError   = 0.0
        pidTimer.reset()
    }

    /** Disable auto-tracking and stop turret. */
    fun disableAutoTrack() {
        mode = Mode.MANUAL
        spinMotor.power = 0.0
    }

    /**
     * Set hood angle in degrees.
     * @param degrees  0.0 = lowest position, 20.0 = highest
     */
    fun setHoodAngle(degrees: Double) {
        hoodAngleDeg = degrees.coerceIn(
            TuningConfig.HOOD_ANGLE_MIN_DEG,
            TuningConfig.HOOD_ANGLE_MAX_DEG
        )
        val t = (hoodAngleDeg - TuningConfig.HOOD_ANGLE_MIN_DEG) /
                (TuningConfig.HOOD_ANGLE_MAX_DEG - TuningConfig.HOOD_ANGLE_MIN_DEG)
        hoodServo.position = TuningConfig.HOOD_SERVO_MIN +
                             t * (TuningConfig.HOOD_SERVO_MAX - TuningConfig.HOOD_SERVO_MIN)
    }

    /** Nudge hood angle by a delta (degrees). Useful for gamepad bumper adjustments. */
    fun nudgeHood(deltaDeg: Double) = setHoodAngle(hoodAngleDeg + deltaDeg)

    /** Current encoder ticks of the spin motor. */
    val currentTicks: Int get() = spinMotor.currentPosition

    /** Stop turret spin motor. */
    fun stop() {
        spinMotor.power = 0.0
        mode = Mode.MANUAL
    }

    // ════════════════════════════════════════════════════════
    //  UPDATE  — call every loop tick
    // ════════════════════════════════════════════════════════

    /**
     * Execute whichever control mode is active.
     * Must be called every loop iteration.
     */
    fun update() {
        when (mode) {
            Mode.POSITION    -> runPositionPID()
            Mode.AUTO_TRACK  -> runHuskyTracking()
            Mode.MANUAL      -> { /* driven externally via spinManual() */ }
        }
    }

    // ════════════════════════════════════════════════════════
    //  PRIVATE HELPERS
    // ════════════════════════════════════════════════════════

    private fun runPositionPID() {
        val dt    = pidTimer.seconds().coerceAtLeast(0.001)
        pidTimer.reset()

        val error = (targetTicks - spinMotor.currentPosition).toDouble()

        if (abs(error) < TuningConfig.TURRET_TOLERANCE_TICKS) {
            spinMotor.power = 0.0
            return
        }

        integralSum += error * dt
        val derivative = (error - lastError) / dt
        lastError = error

        val power = (TuningConfig.TURRET_KP * error +
                     TuningConfig.TURRET_KI * integralSum +
                     TuningConfig.TURRET_KD * derivative)
            .coerceIn(-TuningConfig.TURRET_MAX_POWER, TuningConfig.TURRET_MAX_POWER)

        spinMotor.power = clampToLimits(power)
    }

    private fun runHuskyTracking() {
        // Request latest results from HuskyLens
        val blocks: Array<HuskyLens.Block>? = huskyLens.blocks()

        if (blocks.isNullOrEmpty()) {
            // No target visible — hold position
            spinMotor.power = 0.0
            return
        }

        // Use the first detected AprilTag (closest / highest confidence)
        val target = blocks[0]
        val errorPx = TuningConfig.HUSKY_FRAME_CENTER_X - target.x.toDouble()

        if (abs(errorPx) < TuningConfig.TURRET_TRACK_DEAD_PX) {
            spinMotor.power = 0.0
            return
        }

        val power = (TuningConfig.TURRET_TRACK_KP * errorPx)
            .coerceIn(-TuningConfig.TURRET_MAX_POWER, TuningConfig.TURRET_MAX_POWER)

        spinMotor.power = clampToLimits(power)
    }

    /**
     * Prevent spin motor from driving past soft limits.
     * Returns 0 if already at a limit and power would push further.
     */
    private fun clampToLimits(power: Double): Double {
        val pos = spinMotor.currentPosition
        if (pos <= TuningConfig.TURRET_MIN_TICKS && power < 0) return 0.0
        if (pos >= TuningConfig.TURRET_MAX_TICKS && power > 0) return 0.0
        return power
    }
}
