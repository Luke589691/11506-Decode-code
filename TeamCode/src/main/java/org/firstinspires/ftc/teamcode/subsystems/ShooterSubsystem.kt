package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.HardwareConfig
import org.firstinspires.ftc.teamcode.config.TuningConfig
import kotlin.math.abs

/**
 * ============================================================
 *  SHOOTER SUBSYSTEM  —  Dual Motor Flywheel Shooter
 * ============================================================
 *  Both motors spin at the same commanded speed.
 *  Left motor FORWARD, right motor REVERSE so both flywheels
 *  push the game element in the same direction — flip the
 *  directions in init{} if your wiring is the other way.
 * ============================================================
 */
class ShooterSubsystem(hardwareMap: HardwareMap) {

    private val leftMotor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, HardwareConfig.SHOOTER_MOTOR_LEFT)
    private val rightMotor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, HardwareConfig.SHOOTER_MOTOR_RIGHT)

    enum class State { SPINNING_UP, AT_SPEED, IDLE }
    var state: State = State.IDLE
        private set

    init {
        leftMotor .direction        = DcMotorSimple.Direction.FORWARD
        rightMotor.direction        = DcMotorSimple.Direction.REVERSE
        leftMotor .zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        leftMotor .mode             = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode             = DcMotor.RunMode.RUN_USING_ENCODER
    }

    /** Spin flywheels to target shooting speed. */
    fun spinUp() {
        setSpeed(TuningConfig.SHOOTER_SPEED)
        state = State.SPINNING_UP
    }

    /** Stop flywheels completely. */
    fun stop() {
        setSpeed(TuningConfig.SHOOTER_IDLE_SPEED)
        state = State.IDLE
    }

    /** Manually set shooter speed (0.0 – 1.0). */
    fun setSpeed(speed: Double) {
        val clamped = speed.coerceIn(0.0, 1.0)
        leftMotor .power = clamped
        rightMotor.power = clamped
        state = if (clamped > 0.01) State.SPINNING_UP else State.IDLE
    }

    /** Toggle between full speed and stopped. */
    fun toggle() {
        if (state == State.IDLE) spinUp() else stop()
    }

    /**
     * Returns true when both motors are approximately up to speed.
     * Uses encoder velocity if available; falls back to a power check.
     */
    val isAtSpeed: Boolean
        get() {
            val leftVel  = leftMotor.velocity
            val rightVel = rightMotor.velocity
            val targetVel = TuningConfig.SHOOTER_SPEED *
                            3000.0  // rough ticks/sec at full power — tune to your motor

            val atSpeed = if (abs(leftVel) > 1.0 && abs(rightVel) > 1.0) {
                val leftDiff  = abs(targetVel - leftVel)
                val rightDiff = abs(targetVel - rightVel)
                leftDiff  < TuningConfig.SHOOTER_VELOCITY_TOL &&
                rightDiff < TuningConfig.SHOOTER_VELOCITY_TOL
            } else {
                // No encoder velocity — just check power is set
                leftMotor.power > 0.8
            }

            // Update state based on current velocity
            if (atSpeed && state == State.SPINNING_UP) {
                state = State.AT_SPEED
            } else if (!atSpeed && state == State.AT_SPEED) {
                state = State.SPINNING_UP
            }

            return atSpeed
        }

    /** Current power of the left flywheel motor (for telemetry). */
    val currentPower: Double get() = leftMotor.power
}
