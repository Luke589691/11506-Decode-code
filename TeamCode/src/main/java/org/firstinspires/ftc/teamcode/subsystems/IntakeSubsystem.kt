package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.HardwareConfig
import org.firstinspires.ftc.teamcode.config.TuningConfig

/**
 * ============================================================
 *  INTAKE SUBSYSTEM  —  Single Motor Intake
 * ============================================================
 */
class IntakeSubsystem(hardwareMap: HardwareMap) {

    private val intakeMotor: DcMotor =
        hardwareMap.get(DcMotor::class.java, HardwareConfig.INTAKE_MOTOR)

    enum class State { RUNNING, REVERSING, STOPPED }
    var state: State = State.STOPPED
        private set

    init {
        intakeMotor.direction        = DcMotorSimple.Direction.FORWARD
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intakeMotor.mode             = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /** Spin intake inward to collect game elements. */
    fun intake() {
        intakeMotor.power = TuningConfig.INTAKE_SPEED
        state = State.RUNNING
    }

    /** Spin intake outward to eject / clear jams. */
    fun reverse() {
        intakeMotor.power = TuningConfig.INTAKE_REVERSE_SPEED
        state = State.REVERSING
    }

    /** Stop the intake. */
    fun stop() {
        intakeMotor.power = 0.0
        state = State.STOPPED
    }

    /** Toggle between running and stopped. */
    fun toggle() {
        if (state == State.RUNNING) stop() else intake()
    }
}
