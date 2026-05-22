package org.firstinspires.ftc.teamcode.config

/**
 * ============================================================
 *  HARDWARE CONFIG  —  APOC Bot (DECODE Game)
 * ============================================================
 *  Change the string names here to match your Robot Controller
 *  hardware configuration (the names you typed in the RC app).
 * ============================================================
 */
object HardwareConfig {

    // ── Mecanum Drive Motors ─────────────────────────────────
    // Swift V2 drive (Meconium)
    const val DRIVE_FRONT_LEFT  = "frontLeft"
    const val DRIVE_FRONT_RIGHT = "frontRight"
    const val DRIVE_BACK_LEFT   = "backLeft"
    const val DRIVE_BACK_RIGHT  = "backRight"

    // ── Intake ───────────────────────────────────────────────
    const val INTAKE_MOTOR      = "intake"

    // ── Turret ───────────────────────────────────────────────
    // Spin motor (360 ° travel — gear ratio set in TuningConfig)
    const val TURRET_SPIN_MOTOR = "turretSpin"
    // Hood / pivot servo (20 ° travel — limits set in TuningConfig)
    const val TURRET_HOOD_SERVO = "turretHood"

    // ── Shooter ──────────────────────────────────────────────
    const val SHOOTER_MOTOR_LEFT  = "shooterLeft"
    const val SHOOTER_MOTOR_RIGHT = "shooterRight"

    // ── HuskyLens ────────────────────────────────────────────
    // I²C device name as configured in the RC app
    const val HUSKY_LENS_DEVICE   = "huskyLens"
}
