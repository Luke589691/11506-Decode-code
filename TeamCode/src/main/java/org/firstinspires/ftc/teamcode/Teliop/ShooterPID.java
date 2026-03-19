package org.firstinspires.ftc.teamcode.Teliop;

/**
 * ShooterPID.java
 *
 * Velocity PID controller for the shooter motors.
 * Reads actual RPM from the motor encoders and adjusts output power
 * to hold the target RPM steady, even as the robot moves or battery drops.
 *
 * HOW IT WORKS:
 *   - You set a target velocity (in ticks/sec or RPM — consistent with your motor)
 *   - Each loop, call compute(currentVelocity) to get the corrected power output
 *   - Feed that output directly to shooterLeft.setPower() / shooterRight.setPower()
 *
 * TUNING GUIDE (adjust kP, kI, kD in TunningTeliop.java):
 *   1. Set kI = 0, kD = 0. Raise kP until the shooter reaches speed but oscillates.
 *   2. Lower kP slightly until oscillation stops.
 *   3. Raise kI slowly to eliminate steady-state error (shooter slightly under target).
 *   4. Add a small kD to dampen overshoot if needed. Usually kD can stay near 0.
 */
public class ShooterPID {

    private double kP, kI, kD;

    private double targetVelocity = 0.0;

    private double integralSum    = 0.0;
    private double lastError      = 0.0;
    private long   lastTimeMs     = 0;

    private static final double INTEGRAL_MAX = 0.3; // Clamp to prevent integral windup

    public ShooterPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Set the desired velocity (ticks/sec from DcMotorEx.getVelocity()).
     * Call this whenever the target changes.
     */
    public void setTarget(double targetVelocity) {
        if (this.targetVelocity != targetVelocity) {
            this.targetVelocity = targetVelocity;
            reset(); // Clear integral/derivative history on target change
        }
    }

    public double getTarget() {
        return targetVelocity;
    }

    /**
     * Call once per loop with the current measured velocity.
     * Returns a motor power value clamped to [-1.0, 1.0].
     *
     * @param currentVelocity  ticks/sec from motor.getVelocity()
     * @return motor power to apply
     */
    public double compute(double currentVelocity) {
        long now = System.currentTimeMillis();

        // First call — initialize timer and return feedforward only
        if (lastTimeMs == 0) {
            lastTimeMs = now;
            lastError  = targetVelocity - currentVelocity;
            return feedforward();
        }

        double dt = (now - lastTimeMs) / 1000.0; // seconds
        lastTimeMs = now;

        if (dt <= 0) return feedforward(); // Guard against zero dt

        double error = targetVelocity - currentVelocity;

        // Proportional
        double P = kP * error;

        // Integral with windup clamp
        integralSum += error * dt;
        integralSum  = Math.max(-INTEGRAL_MAX / kI, Math.min(INTEGRAL_MAX / kI, integralSum));
        double I = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double D = kD * derivative;
        lastError = error;

        double output = feedforward() + P + I + D;

        // Clamp to valid motor power range
        return Math.max(-1.0, Math.min(1.0, output));
    }

    /**
     * Feedforward: scales target velocity to an approximate base power.
     * This gives the PID a head start so P/I/D only correct the small remaining error.
     * MAX_TICKS_PER_SEC is the free-spin velocity of your motor at full power.
     */
    private double feedforward() {
        return targetVelocity / TunningTeliop.SHOOTER_MAX_TICKS_PER_SEC;
    }

    /**
     * Reset integral and derivative state.
     * Call when the shooter turns off or target changes significantly.
     */
    public void reset() {
        integralSum = 0.0;
        lastError   = 0.0;
        lastTimeMs  = 0;
    }

    // --- Getters for telemetry ---
    public double getLastError()    { return lastError; }
    public double getIntegralSum()  { return integralSum; }
}