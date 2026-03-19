package org.firstinspires.ftc.teamcode.Teliop;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * GamepadControls.java
 *
 * Handles all gamepad input and edge detection.
 * Call update() once per loop, then use the provided methods/fields
 * to read button states in the main OpMode.
 *
 * CONTROL MAP:
 * -------------------------------------------------------
 * LEFT  STICK  Y         Drive forward/backward
 * LEFT  STICK  X         Strafe left/right
 * RIGHT STICK  X         Turn left/right
 *
 * A                      Toggle intake inward  (-1.0)
 * B                      Toggle intake outward (+1.0)
 * X                      Toggle manual shooter on/off
 * Y                      Toggle stop servo position
 *
 * LEFT  TRIGGER          Tilt servos DOWN
 * RIGHT TRIGGER          Tilt servos UP
 *
 * LEFT  BUMPER           Decrease shooter power by 0.05
 * RIGHT BUMPER           Increase shooter power by 0.05
 *
 * DPAD LEFT              Decrease shooter power by 0.01
 * DPAD RIGHT             Increase shooter power by 0.01
 * DPAD UP                Toggle rapid shoot mode
 * DPAD DOWN              Toggle human player mode
 * -------------------------------------------------------
 */
public class GamepadControls {

    // ========================================
    // RAW AXIS VALUES (updated every loop)
    // ========================================
    public double drive;    // Left stick Y
    public double strafe;   // Left stick X
    public double turn;     // Right stick X

    public boolean leftTriggerHeld;   // Left trigger > 0.5
    public boolean rightTriggerHeld;  // Right trigger > 0.5

    // ========================================
    // BUTTON PRESS EVENTS (true for one loop on press)
    // ========================================
    public boolean pressedA;
    public boolean pressedB;
    public boolean pressedX;
    public boolean pressedY;

    public boolean pressedLeftBumper;
    public boolean pressedRightBumper;

    public boolean pressedDpadUp;
    public boolean pressedDpadDown;
    public boolean pressedDpadLeft;
    public boolean pressedDpadRight;

    // ========================================
    // PREVIOUS BUTTON STATES (internal)
    // ========================================
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    private final Gamepad gamepad;

    public GamepadControls(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Call once per loop iteration to refresh all input values.
     */
    public void update() {

        // --- Axes ---
        drive  = applyDeadzone(-gamepad.left_stick_y);
        strafe = applyDeadzone(gamepad.left_stick_x);
        turn   = applyDeadzone(gamepad.right_stick_x);

        // --- Triggers ---
        leftTriggerHeld  = gamepad.left_trigger  > 0.5;
        rightTriggerHeld = gamepad.right_trigger > 0.5;

        // --- Edge-detected button presses ---
        pressedA = gamepad.a && !lastA;
        pressedB = gamepad.b && !lastB;
        pressedX = gamepad.x && !lastX;
        pressedY = gamepad.y && !lastY;

        pressedLeftBumper  = gamepad.left_bumper  && !lastLeftBumper;
        pressedRightBumper = gamepad.right_bumper && !lastRightBumper;

        pressedDpadUp    = gamepad.dpad_up    && !lastDpadUp;
        pressedDpadDown  = gamepad.dpad_down  && !lastDpadDown;
        pressedDpadLeft  = gamepad.dpad_left  && !lastDpadLeft;
        pressedDpadRight = gamepad.dpad_right && !lastDpadRight;

        // --- Store current states for next loop ---
        lastA = gamepad.a;
        lastB = gamepad.b;
        lastX = gamepad.x;
        lastY = gamepad.y;

        lastLeftBumper  = gamepad.left_bumper;
        lastRightBumper = gamepad.right_bumper;

        lastDpadUp    = gamepad.dpad_up;
        lastDpadDown  = gamepad.dpad_down;
        lastDpadLeft  = gamepad.dpad_left;
        lastDpadRight = gamepad.dpad_right;
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < TunningTeliop.DEADZONE ? 0.0 : value;
    }
}