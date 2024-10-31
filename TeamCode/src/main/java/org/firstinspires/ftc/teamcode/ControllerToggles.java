package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerToggles {
    private Gamepad gamepad;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    public boolean aPressed = false;
    public boolean bPressed = false;
    public boolean xPressed = false;
    public boolean yPressed = false;

    public boolean leftBumperPressed = false;
    public boolean rightBumperPressed = false;

    public ControllerToggles(Gamepad gamepad) {
        update(gamepad);
    }

    public void update(Gamepad gamepad) {
        dpadLeftPressed = gamepad.dpad_left;
        dpadRightPressed = gamepad.dpad_right;
        dpadUpPressed = gamepad.dpad_up;
        dpadDownPressed = gamepad.dpad_down;

        aPressed = gamepad.a;
        bPressed = gamepad.b;
        xPressed = gamepad.x;
        yPressed = gamepad.y;
        leftBumperPressed = gamepad.left_bumper;
        rightBumperPressed = gamepad.right_bumper;

        this.gamepad = gamepad;
    }

    // NOTE - TOGGLES

    public boolean dpadLeftToggled() {
        return gamepad.dpad_left && !dpadLeftPressed;
    }

    public boolean dpadRightToggled() {
        return gamepad.dpad_right && !dpadRightPressed;
    }

    public boolean dpadUpToggled() {
        return gamepad.dpad_up && !dpadUpPressed;
    }

    public boolean dpadDownToggled() {
        return gamepad.dpad_down && !dpadDownPressed;
    }

    public boolean aToggled() {
        return gamepad.a && !aPressed;
    }

    public boolean bToggled() {
        return gamepad.b && !bPressed;
    }

    public boolean xToggled() {
        return gamepad.x && !xPressed;
    }

    public boolean yToggled() {
        return gamepad.y && !yPressed;
    }

    public boolean leftBumperToggled() {
        return gamepad.left_bumper && !leftBumperPressed;
    }

    public boolean rightBumperToggled() {
        return gamepad.right_bumper && !rightBumperPressed;
    }
}
