package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerToggles {
    private Gamepad gamepad;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    public boolean trianglePressed = false;
    public boolean circlePressed = false;
    public boolean crossPressed = false;
    public boolean squarePressed = false;

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

        trianglePressed = gamepad.triangle;
        circlePressed = gamepad.circle;
        crossPressed = gamepad.cross;
        squarePressed = gamepad.square;

        leftBumperPressed = gamepad.left_bumper;
        rightBumperPressed = gamepad.right_bumper;

        this.gamepad = gamepad;
    }

    // NOTE - TOGGLES

    public boolean isDpadLeftToggled() {
        return gamepad.dpad_left && !dpadLeftPressed;
    }

    public boolean isDpadRightToggled() {
        return gamepad.dpad_right && !dpadRightPressed;
    }

    public boolean isDpadUpToggled() {
        return gamepad.dpad_up && !dpadUpPressed;
    }

    public boolean isDpadDownToggled() {
        return gamepad.dpad_down && !dpadDownPressed;
    }

    public boolean isTriangleToggled() {
        return gamepad.triangle && !trianglePressed;
    }

    public boolean isCircleToggled() {
        return gamepad.circle && !circlePressed;
    }

    public boolean isCrossToggled() {
        return gamepad.cross && !crossPressed;
    }

    public boolean isSquareToggled() {
        return gamepad.square && !squarePressed;
    }

    public boolean isLeftBumperToggled() {
        return gamepad.left_bumper && !leftBumperPressed;
    }

    public boolean isRightBumperToggled() {
        return gamepad.right_bumper && !rightBumperPressed;
    }
}
