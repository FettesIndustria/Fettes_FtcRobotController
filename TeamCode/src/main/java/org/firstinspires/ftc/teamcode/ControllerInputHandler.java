package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;



public class ControllerInputHandler {
    private Gamepad gamepad;

    public ControllerInputHandler (Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // buttons
    public boolean isButtonPressed(char buttonName) {
        switch (buttonName) {
            case 'a':
                return gamepad.a;
            case 'b':
                return gamepad.b;
            case 'x':
                return gamepad.x;
            case 'y':
                return gamepad.y;
            default:
                return false;
        }
    }

    public boolean isButtonPressed(String buttonName) {
        switch (buttonName) {
            case "options":
                return gamepad.options;
            case "cross":
                return gamepad.cross;
            case "square":
                return gamepad.square;
            case "circle":
                return gamepad.circle;
            case "triangle":
                return gamepad.triangle;
            default:
                return false;
        }
    }


    // joysticks
    public float getLeftStickX() {
        return gamepad.left_stick_x;
    }

    public float getLeftStickY() {
        return -gamepad.left_stick_y;
    }

    public float getRightStickX() {
        return gamepad.right_stick_x;
    }

    public float getRightStickY() {
        return -gamepad.right_stick_y;
    }

    // triggers
    public float leftTrigger() {
        return gamepad.left_trigger;
    }

    public float rightTrigger() {
        return gamepad.right_trigger;
    }

    // bumpers
    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean dpad(String direction) {
        switch (direction) {
            case "up":
                return gamepad.dpad_up;
            case "down":
                return gamepad.dpad_down;
            case "left":
                return gamepad.dpad_left;
            case "right":
                return gamepad.dpad_right;
            default:
                return false;
        }
    }
}


