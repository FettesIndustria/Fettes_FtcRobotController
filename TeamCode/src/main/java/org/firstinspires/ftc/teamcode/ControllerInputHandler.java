package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;



public class ControllerInputHandler {
    private Gamepad gamepad;

    public ControllerInputHandler (Gamepad gamepad) {
        this.gamepad = gamepad;
    }


    // buttons
    public boolean isButtonPressed(char button) {
        switch (button) {
            case 'a':
                return gamepad.a;
            case 'b':
                return gamepad.b;
            case 'x':
                return gamepad.x;
            case 'y':
                return gamepad.y;
        }
        return false;
    }

    // joysticks
    public double getLeftStickX() {
        return gamepad.left_stick_x;
    }

    public double getLeftStickY() {
        return gamepad.left_stick_y;
    }

    public double getRightStickX() {
        return gamepad.right_stick_x;
    }

    public double getRightStickY() {
        return gamepad.right_stick_y;
    }

    // triggers
    public double leftTrigger() {
        return gamepad.left_trigger;
    }

    public double rightTrigger() {
        return gamepad.right_trigger;
    }

    // bumpers
    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }
}


