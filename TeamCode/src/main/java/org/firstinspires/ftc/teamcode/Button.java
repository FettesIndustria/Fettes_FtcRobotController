package org.firstinspires.ftc.teamcode;

public class Button {
    String buttonType;
    boolean onMode;
    boolean isPressed;

    public Button(String buttonType, boolean mode) {
        this.buttonType = buttonType;
        onMode = mode;
        isPressed = false;
    }
}
