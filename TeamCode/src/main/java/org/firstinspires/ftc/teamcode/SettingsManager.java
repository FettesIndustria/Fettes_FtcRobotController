package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SettingsManager {
    public ControllerInputHandler controllerInput;
    public Gamepad gamepad;
    public Telemetry telemetry;
    public RobotMove robotMove;

    public Button settingsButton, robotCentricMovement, fieldCentricMovement, orientationButton, brushButton, handButton;

    public SettingsManager(Gamepad gamepad, RobotMove robotMove, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.robotMove = robotMove;
        this.telemetry = telemetry;
        controllerInput = new ControllerInputHandler(gamepad);

        // initialise buttons
        settingsButton = new Button("options", false);
        robotCentricMovement = new Button("square", true);
        fieldCentricMovement = new Button("square", false);
        orientationButton = new Button("cross", false);
        brushButton = new Button("dpadright", false);
        handButton = new Button("triangle", false);
    }

    // checks for a button toggle and returns whether or not the button mode has toggled (pressed)
    public boolean checkButton(Button button) {
        boolean hasToggled = false;
        boolean pressed = controllerInput.isButtonPressed(button.buttonType);
        if (!button.isPressed && pressed) {
            // toggle mode
            button.onMode = !button.onMode;
            hasToggled = true;
        }
        button.isPressed = pressed;
        return hasToggled;
    }

    private void printSettings() {
        telemetry.clearAll();
        telemetry.addData("", "Options:");
        telemetry.addData("", "Press Square to toggle the movement mode");
        telemetry.addData("", "Press Cross to set a new default orientation");
        telemetry.addData("", "Press ");
        telemetry.addData("", "\n");
    }

    public void changeSettings() {
        if (checkButton(fieldCentricMovement)) {
            // toggle movement mode
            telemetry.addData("", "Switched to Field Centric Movement");
        }

        if (checkButton(robotCentricMovement)) {
            // toggle movement mode
            telemetry.addData("", "Switched to Robot Centric Movement");
        }

        if (checkButton(orientationButton)) {
            // set new default orientation
            robotMove.setDefaultOrientation();
            telemetry.addData("", "Set new default orientation");
        }
    }

    public void checkSettingsButton() {
        if (checkButton(settingsButton)) {
            if (settingsButton.onMode) {
                // in settings, print commands
                printSettings();
            } else {
                // exited settings mode
            }
        }
    }
}

