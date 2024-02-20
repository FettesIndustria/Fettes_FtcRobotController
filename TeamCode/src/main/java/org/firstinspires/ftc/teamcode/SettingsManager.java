package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SettingsManager {
    public ControllerInputHandler controllerInput;
    public Gamepad gamepad;
    public Telemetry telemetry;

    public RobotMove robotMove;

    public Button settingsButton, robotCentricMovement, fieldCentricMovement, orientationButton;

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
        telemetry.addData("settings", "Options:");
        telemetry.addData("settings", "Press Square to toggle the movement mode");
        telemetry.addData("settings", "Press Cross to set a new default orientation");
        telemetry.addData("settings", "Press ");
        telemetry.addData("settings", "\n");
    }

    public void changeSettings() {
        if (checkButton(fieldCentricMovement)) {
            // toggle movement mode
            telemetry.addData("movement", "Switched to Field Centric Movement");
        }

        if (checkButton(robotCentricMovement)) {
            // toggle movement mode
            telemetry.addData("movement", "Switched to Robot Centric Movement");
        }

        if (checkButton(orientationButton)) {
            // set new default orientation
            robotMove.setDefaultOrientation();
            telemetry.addData("orientation", "Set new default orientation");
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

