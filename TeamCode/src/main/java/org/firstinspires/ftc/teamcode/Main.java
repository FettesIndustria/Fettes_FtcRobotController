package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {

    private ControllerInputHandler controllerInput;
    private RobotMove robotMove;
    private SettingsManager settings;
    private RobotArm robotArm;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap);
        settings = new SettingsManager(gamepad1, robotMove, telemetry);
        robotArm = new RobotArm(hardwareMap);
    }

    @Override
    public void loop() {
        settings.checkSettingsButton();
        settings.checkButton(settings.brushButton);
        settings.checkButton(settings.handButton);

        if (settings.settingsButton.onMode) {
            settings.changeSettings();
        } else {
            doMovement();
        }
        telemetry.update();
    }

    private void doMovement() {
        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = controllerInput.getLeftStickY();    // also negate the sign
        double rightStickX = controllerInput.getRightStickX();

        telemetry.addData("Left stick x\t", leftStickX);
        telemetry.addData("Left stick y\t", leftStickY);

        robotArm.toggleBrush(settings.brushButton.onMode);
        robotArm.toggleHand(settings.handButton.onMode);

        if (settings.fieldCentricMovement.onMode) {
            robotMove.fieldCentricMovement(leftStickX, leftStickY, rightStickX);
        } else {
            robotMove.robotCentricMovement(leftStickX, leftStickY, 0, rightStickX);
        }
    }
}
