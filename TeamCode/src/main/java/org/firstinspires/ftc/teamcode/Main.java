package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {

    private ControllerInputHandler controllerInput;
    private RobotMove robotMove;
    private SettingsManager settings;

    //private RobotArm robotArm;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap);
        settings = new SettingsManager(gamepad1, robotMove, telemetry);
        //robotArm = new RobotArm(hardwareMap);
    }

    @Override
    public void loop() {
        settings.checkSettingsButton();
        if (settings.settingsButton.onMode) {
            settings.changeSettings();
        } else {
            doMovement();
        }
        telemetry.update();
    }

    private void doMovement() {
        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = -controllerInput.getLeftStickY();    // also negate the sign
        double rightStickX = controllerInput.getRightStickX();

        double theta = Math.atan2(leftStickY, leftStickX);
        double power = Math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY);

        telemetry.addData("Theta:\t", theta);
        telemetry.addData("Power:\t", power);
        telemetry.addData("Turn value:\t", rightStickX);

        if (settings.fieldCentricMovement.onMode) {
            robotMove.fieldCentricMovement(theta, power, rightStickX);
        } else {
            robotMove.robotCentricMovement(theta, power, rightStickX);
        }
    }
}
