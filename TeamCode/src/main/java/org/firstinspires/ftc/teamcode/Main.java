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
        robotMove = new RobotMove(hardwareMap, gamepad1);
        settings = new SettingsManager(gamepad1, robotMove, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {
        manageButtons();

        if (settings.settingsButton.onMode) {
            settings.doSettings();
        } else {
            robotMove.doRobotMovement();
            robotArm.doArmMovement();
        }
        telemetry.update();
    }

    private void manageButtons() {
        // check settings button
        if (controllerInput.updateButton(settings.settingsButton)) {
            if (settings.settingsButton.onMode) {
                // in settings, print commands
                settings.printSettings();
            } else {
                // exited settings mode
            }
        }
    }
}
