package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
public class MotorTest extends OpMode {

    private RobotMove robotMove;
    private static final double MOTOR_POWER = 0.8;
    private ControllerInputHandler controllerInput;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap);
    }
    @Override
    public void loop() {
        char[] buttons = {'a', 'b', 'x', 'y'};
        char[] motors = {'A', 'B', 'C', 'D'};

        for (int i = 0; i < 4; i++) {
            if (controllerInput.isButtonPressed(buttons[i])) {
                robotMove.setPower(motors[i], MOTOR_POWER);
                telemetry.addData("Button", "Button " + buttons[i] + " is pressed");
                telemetry.addData("Motor", "Motor " + motors[i] + " is moving");
            }
        }

        telemetry.update();
    }
}
