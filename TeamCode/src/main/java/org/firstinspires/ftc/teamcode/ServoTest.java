package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
public class ServoTest extends OpMode {

    private Servo ;
    private static final double MOTOR_POWER = 0.8;
    private ControllerInputHandler controllerInput;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);
    }
    @Override
    public void loop() {
        String[] buttons = {"cross", "circle", "triangle", "square"};
        char[] motors = {'A', 'B', 'C', 'D'};

        for (int i = 0; i < 4; i++) {
            if (controllerInput.isButtonPressed(buttons[i])) {
                robotMove.setPower(motors[i], MOTOR_POWER);
                telemetry.addData("Button", "Button " + buttons[i] + " is pressed");
                telemetry.addData("Motor", "Motor " + motors[i] + " is moving");
            } else {
                robotMove.setPower(motors[i], 0);
            }
        }

        telemetry.update();
    }
}
