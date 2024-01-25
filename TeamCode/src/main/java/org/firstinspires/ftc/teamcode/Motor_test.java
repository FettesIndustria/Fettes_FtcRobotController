package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main_test", group = "TeleOp")
public class Motor_test extends OpMode {
    private ControllerInputHandler controllerInput;
    private MotorRun coreHexMotorClass;
    private DcMotor coreHexMotor;

    private Servo myServo;

    public double turn_speed = 0.1; // constant turn speed, later maybe control using triggers as opposed to bumpers

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        coreHexMotorClass = new MotorRun(coreHexMotor, 0, "forward"); // power, direction
        coreHexMotor = hardwareMap.get(DcMotor.class, "coreHexMotor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        myServo = hardwareMap.get(Servo.class, "servo");
        myServo.setPosition(0.5);
    }
    @Override
    public void loop() {
        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = -controllerInput.getLeftStickY();    // also negate the sign
        double rightStickX = controllerInput.getRightStickX();
        double rightStickY = -controllerInput.getRightStickY();    // also negate the sign
        telemetry.addData("Left stick has X value:\t", leftStickX);
        telemetry.addData("Left stick has Y value:\t", leftStickY);

        if (leftStickY > 0) {
            myServo.setPosition(1);
        } else if (leftStickY < 0) {
            myServo.setPosition(0);
        }

        /*if (rightStickY > 0) {
            coreHexMotor.setPower(turn_speed);
        } else if (rightStickY < 0) {
            coreHexMotor.setPower(-turn_speed);
        }*/

        telemetry.update();
    }
}
