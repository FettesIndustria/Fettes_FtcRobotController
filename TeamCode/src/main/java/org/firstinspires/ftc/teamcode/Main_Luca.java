package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main_Luca", group = "TeleOp")
public class Main_Luca extends OpMode {
    private ControllerInputHandler controllerInput;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor hdHexMotor;

    public double turn_speed = 0.1; // constant turn speed, later maybe control using triggers as opposed to bumpers

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        /*
        coreHexMotorClass = new MotorRun(coreHexMotor, 0, "forward"); // power, direction
        coreHexMotor = hardwareMap.get(DcMotor.class, "coreHexMotor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);*/

        leftMotor = hardwareMap.get(DcMotor.class, "left");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = -controllerInput.getLeftStickY();    // also negate the sign
        telemetry.addData("Left stick has X value:\t", leftStickX);
        telemetry.addData("Left stick has Y value:\t", leftStickY);

        double sqrt2 = Math.sqrt(2);
        double s = 0.5;  // sensitivity constant

        // use speed equations for AD and BC motors
        double v_ad = (s / sqrt2) * (leftStickY + leftStickX);
        double v_bc = (s / sqrt2) * (leftStickY - leftStickX);

        // set motor speeds
        // motors not defined yet



        boolean leftBumper = controllerInput.leftBumper();
        boolean rightBumper = controllerInput.rightBumper();

        // set turn speeds (turning has higher priority than movement)
        if (leftBumper) {
            leftMotor.setPower(-turn_speed);
            rightMotor.setPower(turn_speed);
        }

        if (rightBumper) {
            leftMotor.setPower(turn_speed);
            rightMotor.setPower(-turn_speed);
        }

        // write to/update telemetry
        telemetry.addData("AD motor speeds:\t", v_ad);
        telemetry.addData("BC motor speeds:\t", v_bc);
        telemetry.update();
    }
}
