package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RyanMain", group = "TeleOp")
public class RyanMain extends OpMode {
    private ControllerInputHandler controllerInput;
    private DcMotor motorA, motorB, motorC, motorD;
    private DcMotor motorArm;
    private Servo servoArm;


    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);

        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);

        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoArm.setDirection(Servo.Direction.FORWARD);

        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);

        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC.setDirection(DcMotorSimple.Direction.FORWARD);

        motorD = hardwareMap.get(DcMotor.class, "motorD");
        motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorD.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void orthogonal_movement(double speed_ad, double speed_bc) {
        motorA.setPower(speed_ad);
        motorD.setPower(speed_ad);
        motorB.setPower(speed_bc);
        motorC.setPower(speed_bc);
    }

    public void pivot_left(double turn_speed) {
        motorA.setPower(-turn_speed);
        motorC.setPower(-turn_speed);
        motorB.setPower(turn_speed);
        motorD.setPower(turn_speed);
    }

    public void pivot_right(double turn_speed) {
        motorA.setPower(turn_speed);
        motorC.setPower(turn_speed);
        motorB.setPower(-turn_speed);
        motorD.setPower(-turn_speed);
    }

    @Override
    public void loop() {
        double sqrt2 = Math.sqrt(2);
        double turn_speed = 0.1;
        double s = 0.5;  // sensitivity constant

        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = -controllerInput.getLeftStickY();    // also negate the sign
        telemetry.addData("\nLeft stick has X value:\t", leftStickX);
        telemetry.addData("Left stick has Y value:\t", leftStickY);

        // use speed equations for AD and BC motors
        double speed_ad = -(s / sqrt2) * (leftStickY + leftStickX);
        double speed_bc = -(s / sqrt2) * (leftStickY - leftStickX);
        orthogonal_movement(speed_ad, speed_bc);

        // set turn speeds (turning has higher priority than movement)
        boolean leftBumper = controllerInput.leftBumper();
        boolean rightBumper = controllerInput.rightBumper();

        if (leftBumper) pivot_left(turn_speed);
        if (rightBumper) pivot_right(turn_speed);

        // write to/update telemetry
        telemetry.addData("AD motor speeds:\t", speed_ad);
        telemetry.addData("BC motor speeds:\t", speed_bc);
        telemetry.update();
    }
}


