package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotArm {

    private DcMotor motorArm, motorBrush;
    private Servo servoArm, servoHand;
    private static final double MAX_ARM_POWER = 0.4;
    private static final double MAX_HAND_POWER = 0.5;

    private static final double BRUSH_POWER = 0.5;

    public RobotArm(HardwareMap hardwareMap) {
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorBrush = hardwareMap.get(DcMotor.class, "motorBrush");
        //servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");
        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrush.setDirection(DcMotorSimple.Direction.REVERSE);
        //servoArm.setDirection(Servo.Direction.FORWARD);
        servoHand.setDirection(Servo.Direction.FORWARD);
    }

    public void armMove(double power) {
        //motorArm.setPower(power * MAX_ARM_POWER);
    }

    public void toggleBrush(boolean mode) {
        motorBrush.setPower(mode ? BRUSH_POWER : 0);
    }

    public void toggleHand(boolean mode) {
        handMove(mode ? 30 : 0);
    }

    public void handMove(double pos) {
        servoHand.setPosition(pos);
    }
}


