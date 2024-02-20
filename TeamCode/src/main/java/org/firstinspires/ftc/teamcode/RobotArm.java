package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotArm {

    private DcMotor motorArm;
    private Servo servoArm;
    private static final double MAX_ARM_POWER = 0.4;
    private static final double MAX_HAND_POWER = 5;

    public RobotArm(HardwareMap hardwareMap) {
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
    }

    public void armMove(double power) {
        motorArm.setPower(power * MAX_ARM_POWER);
    }

    public void handMove(double power) {
        double pos = servoArm.getPosition();
        servoArm.setPosition(pos * (1 + MAX_ARM_POWER * power));
    }
}


