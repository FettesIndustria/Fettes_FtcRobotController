package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotArm {

    private DcMotor motorArm;
    private Servo servoArm;

    public RobotArm(HardwareMap hardwareMap) {
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
    }
}


