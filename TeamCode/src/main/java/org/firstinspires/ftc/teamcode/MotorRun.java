package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorRun extends LinearOpMode {
    public double power;
    public String direction;
    public DcMotor motor;

    public MotorRun(DcMotor motor, double power, String direction)
    {
        this.power = power;
        this.direction = direction;
        this.motor = motor;

    }
    public void setPower() {
        motor.setPower(power);
    }

    public void setDirection() {
        motor.setDirection(DcMotor.Direction.valueOf(direction));
    }


    @Override
    public void runOpMode() throws InterruptedException {
        System.out.println("MotorRun Failure");
    }
}
