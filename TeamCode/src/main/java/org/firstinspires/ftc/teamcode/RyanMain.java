package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RyanMain", group = "TeleOp")
public class RyanMain extends OpMode {
    private ControllerInputHandler controllerInput;
    private DcMotor leftMotorfront;
    private DcMotor rightMotorfront;
    private DcMotor leftMotorback;
    private DcMotor rightMotorback;
    private DcMotor hdHexMotor;


    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        /*
        coreHexMotorClass = new MotorRun(coreHexMotor, 0, "forward"); // power, direction
        coreHexMotor = hardwareMap.get(DcMotor.class, "coreHexMotor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);*/

        leftMotorfront = hardwareMap.get(DcMotor.class, "left");
        leftMotorfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorfront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotorfront = hardwareMap.get(DcMotor.class, "right");
        rightMotorfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorfront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotorback = hardwareMap.get(DcMotor.class, "left");
        leftMotorback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorback.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotorback = hardwareMap.get(DcMotor.class, "right");
        rightMotorback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorback.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        float leftStickX = controllerInput.getLeftStickX();
        float leftStickY = controllerInput.getLeftStickY();
        telemetry.addData("Left stick has X value:\t", leftStickX);
        telemetry.addData("Left stick has Y value:\t", leftStickY);
        double theta = atan(leftStickY/leftStickX);

        //positive between 0 and pi
        if(theta>=0 && theta<(PI/2)){
            telemetry.addData("rightup", theta);
            telemetry.addData("BC: ", (sin((2*theta)-(PI/2))));
            telemetry.addData("AD: ", (cos((2*theta)-(PI/2))));
        }
        if(theta>(PI/2) && theta<(PI)){
            telemetry.addData("leftdown", theta);
            telemetry.addData("BC: ", -(sin((2*theta)+(PI/2))));
            telemetry.addData("AD: ", -(cos((2*theta)+(PI/2))));
        }


        if(theta<0 && theta>(-PI/2)){
            telemetry.addData("leftup", theta);
            telemetry.addData("AD: ", -(sin((2*theta)-(PI/2))));
            telemetry.addData("BC: ", -(cos((2*theta)-(PI/2))));
        }
        if(theta<(-PI/2) && theta>(-PI)){
            telemetry.addData("rightdown", theta);
            telemetry.addData("AD: ", -(sin((2*theta)+(PI/2))));
            telemetry.addData("BC: ", -(cos((2*theta)+(PI/2))));
        }



    }
    /*public void forward()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }

    public void moveright()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(speed);
    }

    public void moveleft()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(-speed);
    }
    public void backward()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }

    public void diagonalfright()
    {
        leftMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }
    public void diagonalfleft()
    {
        rightMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
    }
    public void diagonalbright()
    {
        leftMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }
    public void diagonalbleft()
    {
        rightMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
    }

    public void backturn()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }
    public void leftturn()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }
    public void rightturn()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }*/

}


