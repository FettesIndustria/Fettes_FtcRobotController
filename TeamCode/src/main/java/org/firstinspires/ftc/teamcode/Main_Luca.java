/*import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ControllerInputHandler;
import org.firstinspires.ftc.teamcode.MotorRun;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {

    private ControllerInputHandler controllerInput;
    private DcMotor coreHexMotor;
    private MotorRun coreHexMotorClass;
    private DcMotor hdHexMotor;
    private MotorRun hdHexMotorClass;


    @Override
    public void init() {
        /*controllerInput = new ControllerInputHandler(gamepad1);
        coreHexMotorClass = new MotorRun(coreHexMotor, 0, "forward"); // power, direction
        coreHexMotor = hardwareMap.get(DcMotor.class, "coreHexMotor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hdHexMotorClass = new MotorRun(hdHexMotor, 0, "forward");
        hdHexMotor = hardwareMap.get(DcMotor.class, "left");
        hdHexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    @Override
    public void loop() {
        boolean isButtonAPressed = controllerInput.isButtonPressed('a');
        boolean isButtonBPressed = controllerInput.isButtonPressed('b');
        boolean isButtonXPressed = controllerInput.isButtonPressed('x');
        boolean isButtonYPressed = controllerInput.isButtonPressed('y');

        //coreHexMotor.setPower(isButtonAPressed ? 0.5 : 0);
        hdHexMotor.setPower(isButtonBPressed ? 0.5 : 0);

        telemetry.addData("Button A Pressed ", false); // a pressed
        telemetry.addData("Button B Pressed ", false); // b pressed
        telemetry.update();
    }
}
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main_Luca", group = "TeleOp")
public class Main_Luca extends OpMode {
    private ControllerInputHandler controllerInput;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor hdHexMotor;

    public double speed = 0.5;
    public double turn_speed = 0.1;

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
        boolean isButtonAPressed = controllerInput.isButtonPressed('a');
        boolean isButtonBPressed = controllerInput.isButtonPressed('b');
        boolean isButtonXPressed = controllerInput.isButtonPressed('x');
        boolean isButtonYPressed = controllerInput.isButtonPressed('y');



        boolean leftBumper = controllerInput.leftBumper();
        boolean rightBumper = controllerInput.rightBumper();

        float leftStickX = controllerInput.getLeftStickX();
        float leftStickY = controllerInput.getLeftStickY();
        telemetry.addData("Left stick has X value:\t", leftStickX);
        telemetry.addData("Left stick has Y value:\t", leftStickY);

        // adjust speed
        if (leftBumper) {
            leftMotor.setPower(-turn_speed);
            rightMotor.setPower(turn_speed);
        }

        if (rightBumper) {
            leftMotor.setPower(turn_speed);
            rightMotor.setPower(-turn_speed);
        }

        // adjust direction
        speed = leftStickY;

        //rightMotor.setPower(isButtonXPressed ? speed : 0);
        //leftMotor.setPower(isButtonXPressed ? speed : 0);
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);

        telemetry.addData("Button A Pressed ", isButtonAPressed); // a pressed
        telemetry.addData("Button B Pressed ", isButtonBPressed); // b pressed
        telemetry.addData("The speed is:\t", speed);
        telemetry.update();
    }
}
