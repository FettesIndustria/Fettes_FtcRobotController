package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArm {
    public DcMotor motorArmLeft, motorArmRight, motorBrush;
    public Servo servoArm, servoHand;
    private Gamepad gamepad;
    private ControllerInputHandler controllerInput;
    private static final double ARM_POWER = 0.3;
    private static final double BRUSH_POWER = 0.5;
    private static final double HAND_ANGLE_INCREMENT = 0.05;
    private static final double SERVO_ARM_ANGLE_INCREMENT = 0.05;
    private double SERVO_HAND_START;
    private double SERVO_ARM_START;
    private Telemetry telemetry;
    public Button brushButton, handButton, handReleaseButton, motorArmUpButton, motorArmDownButton, servoArmUpButton, servoArmDownButton;
    public double handAngle, servoArmAngle;

    public RobotArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        motorBrush = hardwareMap.get(DcMotor.class, "motorBrush");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        controllerInput = new ControllerInputHandler(gamepad);
        brushButton = new Button("leftstickbutton", false);
        handButton = new Button("triangle", false);
        handReleaseButton = new Button("circle", false);
        motorArmUpButton = new Button("leftbumper", false);
        motorArmDownButton = new Button("rightbumper", false);
        servoArmUpButton = new Button("lefttrigger", false);
        servoArmDownButton = new Button("righttrigger", false);

        initialiseMotors();
        SERVO_HAND_START = servoHand.getPosition();
        handAngle = SERVO_HAND_START;

        SERVO_ARM_START = servoArm.getPosition();
        servoArmAngle = SERVO_ARM_START;
    }

    private void initialiseMotors() {
        motorArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrush.setDirection(DcMotorSimple.Direction.REVERSE);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoHand.setDirection(Servo.Direction.FORWARD);
        servoHand.setPosition(SERVO_HAND_START);
        servoArm.setPosition(SERVO_ARM_START);
    }

    public void doArmMovement() {
        // update arm buttons
        controllerInput.updateButton(brushButton);
        controllerInput.updateButton(motorArmUpButton);
        controllerInput.updateButton(motorArmDownButton);

        // hand buttons
        if (controllerInput.updateButton(handButton)) {
            handAngle += HAND_ANGLE_INCREMENT;
            servoHand.setPosition(handAngle);
        }
        if (controllerInput.updateButton(handReleaseButton)) {
            handAngle -= HAND_ANGLE_INCREMENT;
            servoHand.setPosition(handAngle);
        }

        // servo arm buttons
        if (controllerInput.updateButton(servoArmUpButton)) {
            servoArmAngle += SERVO_ARM_ANGLE_INCREMENT;
            servoArm.setPosition(servoArmAngle);
        }
        if (controllerInput.updateButton(servoArmDownButton)) {
            servoArmAngle -= SERVO_ARM_ANGLE_INCREMENT;
            servoArm.setPosition(servoArmAngle);
        }

        servoHand.setPosition(handAngle);
        servoArm.setPosition(servoArmAngle);

        telemetry.addData("hand angle", handAngle);
        telemetry.addData("servo arm angle", servoArmAngle);
        telemetry.addData("motor arm left:", motorArmLeft.getCurrentPosition());
        telemetry.addData("motor arm right:", motorArmRight.getCurrentPosition());

        motorBrush.setPower(brushButton.onMode ? BRUSH_POWER : 0);
        motorArmLeft.setPower(motorArmUpButton.isPressed ? ARM_POWER : (motorArmDownButton.isPressed ? -ARM_POWER : 0));
        motorArmRight.setPower(motorArmUpButton.isPressed ? ARM_POWER : (motorArmDownButton.isPressed ? -ARM_POWER : 0));
    }
}


