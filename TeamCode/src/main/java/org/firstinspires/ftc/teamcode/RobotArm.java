package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArm {
    public DcMotor motorArmLeft, motorArmRight;
    public Servo servoArm, servoHand;
    private Gamepad gamepad;
    private ControllerInputHandler controllerInput;
    private static final double ARM_POWER = 0.3;
    private static final double HAND_ANGLE_INCREMENT = 0.025;
    private static final double SERVO_ARM_ANGLE_INCREMENT = 0.025;
    public static final double SERVO_HAND_CLOSED = 0.0;
    public static final double SERVO_HAND_OPEN = 0.05;
    public static final double SERVO_ARM_DOWN = 0.55;
    public static final double SERVO_ARM_UP = 0.7;
    public static final double SERVO_ARM_BOARD = 0.0;
    public static final int MOTOR_ARM_DOWN = -105;
    public static final int MOTOR_ARM_UP = -9;
    public static final int MOTOR_ARM_BOARD = 10;
    private Telemetry telemetry;
    public Button handButton, handReleaseButton, motorArmUpButton, motorArmDownButton, servoArmUpButton, servoArmDownButton, armDownButton, armUpButton, armBoardButton;
    public double handAngle, servoArmAngle;

    public RobotArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");

        this.gamepad = gamepad;
        this.telemetry = telemetry;

        controllerInput = new ControllerInputHandler(gamepad);
        handButton = new Button("triangle", false);
        handReleaseButton = new Button("circle", false);
        motorArmUpButton = new Button("leftbumper", false);
        motorArmDownButton = new Button("rightbumper", false);
        servoArmUpButton = new Button("lefttrigger", false);
        servoArmDownButton = new Button("righttrigger", false);
        armDownButton = new Button("dpaddown", false);
        armUpButton = new Button("dpadup", false);
        armBoardButton = new Button("dpadleft", false);

        initialiseMotors();
        handAngle = 0;
        servoArmAngle = SERVO_ARM_DOWN;
    }

    private void initialiseMotors() {
        motorArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmRight.setDirection(DcMotorSimple.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoHand.setDirection(Servo.Direction.FORWARD);
        servoHand.setPosition(SERVO_HAND_OPEN);
        servoArm.setPosition(SERVO_ARM_DOWN);
        motorArmLeft.setTargetPosition(MOTOR_ARM_DOWN);
        motorArmRight.setTargetPosition(MOTOR_ARM_DOWN);
    }

    public void moveArmDown() {
        servoArm.setPosition(SERVO_ARM_DOWN);
        motorArmLeft.setTargetPosition(MOTOR_ARM_DOWN);
        motorArmRight.setTargetPosition(MOTOR_ARM_DOWN);
    }

    public void moveArmUp() {
        servoArm.setPosition(SERVO_ARM_UP);
        motorArmLeft.setTargetPosition(MOTOR_ARM_UP);
        motorArmRight.setTargetPosition(MOTOR_ARM_UP);
    }

    public void moveArmBoard() {
        servoArm.setPosition(SERVO_ARM_BOARD);
        motorArmLeft.setTargetPosition(MOTOR_ARM_BOARD);
        motorArmRight.setTargetPosition(MOTOR_ARM_BOARD);
    }

    public void doArmMovement() {
        // update arm buttons
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

        // arm movement buttons
        if (controllerInput.updateButton(armDownButton)) {
            telemetry.addData("Moving arm down", "");
            moveArmDown();
        }
        if (controllerInput.updateButton(armUpButton)) {
            telemetry.addData("Moving arm up", "");
            moveArmUp();
        }
        if (controllerInput.updateButton(armBoardButton)) {
            telemetry.addData("Moving arm to board", "");
            moveArmBoard();
        }

        servoHand.setPosition(handAngle);
        servoArm.setPosition(servoArmAngle);

        motorArmLeft.setPower(motorArmUpButton.isPressed ? ARM_POWER : (motorArmDownButton.isPressed ? -ARM_POWER : 0));
        motorArmRight.setPower(motorArmUpButton.isPressed ? ARM_POWER : (motorArmDownButton.isPressed ? -ARM_POWER : 0));
    }
}


