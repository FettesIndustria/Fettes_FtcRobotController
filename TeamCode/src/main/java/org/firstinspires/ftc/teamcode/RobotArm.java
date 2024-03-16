package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotArm {
    private DcMotor motorArm, motorBrush;
    private Servo servoArm, servoHand;
    private Gamepad gamepad;
    private ControllerInputHandler controllerInput;
    private static final double ARM_POWER = 0.3;
    private static final double BRUSH_POWER = 0.5;
    private static final double HAND_ANGLE_INCREMENT = 0.075;
    private static final double CLOSED_POSITION = 0.0;
    public Button brushButton, handButton, handReleaseButton, motorArmUpButton, motorArmDownButton;
    public double handAngle;

    public RobotArm(HardwareMap hardwareMap, Gamepad gamepad) {
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorBrush = hardwareMap.get(DcMotor.class, "motorBrush");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");
        this.gamepad = gamepad;

        controllerInput = new ControllerInputHandler(gamepad);
        brushButton = new Button("leftstickbutton", false);
        handButton = new Button("triangle", false);
        handReleaseButton = new Button("circle", false);
        motorArmUpButton = new Button("leftbumper", false);
        motorArmDownButton = new Button("rightbumper", false);
        handAngle = 0.0;

        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrush.setDirection(DcMotorSimple.Direction.REVERSE);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoHand.setDirection(Servo.Direction.FORWARD);
        servoHand.setPosition(CLOSED_POSITION);
        servoArm.setPosition(CLOSED_POSITION);
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
            handAngle = CLOSED_POSITION;
            servoHand.setPosition(handAngle);
        }

        motorBrush.setPower(brushButton.onMode ? BRUSH_POWER : 0);
        motorArm.setPower(motorArmUpButton.isPressed ? ARM_POWER : (motorArmDownButton.isPressed ? -ARM_POWER : 0));
    }
}


