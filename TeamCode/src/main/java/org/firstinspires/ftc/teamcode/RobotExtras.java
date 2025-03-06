package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotExtras {
    public DcMotor motorPulley, motorArm;
    public Servo servoHand;
    private Gamepad gamepad;
    private ControllerInputHandler controllerInput;
    private static final double MOTOR_ARM_POWER = 0.2;
    private static final double MOTOR_PULLEY_POWER = 0.5;
    private static final double SERVO_HAND_ANGLE_INCREMENT = 0.025;
    private Telemetry telemetry;
    public Button armUpButton, armDownButton, pulleyUpButton, pulleyDownButton, servoHandOpenButton, servoHandCloseButton;
    public double servoHandAngle;

    public RobotExtras(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorPulley = hardwareMap.get(DcMotor.class, "motorPulley");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");

        this.gamepad = gamepad;
        this.telemetry = telemetry;

        controllerInput = new ControllerInputHandler(gamepad);
        armUpButton = new Button("lefttrigger", false);
        armDownButton = new Button("righttrigger", false);
        pulleyUpButton = new Button("dpadup", false);
        pulleyDownButton = new Button("dpaddown", false);
        servoHandOpenButton = new Button("leftbumper", false);
        servoHandCloseButton = new Button("rightbumper", false);

        initialiseMotors();
        servoHandAngle = 0;
    }

    private void initialiseMotors() {
        motorPulley.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);

        motorPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorPulley.setPower(MOTOR_ARM_POWER);
        motorArm.setPower(MOTOR_PULLEY_POWER);

        servoHand.setDirection(Servo.Direction.FORWARD);
    }

    public void openHand() {
        servoHandAngle += SERVO_HAND_ANGLE_INCREMENT;
        servoHand.setPosition(servoHandAngle);
    }
    public void closeHand() {
        servoHandAngle -= SERVO_HAND_ANGLE_INCREMENT;
        servoHand.setPosition(servoHandAngle);
    }

    public void doHardwareMovement() {
        // update arm buttons
        controllerInput.updateButton(armUpButton);
        controllerInput.updateButton(armDownButton);

        // update pulley buttons
        controllerInput.updateButton(pulleyUpButton);
        controllerInput.updateButton(pulleyDownButton);

        // servo hand buttons
        if (controllerInput.updateButton(servoHandOpenButton)) {
            openHand();
            telemetry.addData("hand opening", "");
        }
        if (controllerInput.updateButton(servoHandCloseButton)) {
            closeHand();
            telemetry.addData("hand closing", "");
        }

        motorPulley.setPower(pulleyUpButton.isPressed ? MOTOR_PULLEY_POWER : (pulleyDownButton.isPressed ? -MOTOR_PULLEY_POWER : 0));
        motorArm.setPower(armUpButton.isPressed ? MOTOR_ARM_POWER : (armDownButton.isPressed ? -MOTOR_ARM_POWER : 0));
    }
}
