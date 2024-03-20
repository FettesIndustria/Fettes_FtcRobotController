package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotExtras {
    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Telemetry telemetry;
    public Servo servoPixel;
    public DcMotor motorBrush;
    public static final double SERVO_PIXEL_CLOSED = 0.0;
    public static final double SERVO_PIXEL_OPEN = 0.055;
    public Button brushButton;
    private static final double BRUSH_POWER = 0.5;
    private ControllerInputHandler controllerInput;
    public RobotExtras(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        servoPixel = hardwareMap.get(Servo.class, "servoPixel");
        motorBrush = hardwareMap.get(DcMotor.class, "motorBrush");

        motorBrush.setDirection(DcMotorSimple.Direction.REVERSE);
        servoPixel.setPosition(SERVO_PIXEL_CLOSED);

        controllerInput = new ControllerInputHandler(gamepad);
        brushButton = new Button("leftstickbutton", false);
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void doHardwareMovement() {
        controllerInput.updateButton(brushButton);
        motorBrush.setPower(brushButton.onMode ? BRUSH_POWER : 0);
    }
}
