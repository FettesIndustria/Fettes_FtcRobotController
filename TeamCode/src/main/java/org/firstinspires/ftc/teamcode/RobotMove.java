package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMove {
    private final DcMotor motorA, motorB, motorC, motorD;
    private static final double MAX_AVAILABLE_POWER = 0.98;   // 2% reduction in max power
    private static final double MAX_MOTOR_POWER = 0.9 * MAX_AVAILABLE_POWER;   // don't use all available power (too sensitive)
    private static final double TURN_SCALAR = 0.6;    // turning scalar (can be adjusted)
    private BHI260IMU bhi260; // Assuming BHI260IMU is the IMU class8
    private Orientation defaultOrientation;

    public RobotMove(HardwareMap hardwareMap) {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");

        initializeMotors();
        defaultOrientation = getIMUOrientation(); // Initialize defaultOrientation

        // Initialize IMU
        //bhi260 = new BHI260IMU(hardwareMap.i2cDeviceSynch.get("imu"), true);
    }

    private void initializeMotors() {
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);

        motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC.setDirection(DcMotorSimple.Direction.REVERSE);

        motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorD.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // gives power to any wheel motor
    public void setPower(char motor, double power) {
        switch (motor) {
            case 'A':
                motorA.setPower(power);
                break;
            case 'B':
                motorB.setPower(power);
                break;
            case 'C':
                motorC.setPower(power);
                break;
            case 'D':
                motorD.setPower(power);
                break;
        }
    }

    // converts joystick coords to an angle
    private double xy_to_angle(double x, double y) {
        if (x >= 0 && y == 0) return 0.0;
        if (x == 0 && y > 0) return Math.PI / 2;
        if (x < 0 && y == 0) return Math.PI;
        if (x == 0 && y < 0) return 3 * Math.PI / 2;
        return Math.atan2(y, x);
    }

    // sets the motors to move orthogonally at some angle and power value while turning with speed turn_value
    public void robotCentricMovement(double x, double y, double offset_angle, double turn_value) {
        double theta = xy_to_angle(x, y) - offset_angle;
        double power = Math.sqrt(x*x + y*y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // orthogonal movement (normalised sin and cos to make use of all available power)
        // multiply by MAX_MOTOR_POWER to account for any variation in motor strength (2% assumed) + sensitivity
        double speed_a = power * cos/max * MAX_MOTOR_POWER;
        double speed_b = power * sin/max * MAX_MOTOR_POWER;
        double speed_c = power * sin/max * MAX_MOTOR_POWER;
        double speed_d = power * cos/max * MAX_MOTOR_POWER;

        // add turning
        if (y < 0) turn_value = -turn_value;
        speed_a += turn_value * TURN_SCALAR;
        speed_b -= turn_value * TURN_SCALAR;
        speed_c += turn_value * TURN_SCALAR;
        speed_d -= turn_value * TURN_SCALAR;

        // account for any power overshooting
        if ((power * MAX_MOTOR_POWER + Math.abs(turn_value)) > MAX_MOTOR_POWER) {
            speed_a /= (power + Math.abs(turn_value));
            speed_b /= (power + Math.abs(turn_value));
            speed_c /= (power + Math.abs(turn_value));
            speed_d /= (power + Math.abs(turn_value));
        }

        // set motor speeds
        motorA.setPower(speed_a);
        motorB.setPower(speed_b);
        motorC.setPower(speed_c);
        motorD.setPower(speed_d);
    }

    // the same as robot centric movement except controls work relative to the field instead of the robot
    public void fieldCentricMovement(double x, double y, double turn_value) {
        // get orientation of the robot relative to the field using IMU
        Orientation currentOrientation = getIMUOrientation();
        double deltaAngle = currentOrientation.firstAngle - defaultOrientation.firstAngle;

        // do movement with new angle
        robotCentricMovement(x, y, deltaAngle, turn_value);
    }

    public void setDefaultOrientation() {
        defaultOrientation = getIMUOrientation();
    }

    // gets the current orientation of the robot
    private Orientation getIMUOrientation() {
        //return bhi260.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return new Orientation();
    }
}


