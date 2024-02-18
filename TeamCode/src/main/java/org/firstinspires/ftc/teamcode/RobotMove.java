package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RobotMove {
    private DcMotor motorA, motorB, motorC, motorD;
    private static final double S = 0.98;    // set max speed to S
    private static final double T = 0.25;    // turning scalar (can be adjusted)
    private BNO055IMU imu; // Assuming BNO055IMU is the IMU class

    private Orientation defaultOrientation;

    public RobotMove(HardwareMap hardwareMap) {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");

        initializeMotors();
        defaultOrientation = getIMUOrientation(); // Initialize defaultOrientation

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize IMU parameters
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParameters);
    }

    private void initializeMotors() {
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);

        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorC.setDirection(DcMotorSimple.Direction.FORWARD);

        motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorD.setDirection(DcMotorSimple.Direction.REVERSE);
    }


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

    // sets the motors to move orthogonally at some angle and power value while turning with speed turn_value
    public void robot_centric_movement(double theta, double power, double turn_value) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(sin, cos);

        // orthogonal movement (normalised sin and cos to make use of all available power)
        // multiply by S to account for any variation in motor strength (2% assumed)
        double speed_a = power * sin/max * S;
        double speed_b = power * cos/max * S;
        double speed_c = power * cos/max * S;
        double speed_d = power * sin/max * S;

        // add turning
        speed_a += turn_value * T;
        speed_b -= turn_value * T;
        speed_c += turn_value * T;
        speed_d -= turn_value * T;

        // account for any power overshooting
        if ((power * S + Math.abs(turn_value)) > S) {
            speed_a /= (power + turn_value);
            speed_b /= (power + turn_value);
            speed_c /= (power + turn_value);
            speed_d /= (power + turn_value);
        }

        // set motor speeds
        motorA.setPower(speed_a);
        motorB.setPower(speed_b);
        motorC.setPower(speed_c);
        motorD.setPower(speed_d);
    }

    // the same as robot centric movement except controls work relative to the field instead of the robot
    public void field_centric_movement(double theta, double power, double turn_value) {
        // get orientation of the robot relative to the field using IMU
        Orientation currentOrientation = getIMUOrientation();
        double deltaAngle = currentOrientation.firstAngle - defaultOrientation.firstAngle;

        // do movement with new angle
        robot_centric_movement(theta - deltaAngle, power, turn_value);
    }

    private void setDefaultOrientation() {
        defaultOrientation = getIMUOrientation();
    }

    // gets the current orientation of the robot
    private Orientation getIMUOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}


