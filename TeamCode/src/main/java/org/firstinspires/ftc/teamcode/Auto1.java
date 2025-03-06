package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;
/*
@Autonomous(name = "Auto1", group = "Autonomous")
public class Auto1 extends LinearOpMode {
    private ObjectDetector objectdetector123;
    private RobotMove robotMove;
    private RobotArm robotArm;
    private Gamepad gamepad;
    private RobotProcesses robotProcesses;
    private RobotExtras robotExtras;
    private static final double SERVO_PIXEL_CLOSED_POSITION = 0.3;
    private static final int MOTOR_ARM_DOWN_POSITION = -6;
    private static final int MOTOR_ARM_UP_POSITION = 20;

    @Override
    public void runOpMode() {
        robotMove = new RobotMove(hardwareMap, gamepad, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad, telemetry);
        robotProcesses = new RobotProcesses(robotMove, robotArm);
        robotExtras = new RobotExtras(hardwareMap, gamepad, telemetry);

        waitForStart();

        //awayBoardModified("blue", "middle");
        blueFront();


    }

    private void placePixel() {
        robotExtras.servoPixel.setPosition(robotExtras.SERVO_PIXEL_CLOSED);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            // Handle the interruption (e.g., log it or throw a new exception)
            //robotMove.robotCentricMovement(0, 0, 0, 0);
        }
        robotExtras.servoPixel.setPosition(robotExtras.SERVO_PIXEL_OPEN);
    }

    private int colourToSign(String colour) {
        switch (colour) {
            case "blue":
                return 1;
            case "red":
                return -1;
            default:
                return 0;
        }
    }


    private void blueFront() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        telemetry.addData("On left", "");
        robotProcesses.moveRobotTime(0, 0.8, 0.8);
        robotMove.robotCentricMovement(0, 0, 0, 0);
        // place pixel
        placePixel();

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        lowerArm();
        robotProcesses.moveRobotTime(0, -0.8, 0.8);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 1.6);
    }

    private void blueBack() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        telemetry.addData("On left", "");
        robotProcesses.moveRobotTime(0, 0.8, 0.8);
        robotMove.robotCentricMovement(0, 0, 0, 0);
        // place pixel
        placePixel();

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        lowerArm();
        robotProcesses.moveRobotTime(0, 0.8, 0.69);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 3.69);
    }

    private void redFront() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        telemetry.addData("On left", "");
        robotProcesses.moveRobotTime(0, 0.8, 0.8);
        robotMove.robotCentricMovement(0, 0, 0, 0);
        // place pixel
        placePixel();

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        lowerArm();
        robotProcesses.moveRobotTime(0, -0.8, 0.8);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 1.6);
    }

    private void redBack() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        telemetry.addData("On left", "");
        robotProcesses.moveRobotTime(0, 0.8, 0.8);
        robotMove.robotCentricMovement(0, 0, 0, 0);
        // place pixel
        placePixel();

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        lowerArm();
        robotProcesses.moveRobotTime(0, 0.8, 0.69);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 3.69);
    }

    private void lowerArm() {

        robotArm.motorArmLeft.setPower(0.2);
        robotArm.motorArmRight.setPower(0.2);

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robotArm.motorArmLeft.setPower(0);
        robotArm.motorArmRight.setPower(0);
    }

}*/