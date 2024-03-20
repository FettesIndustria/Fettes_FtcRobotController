package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
        lowerArm();

        waitForStart();

        //runMode("red", "front", "left");
        blueBackModified("left");

    }

    private void runMode(String colour, String position, String block) {
        switch (colour) {
            case "blue":
                switch (position) {
                    case "front":
                        blueFront(block);
                        break;
                    case "back":
                        blueBack(block);
                        break;
                }
                break;
            case "red":
                switch (position) {
                    case "front":
                        redFront(block);
                        break;
                    case "back":
                        redBack(block);
                        break;
                }
                break;
        }
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

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void placePixelMove(String block, Orientation initialOrientation) {
        if (block == "left") {
            telemetry.addData("On left", "");
            placePixel();
        } else if (block == "right") {
            telemetry.addData("On right", "");
            robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI, 1.5);
            placePixel();
            robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1.5);
        } else {
            telemetry.addData("In middle", "");
            robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI / 2, 1);
            placePixel();
            robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);
        }
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


    private void blueFront(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.1);

        placePixelMove(block, initialOrientation);

        robotProcesses.moveRobotTime(0, -0.8, 1.1);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI / 2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 1.6);
    }

    private void blueFrontModified(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.1);

        robotProcesses.moveRobotTime(0, -0.8, 1.1);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI / 2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 0.8);
        robotMove.robotCentricMovement(0, 0, 0, 0);

        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);
        robotProcesses.moveRobotTime(0, 1, 0.9);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
        robotProcesses.moveRobotTime(0, -1, 0.6);
    }

    private void blueBack(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.1);
        robotMove.robotCentricMovement(0, 0, 0, 0);

        placePixelMove(block, initialOrientation);

        robotProcesses.moveRobotTime(0, 0.8, 0.69);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 3.69);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    private void blueBackModified(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.73);
        robotMove.robotCentricMovement(0, 0, 0, 0);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 2.9);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);
        placePixel();
    }

    private void redFront(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.1);

        placePixelMove(block, initialOrientation);

        robotProcesses.moveRobotTime(0, -0.8, 1.1);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI / 2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 1.6);
    }

    private void redBack(String block) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        robotProcesses.moveRobotTime(0, 0.8, 1.1);
        robotMove.robotCentricMovement(0, 0, 0, 0);

        placePixelMove(block, initialOrientation);

        robotProcesses.moveRobotTime(0, 0.8, 0.69);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI/2, 1);
        telemetry.addData("Turned and placed pixel", "");

        // move to taped area
        robotProcesses.moveRobotTime(0, -1, 3.69);
    }

    private void lowerArm() {

        robotArm.motorArmLeft.setPower(0.3);
        robotArm.motorArmRight.setPower(0.3);

        try {
            TimeUnit.MILLISECONDS.sleep(780);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robotArm.motorArmLeft.setPower(0);
        robotArm.motorArmRight.setPower(0);
    }

}