package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
    private RobotMove robotMove;
    private RobotArm robotArm;
    private Gamepad gamepad;
    private RobotProcesses robotProcesses;
    private Servo servoPixel;
    private static double SERVO_PIXEL_CLOSED_POSITION = 0.3;

    @Override
    public void runOpMode() {

        robotMove = new RobotMove(hardwareMap, gamepad, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad, telemetry);
        robotProcesses = new RobotProcesses(robotMove, robotArm);
        servoPixel = hardwareMap.get(Servo.class, "servoPixel");
        servoPixel.setPosition(SERVO_PIXEL_CLOSED_POSITION);

        waitForStart();

        nearBoard("blue");
        //feedbackPositions();


        /*
        // Load OpenCV library and initialize object detector
        this.objectDetector = new CascadeClassifier("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/cascade.xml");
        this.camera = new VideoCapture(0);

        // Assuming you've configured the motor in the robot configuration file
        //blue(true);
        //red(true);
        waitForStart();

        if (!camera.isOpened()) {
            telemetry.addData("Error opening camera", 1);
            telemetry.update();

            return;
        }

        telemetry.addData("Has started now", "");
        telemetry.update();

        while (opModeIsActive()) {
            feedbackPositions(); // Perform initial feedback

            sleep(10000);
        }
        //drive(); // Execute autonomous actions

        // Cleanup resources
        camera.release();
        objectDetector = null;
         */
    }

    private void feedbackPositions() {
        for (int i = 0; i < 1000; i++) {
            telemetry.addData("Motor arm left:", robotArm.motorArmLeft.getCurrentPosition());
            telemetry.addData("Motor arm right:", robotArm.motorArmLeft.getCurrentPosition());
            telemetry.addData("Servo arm:", robotArm.servoArm.getPosition());
            telemetry.addData("Servo hand:", robotArm.servoHand.getPosition());
            telemetry.addData("Servo pixel:", servoPixel.getPosition());
            telemetry.update();

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // Handle the interruption (e.g., log it or throw a new exception)
                //robotMove.robotCentricMovement(0, 0, 0, 0);
            }
        }
    }

    private double turnCheckPixel(Orientation initialOrientation) {
        // turn left
        for (int i = 0; i < 5; i++) {
            //if (detectFrame()) return robotMove.getIMUOrientation().firstAngle - initialOrientation.firstAngle;
            robotProcesses.turnToOrientation(initialOrientation.firstAngle + (Math.PI / 30) * i, 0.2);
        }

        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 0.4);

        // turn right
        for (int i = 0; i < 5; i++) {
            //if (detectFrame()) return robotMove.getIMUOrientation().firstAngle - initialOrientation.firstAngle;
            robotProcesses.turnToOrientation(initialOrientation.firstAngle - (Math.PI / 20) * i, 0.2);
        }

        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 0.4);
        return 0;
    }

    private void placePixel() {
        servoPixel.setPosition(SERVO_PIXEL_CLOSED_POSITION - 0.1);
        servoPixel.setPosition(SERVO_PIXEL_CLOSED_POSITION);
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

    private void awayBoard(String colour) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        int sign = colourToSign(colour);

        // turn left
        // then move under truss
        // then turn right again
        robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI/2 * sign, 1);
        robotProcesses.moveRobotTime(0, 1, 2.5);
        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);

        nearBoard(colour);
    }

    private void nearBoard(String colour) {
        Orientation initialOrientation = robotMove.getIMUOrientation();
        int sign = colourToSign(colour);

        // move forward a little bit
        robotProcesses.moveRobotTime(0, 1, 0.4);

        double deltaAngle = turnCheckPixel(initialOrientation);
        deltaAngle = robotMove.angleToRange(deltaAngle);
        telemetry.clearAll();
        telemetry.addData("Robot has turned initally", "");
        telemetry.addData("Delta angle is", deltaAngle);

        if (deltaAngle > Math.PI/18) {
            // on left
            robotProcesses.moveRobotTime(0, 0.4, 0.3);
            // place pixel
            placePixel();
            robotProcesses.moveRobotTime(0, -0.4, 0.3);
        } else if (deltaAngle < -Math.PI/18) {
            // on right
            robotProcesses.moveRobotTime(0, 0.4, 0.3);
            robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI, 1.5);
            // place pixel
            placePixel();
            robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1.5);
            robotProcesses.moveRobotTime(0, -0.4, 0.3);
        } else {
            // in middle
            telemetry.addData("In middle", "");
            robotProcesses.moveRobotTime(0, 0.4, 0.5);
            robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
            // place pixel
            placePixel();
            robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);
            robotProcesses.moveRobotTime(0, -0.4, 0.5);
            telemetry.addData("Turned and placed pixel", "");
        }

        // move back a little bit
        robotProcesses.moveRobotTime(0, -1, 0.4);

        // move towards board and face away from it
        robotProcesses.moveRobotTime(-3 / Math.sqrt(13) * sign, 2 / Math.sqrt(13), 4.8);
        robotProcesses.turnToOrientation(robotMove.angleToRange(initialOrientation.firstAngle - Math.PI/2 * sign), 1.5);

        // place pixels on board

    }

    /*private boolean detectFrame() {
        final AtomicBoolean detectionResult = new AtomicBoolean(false);

        new Thread(() -> {
            Mat frameMat = new Mat();
            MatOfRect objectDetections = new MatOfRect();

            // read camera frame
            camera.read(frameMat);

            if (!frameMat.empty()) {
                objectDetector.detectMultiScale(frameMat, objectDetections);
                detectionResult.set(objectDetections.toArray().length > 0);

                // Delay to avoid excessive processing
                try {
                    TimeUnit.MILLISECONDS.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                System.out.println("Error: Frame is empty.");
            }

            // Release resources
            camera.release();
            objectDetector = null;

        }).start();
        return detectionResult.get();
    }*/
}