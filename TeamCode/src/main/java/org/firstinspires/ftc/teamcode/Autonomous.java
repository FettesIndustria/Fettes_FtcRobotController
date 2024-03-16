package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Autonomous", group = "TeleOp")
public class Autonomous extends OpMode {
    //private CascadeClassifier objectDetector;
    //private VideoCapture camera;
    private RobotMove robotMove;
    private RobotArm robotArm;
    private Gamepad gamepad;
    private RobotProcesses robotProcesses;
    private boolean objectDetected;
    private static boolean nearboard;
    private static boolean isBlue;
    private static final double TURN_DURATION = 1.0;
    private static final double TWO_PI = 2 * Math.PI;
    private Servo servoPixel;
    private static final double CLOSED_POSITION = 0.0;

    public Autonomous() {
        // Load OpenCV library and initialize object detector
        //System.load("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opencv_java2413.dll");
        /*this.objectDetector = new CascadeClassifier("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/cascade.xml");
        this.camera = new VideoCapture(0);
        nearboard = true;
        if (!camera.isOpened()) {
            System.out.println("Error: Camera not opened.");
            return;
        }*/

        // Assuming you've configured the motor in the robot configuration file
        red();
        //blue();
    }

    @Override
    public void init() {
        robotMove = new RobotMove(hardwareMap, gamepad, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad);
        robotProcesses = new RobotProcesses(robotMove, robotArm);
        servoPixel = hardwareMap.get(Servo.class, "servoPixel");
    }

    private double turnCheckPixel(Orientation initialOrientation) {
        // turn left
        for (int i = 0; i < 5; i++) {
            //if (detectFrame()) return robotMove.getIMUOrientation().firstAngle - initialOrientation.firstAngle;
            robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI / 100, 1/5.0);
        }

        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 0.5);

        // turn right
        for (int i = 0; i < 5; i++) {
            //if (detectFrame()) return robotMove.getIMUOrientation().firstAngle - initialOrientation.firstAngle;
            robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI / 100, 1/5.0);
        }

        return 0;
    }

    private void placePixel() {
        servoPixel.setPosition(CLOSED_POSITION);
    }

    private void red() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        if (nearboard) {
            // move forward a little bit
            robotProcesses.moveRobotTime(0, 1, 0.2);

            double deltaAngle = turnCheckPixel(initialOrientation);

            if (deltaAngle > Math.PI/18) {
                // on left
                robotProcesses.moveRobotTime(0, 0.6, 0.3);
                // place pixel
                placePixel();
                robotProcesses.moveRobotTime(0, -0.6, 0.3);
            } else if (deltaAngle < -Math.PI/18) {
                // on right
                robotProcesses.moveRobotTime(0, 0.6, 0.3);
                robotProcesses.turnToOrientation(initialOrientation.firstAngle + Math.PI, 1.5);
                // place pixel
                placePixel();
                robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1.5);
                robotProcesses.moveRobotTime(0, -0.6, 0.3);
            } else {
                // in middle
                robotProcesses.moveRobotTime(0, 0.6, 0.3);
                robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI/2, 1);
                // place pixel
                placePixel();
                robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1);
                robotProcesses.moveRobotTime(0, -0.6, 0.3);
            }

            // move towards board and face it
            robotProcesses.moveRobotTime(-3 / Math.sqrt(13), 2 / Math.sqrt(13), 1.5);
            robotProcesses.turnToOrientation((initialOrientation.firstAngle + (float) Math.PI / 2) % (float) TWO_PI, TURN_DURATION);

            // move back a little bit

            // place pixels on board

        } else {


            // place pixels on board

        }

        {
            //turn right pi/2
            //go for one block (0.66m)
            //left one block(0.66)
            //go front 2.5 block
            //put down pixel on the board
        }
    }

    private void blue() {
        boolean nearboard = true;
        if (nearboard) {
            // turn facing north left arg(33.69)
            // vector = sqrt(13)*0.66 meters
            // put down pixel

            // robot already in right orientation from beginning (facing east relative to boards)

            robotMove.fieldCentricMovement(-3 / Math.sqrt(13), 2 / Math.sqrt(13), 0);

        } else {
            //turn left pi/2
            //go for one block (0.66m)
            //right one block(0.66)
            //go front 2.5 block
            //put down pixel on the board
        }
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

    @Override
    public void loop() {


    }
}
