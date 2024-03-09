package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.highgui.VideoCapture;
import org.opencv.objdetect.CascadeClassifier;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Autonomous", group = "TeleOp")
public class Autonomous extends OpMode {

    private CascadeClassifier objectDetector;
    private VideoCapture camera;
    private volatile boolean running;
    private RobotMove robotMove;
    private RobotArm robotArm;

    private Gamepad gamepad;
    private boolean objectDetected;

    public Autonomous() {
        // Load OpenCV library and initialize object detector
        System.load("D:/Desktop/xml/opencv/build/java/x64/opencv_java2413.dll");
        this.objectDetector = new CascadeClassifier("D:\\Desktop\\positiveexamples\\classifier\\cascade.xml");
        this.camera = new VideoCapture(0);
        if (!camera.isOpened()) {
            System.out.println("Error: Camera not opened.");
            return;
        }

        // Assuming you've configured the motor in the robot configuration file
        startDetection();
    }


    private void red()
    {
        boolean nearboard = true;

        if(nearboard)
        {
            robotMove.robotCentricMovement(0,0,0,90);
            // turn facing north right arg(33.69)
            // vector = sqrt(13)*0.66 meters
            // put down pixel
        }
        else
        {
            //turn right pi/2
            //go for one block (0.66m)
            //left one block(0.66)
            //go front 2.5 block
            //put down pixel on the board
        }
    }

    private void blue()
    {
        boolean nearboard = true;
        if(nearboard)
        {
            // turn facing north left arg(33.69)
            // vector = sqrt(13)*0.66 meters
            // put down pixel
        }
        else
        {
            //turn left pi/2
            //go for one block (0.66m)
            //right one block(0.66)
            //go front 2.5 block
            //put down pixel on the board
        }
    }

    private void startDetection() {
        running = true;
        new Thread(() -> {
            Mat frameMat = new Mat();
            MatOfRect objectDetections = new MatOfRect();
            boolean objectFound = false;
            while (running) {
                camera.read(frameMat);
                if (!frameMat.empty()) {
                    objectDetector.detectMultiScale(frameMat, objectDetections);

                    if (objectDetections.toArray().length > 0) {
                        // Object detected
                        if (!objectFound) {
                            objectFound = true;
                            // Perform action based on detection
                            // Move forward with 50% power (example)
                            // Put down the pin
                            // Example method to put down the pin
                            // Wait for a moment to ensure the pin is down
                            try {
                                TimeUnit.SECONDS.sleep(1);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            // Return to original position
                            // Example method to return to original position
                            // Stop further detection
                            stopDetection();
                        }
                    } else {
                        // Object not detected
                        objectFound = false;
                        // Resume default behavior (e.g., spinning slowly)
                        // Example: Spin slowly
                        // Add more default behavior as needed
                    }

                    // Delay to avoid excessive processing
                    try {
                        TimeUnit.SECONDS.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else {
                    System.out.println("Error: Frame is empty.");
                }
            }
            // Release resources
            camera.release();
            objectDetector = null;
        }).start();
    }



    public void stopDetection() {
        running = false;
    }

    @Override
    public void init() {
        robotMove = new RobotMove(hardwareMap, gamepad);
        robotArm = new RobotArm(hardwareMap, gamepad);



    }

    @Override
    public void loop() {
    }
}
