package org.firstinspires.ftc.teamcode;

/*
@Autonomous(name = "Auto3", group = "Autonomous")
public class Auto3 extends LinearOpMode {
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

        /*WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        MyPipeline myPipeline = new MyPipeline();
        camera.setPipeline(myPipeline);

        try {
            initializeObjectDetector();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }*/
/*
        waitForStart();

        //awayBoardModified("blue", "middle");
        runMode();
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

    private void runMode() {
        Orientation initialOrientation = robotMove.getIMUOrientation();

        telemetry.addData("On right", "");
        robotProcesses.moveRobotTime(0, 0.8, 0.8);

        // turn and place pixel
        robotProcesses.turnToOrientation(initialOrientation.firstAngle - Math.PI, 1.5);
        placePixel();
        robotProcesses.turnToOrientation(initialOrientation.firstAngle, 1.5);

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