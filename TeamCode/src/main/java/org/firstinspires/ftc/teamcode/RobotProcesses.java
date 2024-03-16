package org.firstinspires.ftc.teamcode;

public class RobotProcesses {
    private RobotMove robotMove;
    private RobotArm robotArm;

    public RobotProcesses(RobotMove robotMove, RobotArm robotArm) {
        this.robotMove = robotMove;
        this.robotArm = robotArm;
    }

    public void moveRobotTime(double x, double y, double seconds) {
        int totalTime = (int) (1000000 * seconds);
        long startTime = System.nanoTime();
        boolean finished = false;

        while (!finished) {
            robotMove.robotCentricMovement(x, y, 0, 0);
            finished = (System.nanoTime() - startTime >= totalTime);
        }
    }

    public void turnToOrientation(double targetAngle, double duration) {
        int totalTime = (int) (1000000 * duration);
        long startTime = System.nanoTime();
        boolean finished = false;

        robotMove.autoCorrectOrientation.firstAngle = (float) targetAngle;

        while (!finished) {
            // auto correct orientation to 90 degrees left facing board
            robotMove.robotCentricMovement(0, 0, 0, 0);
            finished = (System.nanoTime() - startTime >= totalTime);
        }
    }
}
