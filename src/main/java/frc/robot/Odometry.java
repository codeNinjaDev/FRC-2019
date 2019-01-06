package frc.robot;

import frc.robot.controllers.DriveController;

public class Odometry {
    DriveController driveController;
    private double prevX, prevY, prevDistance;
    private double currX, currY, currDistance;
    public Odometry(DriveController driveController) {
        this.driveController = driveController;
        prevX = prevY = prevDistance = currX = currY = currDistance = 0;
    }

    public void update() {
        currDistance = driveController.getAverageTotalDistance() - prevDistance;
        currX = prevX + currDistance*Math.cos(driveController.getTotalGyroAngle());
        currY = prevY + currDistance*Math.sin(driveController.getTotalGyroAngle());

        prevX = currX;
        prevY = currY;
        
        prevDistance = currDistance;
    }
}
