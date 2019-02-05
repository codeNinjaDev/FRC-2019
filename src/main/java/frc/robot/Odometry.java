package frc.robot;

import java.nio.Buffer;

import edu.wpi.first.wpilibj.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.DriveController;
import org.apache.commons.collections4.queue.CircularFifoQueue;;
public class Odometry {
    CircularFifoQueue drivePositions;
    DriveController driveController;
    private TimeVector prevDriveVector;
    private TimeVector currDriveVector;
    private double prevX, prevY, prevDistance;
    private double currX, currY, currDistance;
    Timer visionTimer;
    public Odometry(DriveController driveController, Timer visionTimer) {
        this.driveController = driveController;
        this.visionTimer = visionTimer;

        drivePositions = new CircularFifoQueue<TimeVector>(10);
        currDriveVector = new TimeVector();

        prevX = prevY = prevDistance = currX = currY = currDistance = 0;
        prevDriveVector = new TimeVector(currX, currY);

    }

    public void update() {

        currDistance = driveController.getAverageTotalDistance() - prevDistance;
        currX = prevX + currDistance*Math.cos(driveController.getTotalGyroAngle());
        currY = prevY + currDistance*Math.sin(driveController.getTotalGyroAngle());

        prevX = currX;
        prevY = currY;

        currDriveVector.setXY(currX, currY);
        currDriveVector.setTime(visionTimer.get());
        SmartDashboard.putNumber("ODOMETRY-ANGLE", currDriveVector.getAngle());


        TimeVector storedVector = currDriveVector;
        
        drivePositions.add(storedVector);
        prevDistance = currDistance;

        

    }
                


}


