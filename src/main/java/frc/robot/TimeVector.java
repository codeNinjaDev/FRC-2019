package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import java.lang.Math;


public class TimeVector extends Vector2d {

    private double timestamp;
    public TimeVector(double x, double y) {
        super(x, y);
        timestamp = 0;
    }

    public TimeVector() {
        super();
        timestamp = 0;
    }

    public void setXY(double x, double y) {
        super.x = x;
        super.y = y;
    }

    public double getAngle() {
        
        return Math.atan2(super.y, super.x);
    }

    public double getTime() {
        return timestamp;
    }

    public void setTime(double timestamp) {
        this.timestamp = timestamp;
    }


}