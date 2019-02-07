package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.*;
public class VisionController extends Subsystem {
	private boolean is_enabled, driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen;
	private NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw, driveWanted, tapeWanted, cargoWanted, videoTimestamp;
	private double targetAngle, timestamp, cargoAngle, tapeAngle;
	private int left_contour, right_contour;
	private RemoteControl humanControl;
	private Timer visionTimer;
	NetworkTableInstance instance;
	NetworkTable chickenVision;
	public VisionController(RemoteControl humanControl, Timer visionTimer) {
		this.humanControl = humanControl;
		this.visionTimer = visionTimer;
		instance = NetworkTableInstance.getDefault();
		instance.addConnectionListener(event -> {
			visionTimer.start();
		}, true);
		chickenVision = instance.getTable("ChickenVision");

		tapeDetected = chickenVision.getEntry("tapeDetected");
		cargoDetected = chickenVision.getEntry("cargoDetected");
		tapeYaw = chickenVision.getEntry("tapeYaw");
		cargoYaw = chickenVision.getEntry("cargoYaw");

		driveWanted = chickenVision.getEntry("Driver");
		tapeWanted = chickenVision.getEntry("Tape");
		cargoWanted = chickenVision.getEntry("Cargo");

		videoTimestamp = chickenVision.getEntry("VideoTimestamp");

		is_enabled = tapeVision = cargoVision = false;
		driverVision = true;
	}

	public void reset() {
		
	}
	
	
	public void update() {

		cargoSeen = cargoDetected.getBoolean(false);
		tapeSeen = tapeDetected.getBoolean(false);

		if(tapeSeen)
			tapeAngle = tapeYaw.getDouble(0);
		if(cargoSeen)
			cargoAngle = cargoYaw.getDouble(0);


		if(humanControl.getCargoVisionDesired()) {
			driverVision = false;
			tapeVision = false;
			cargoVision = true;


			driveWanted.setBoolean(driverVision);
			tapeWanted.setBoolean(tapeVision);
			cargoWanted.setBoolean(cargoVision);
			SmartDashboard.putBoolean("Cargo detected", cargoDetected.getBoolean(false));

			
		} else if (humanControl.getTapeVisionDesired()) {
			driverVision = false;
			tapeVision = true;
			cargoVision = false;

			driveWanted.setBoolean(driverVision);
			tapeWanted.setBoolean(tapeVision);
			cargoWanted.setBoolean(cargoVision);
			
		} else {
			driverVision = true;
			tapeVision = false;
			cargoVision = false;

			driveWanted.setBoolean(driverVision);
			tapeWanted.setBoolean(tapeVision);
			cargoWanted.setBoolean(cargoVision);
		}
		
		SmartDashboard.putNumber("Cargo Yaw", targetAngle);

	}

	public double targetYaw() {
		if(cargoVision) {
			return cargoYaw();
		} else if(tapeVision) {
			return tapeYaw();
		} else {
			return 0;
		}

	}
	
	public double cargoYaw() {
		update();
		if(cargoVision && cargoSeen)
			return cargoAngle;
		else
			return 0;
	}
	public double tapeYaw() {
		update();
		if(tapeVision && tapeSeen)
			return tapeAngle;
		else
			return 0;
	}
	
	
	public void enable() {
		is_enabled = true;
	}
	
	public void disable() {
		is_enabled = false;
	}
	
	public boolean isEnabled() {
		return is_enabled;
	}
	

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	private void startTimer() {
		visionTimer.start();
	}
	
}
