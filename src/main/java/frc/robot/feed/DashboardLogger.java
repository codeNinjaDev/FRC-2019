package frc.robot.feed;

import frc.robot.Params;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.MotionController;
import frc.robot.controllers.VisionController;
import frc.robot.controllers.WristController;
import frc.robot.hardware.RemoteControl;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DashboardLogger {
	private RemoteControl humanControl;
	private DriveController driveController;
	private VisionController visionController;
	private ShuffleboardTab controllerTab; 
	private ShuffleboardTab electricalTab; 
	private ShuffleboardTab sensorTab; 
	private ShuffleboardTab matchTab; 
	private ShuffleboardTab visionTab; 
	private NetworkTableEntry batteryVoltage;
	private NetworkTableEntry brownOut;
	private NetworkTableEntry current3v3;
	private NetworkTableEntry current5v;
	private NetworkTableEntry current6v;
	private NetworkTableEntry inputCurrent;
	private NetworkTableEntry voltage3v3;
	private NetworkTableEntry voltage5v;
	private NetworkTableEntry voltage6v;
	private NetworkTableEntry inputVoltage;
	private NetworkTableEntry eventName;
	private NetworkTableEntry matchName;
	private NetworkTableEntry matchType;
	private NetworkTableEntry alliance;
	private NetworkTableEntry location;
	private NetworkTableEntry driverJoyLY;
	private NetworkTableEntry driverJoyLX;
	private NetworkTableEntry driverJoyY;
	private NetworkTableEntry time;



	NetworkTableEntry resetEncoder;

	NetworkTableEntry encoderAngle;
	NetworkTableEntry armSetpoint;







	Compressor compressor;
	WristController wrist;

	public DashboardLogger(RemoteControl humanControl, DriveController driveController, VisionController vision, WristController wrist) {
		this.humanControl = humanControl;
		this.driveController = driveController;
		this.visionController = vision;
		this.wrist = wrist;
		controllerTab = Shuffleboard.getTab("Controllers");
		electricalTab = Shuffleboard.getTab("Electrical");
		sensorTab = Shuffleboard.getTab("Sensors");
		matchTab = Shuffleboard.getTab("Match Info");
		visionTab = Shuffleboard.getTab("Vision Info");



		resetEncoder = sensorTab.add("Reset Encoder", false).getEntry();
		encoderAngle = sensorTab.add("Encoder Angle", 0).getEntry();
		armSetpoint = sensorTab.add("Arm Setpoint", 0).getEntry();

		if(DriverStation.getInstance().isFMSAttached()) {
			Shuffleboard.startRecording();
			putMatchInfo();
		}
		compressor = new Compressor();
	}

	public void updateData() {
		if(resetEncoder.getBoolean(false))
			wrist.reset();
			
		//SmartDashboard.putNumber("DEBUG_FPGATimestamp", robot.getTimestamp());
		if(DriverStation.getInstance().isFMSAttached()) {
			
			putMatchInfo();
			
		}
		putRobotElectricalData();
		putJoystickAxesData();
		putMotorOutputs();
		putSensors();

		
		
	}


	public void putRobotElectricalData() {
		SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putBoolean("Brown Out", RobotController.isBrownedOut());
		SmartDashboard.putNumber("3Volt Rail Current", RobotController.getCurrent3V3());
		SmartDashboard.putNumber("5Volt Rail Current", RobotController.getCurrent5V());
		SmartDashboard.putNumber("6Volt Rail Current", RobotController.getCurrent6V());
		SmartDashboard.putNumber("Input Current", RobotController.getInputCurrent());
		SmartDashboard.putNumber("3Volt Rail Voltage", RobotController.getVoltage3V3());
		SmartDashboard.putNumber("5Volt Rail Voltage", RobotController.getVoltage5V());
		SmartDashboard.putNumber("6Volt Rail Voltage", RobotController.getVoltage6V());
		SmartDashboard.putNumber("Input Voltage", RobotController.getInputVoltage());

	}

	public void putMatchInfo() {
		SmartDashboard.putString("EVENT_NAME", DriverStation.getInstance().getEventName());
		SmartDashboard.putNumber("MATCH_NUMBER", DriverStation.getInstance().getMatchNumber());
		SmartDashboard.putString("MATCH_TYPE", DriverStation.getInstance().getMatchType().toString());
		SmartDashboard.putString("ALLIANCE", DriverStation.getInstance().getAlliance().toString());
		SmartDashboard.putNumber("LOCATION", DriverStation.getInstance().getLocation());
		SmartDashboard.putNumber("MATCH_TIME", DriverStation.getInstance().getMatchTime());

	}
	
	public void putJoystickAxesData() {
		SmartDashboard.putNumber("DRIVERJOY_operatorrLeftX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLX));
		SmartDashboard.putNumber("DRIVERJOY_driverLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("DRIVERJOY_driverRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX));
		SmartDashboard.putNumber("DRIVERJOY_driverRightY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRY));
		SmartDashboard.putNumber("OPERATORJOY_operatorLeftX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kLX));
		SmartDashboard.putNumber("OPERATORJOY_operatorLeftY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kLY));
		SmartDashboard.putNumber("OPERATORJOY_operatorRightX",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kRX));
		SmartDashboard.putNumber("OPERATORJOY_operatorRightY",
				humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
	}

	

	public void putMotorOutputs() {
		
		SmartDashboard.putBoolean("MOTORS_leftDrive_REVERSED", driveController.leftDriveMotors.getInverted());
		SmartDashboard.putBoolean("MOTORS_rightDrive_REVERSED", driveController.leftDriveMotors.getInverted());
		
		SmartDashboard.putNumber("LEFT_DRIVE_MOTORS", driveController.leftDriveMotors.get());
		

	}

	
	public void putParamData() {

		SmartDashboard.putNumber("DRIVE_P", Params.drive_p);
		SmartDashboard.putNumber("DRIVE_I", Params.drive_i);
		SmartDashboard.putNumber("DRIVE_D", Params.drive_d);

		SmartDashboard.putNumber("TURN_P", Params.turn_drive_p);		
		SmartDashboard.putNumber("TURN_I", Params.turn_drive_i);
		SmartDashboard.putNumber("TURN_D", Params.turn_drive_d);

		SmartDashboard.putNumber("MAX_SPEED", Params.MAX_SPEED);
		SmartDashboard.putNumber("DELTA_TIME_MP", Params.dt);
		SmartDashboard.putNumber("WHEEL_CIRCUMFERENCE", Params.WHEEL_CIRCUMFERENCE);
		SmartDashboard.putNumber("WHEEL_DIAMETER", Params.WHEEL_DIAMETER);
		SmartDashboard.putNumber("TRACK_BASE_WIDTH", Params.track_base_width);
		SmartDashboard.putNumber("WHEEL_BASE_WIDTH", Params.wheel_base_width);
		SmartDashboard.putNumber("X_SPEED_MULTIPLIER", Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER);
		SmartDashboard.putNumber("Y_SPEED_MULTIPLIER", Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER);
	
	}
	
	
	
	
	public void putSensors() {
		SmartDashboard.putNumber("LEFT_ENC_DISTANCE", driveController.leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("RIGHT_ENC_DISTANCE", driveController.rightDriveEncoder.getDistance());
		SmartDashboard.putNumber("LEFT_ENC_VELOCITY", driveController.leftDriveEncoder.getRate());
		SmartDashboard.putNumber("RIGHT_ENC_VELOCITY", driveController.rightDriveEncoder.getRate());
		SmartDashboard.putNumber("GYRO_ANGLE", driveController.getGyroAngle());
		encoderAngle.setNumber(wrist.wristPotentiometer.getDistance());
		armSetpoint.setNumber(wrist.armPID.getSetpoint());
		
		

	}
	

}
