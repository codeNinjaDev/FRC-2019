package frc.robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import frc.robot.hardware.*;
import frc.robot.controllers.*;
import frc.robot.auto.*;
import frc.robot.feed.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	
	/*** Initializes all classes ***/

	private RemoteControl humanControl;
	private DriveController driveController;
	private VisionController visionController;
	private LightController lights;
	private MotionController motion;
	private DashboardLogger dashboardLogger;
	private DashboardInput input;
	private AutoSelector auto;
	private Timer visionTimer;
	private Odometry odometry;

	
	public Robot() {
		super();
		visionTimer = new Timer();

		humanControl = new ControlBoard();
		visionController = new VisionController(humanControl, visionTimer);

		driveController = new DriveController(humanControl, visionController);
		lights = new LightController();
		motion = new MotionController(driveController);
		dashboardLogger = new DashboardLogger(humanControl, driveController, motion);
		input  = new DashboardInput();	
		auto = new AutoSelector(driveController, motion, visionController, lights);
		odometry = new Odometry(driveController, visionTimer);

	}


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(240, 180);
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("DriverCam", 240, 180);
			
			Mat source = new Mat();
			Mat output = new Mat();
			
			while(!Thread.interrupted()) {
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();

		// Sets enabled lights
		lights.setEnabledLights();
		// Resets autonomous
		//List autonomous routines on Dashboard
		auto.listOptions();
		//Updates input from Robot Preferences
		input.updateInput();

		
		
		/*** Update Dashboard ***/

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		/*** Reset all hardware ***/
		driveController.reset();
		motion.reset();
		
		//Reset auto timer
		//Update robot preferences
		input.updateInput();
		//Starts Autonomous Routine
		auto.getSelectedAuto().start();
	}

	/**
	 * This function is called periodically during autonomous, but is mainly used for logging
	 */
	@Override
	public void autonomousPeriodic() {
		odometry.update();
		Scheduler.getInstance().run();
		//Log Gyro angle
		//SmartDashboard.putNumber("gyro", robot.getAngle());
		//Start auto pattern on led strip
		lights.setAutoLights();
		//Log data to the Dashboard
		dashboardLogger.updateData();

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		//Stop autonomous
		Scheduler.getInstance().removeAll();
		//Reset Gyro
		//robot.resetGyro();
		//Reset Encoders
		driveController.resetEncoders();
		//Reset Drive
		driveController.reset();
		//Update input from Robot Preferences
		input.updateInput();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//Updates Gyro angle
		//SmartDashboard.putNumber("gyro", robot.getAngle());
		visionController.update();
		//Logs data to Dashboard
		dashboardLogger.updateData();
		//Read input from Gamepad
		humanControl.readControls();
		//Updates Drive e.g arcadeDrive
		driveController.update();
		//Set enabled Light pattern
		lights.setEnabledLights();
		//Log Data to Dashboard

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {


		input.updateInput();

	}

	public void disabledInit() {
		//robot.resetGyro();
		//Reset drive
		driveController.reset();
		//Update input from Dashboard
		input.updateInput();
		//Log Data

	}

	public void disabledPeriodic() {
		dashboardLogger.putParamData();
		visionController.update();

		//Put Gyro Angle on SmartDashboard
		//SmartDashboard.putNumber("gyro", robot.getAngle());
		//Update input from Dashboard
		input.updateInput();
		//Logs arm angle on Dashboard
		//SmartDashboard.putNumber("Arm Angle", robot.getArmAngle());
		//Log Dashboard
		dashboardLogger.updateData();
		//Read Gamepad Controlls
		humanControl.readControls();
		//Set Disabled pattern for led strips
		lights.setDisabledLights();
	}
	
	

}