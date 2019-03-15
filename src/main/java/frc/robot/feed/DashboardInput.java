package frc.robot.feed;

import frc.robot.Params;

import java.util.Map;

import com.sun.javadoc.Parameter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Allows input from Robot Preferences in SmartDashboard ***/
public class DashboardInput {
	/*** Preferences object to get input ***/
	private ShuffleboardTab parameterTab; 
	private NetworkTableEntry maxSpeed;
	private NetworkTableEntry driveP;
	private NetworkTableEntry driveI;
	private NetworkTableEntry driveD;
	private NetworkTableEntry gyroP;
	private NetworkTableEntry gyroI;
	private NetworkTableEntry gyroD;
	private NetworkTableEntry armP;
	private NetworkTableEntry armI;
	private NetworkTableEntry armD;
	private NetworkTableEntry armStow;
	private NetworkTableEntry armLowCargo;
	private NetworkTableEntry armHighCargo;
	private NetworkTableEntry armMidCargo;
	private NetworkTableEntry armScoreHatch;
	private NetworkTableEntry armLoadHatch;
	private NetworkTableEntry armFloorHatch;
	private NetworkTableEntry armMaxSpeed;

	private NetworkTableEntry visionP;
	private NetworkTableEntry visionI;
	private NetworkTableEntry visionD;
	private NetworkTableEntry cameraClimb;
	private NetworkTableEntry cameraCargo;
	private NetworkTableEntry cameraTape;



	private NetworkTableEntry autoDelay;
	private ShuffleboardLayout drivePID;
	private ShuffleboardLayout armPID;
	private ShuffleboardLayout visionPID;


	// TODO Check if Dashboard Variables or Params works better
	/*** Get Instance of preferences and Update input ***/
	public DashboardInput() {
		// Get overall Input from preferences
		try {
			parameterTab = Shuffleboard.getTab("Parameters");

			drivePID = parameterTab.getLayout("Drive PID", BuiltInLayouts.kList).withSize(3, 3);
			armPID = parameterTab.getLayout("Arm PID", BuiltInLayouts.kList).withSize(3, 3);
			visionPID = parameterTab.getLayout("Vision PID", BuiltInLayouts.kList).withSize(3, 3);

			maxSpeed = parameterTab.addPersistent("Max Speed", Params.MAX_SPEED).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();

			driveP = parameterTab.addPersistent("Drive P", Params.drive_p).getEntry();
			driveI = parameterTab.addPersistent("Drive I", Params.drive_i).getEntry();
			driveD = parameterTab.addPersistent("Drive D", Params.drive_d).getEntry();

			gyroP = parameterTab.addPersistent("gyro P", Params.new_drive_p).getEntry();
			gyroI = parameterTab.addPersistent("gyro I", Params.new_drive_i).getEntry();
			gyroD = parameterTab.addPersistent("gyro D", Params.new_drive_d).getEntry();

			armP = parameterTab.addPersistent("Arm P", Params.arm_p).getEntry();
			armI = parameterTab.addPersistent("Arm I", Params.arm_i).getEntry();
			armD = parameterTab.addPersistent("Arm D", Params.arm_d).getEntry();

			armStow = parameterTab.addPersistent("Arm Stow Angle", Params.ARM_STOW_SETPOINT).getEntry();
			armLowCargo = parameterTab.addPersistent("Arm Low Cargo Angle", Params.ARM_LOW_CARGO_SETPOINT).getEntry();
			armMidCargo = parameterTab.addPersistent("Arm Mid Cargo Angle", Params.ARM_MID_CARGO_SETPOINT).getEntry();
			armHighCargo = parameterTab.addPersistent("Arm High Cargo Angle", Params.ARM_HIGH_CARGO_SETPOINT).getEntry();

			armScoreHatch = parameterTab.addPersistent("Hatch Score Angle", Params.ARM_SCORE_HATCH_SETPOINT).getEntry();
			armLoadHatch = parameterTab.addPersistent("Hatch Load Angle", Params.ARM_LOAD_HATCH_SETPOINT).getEntry();
			armFloorHatch = parameterTab.addPersistent("Hatch Floor Angle", Params.ARM_FLOOR_HATCH_SETPOINT).getEntry();
			armMaxSpeed = parameterTab.addPersistent("Arm Max Speed", Params.MAX_ARM_PID_OUT).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();

			cameraClimb = parameterTab.addPersistent("CameraClimb Angle", Params.camera_climb).getEntry();
			cameraTape = parameterTab.addPersistent("CameraTape Angle", Params.camera_tape).getEntry();
			cameraCargo = parameterTab.addPersistent("CameraCargo Angle", Params.camera_cargo).getEntry();

			visionP = parameterTab.addPersistent("Vision P", Params.vision_p).getEntry();
			visionI = parameterTab.addPersistent("Vision I", Params.vision_i).getEntry();
			visionD = parameterTab.addPersistent("Vision D", Params.vision_d).getEntry();

			visionPID.add("Vision P", visionP);
			visionPID.add("Vision I", visionI);
			visionPID.add("Vision D", visionD);

			drivePID.add("Drive P", driveP);
			drivePID.add("Drive I", driveI);
			drivePID.add("Drive D", driveD);

			armPID.add("Arm P", armP);
			armPID.add("Arm I", armI);
			armPID.add("Arm D", armD);

			updateInput();
		} catch(Exception e) {

		}
	}

	/*** Updates input from Dashboard ***/
	public void updateInput() {
		
		try {
			Params.arm_p = armP.getDouble(Params.arm_p);
			Params.arm_i = armI.getDouble(Params.arm_i);
			Params.arm_d = armD.getDouble(Params.arm_d);

			Params.vision_p = visionP.getDouble(Params.vision_p);
			Params.vision_i = visionI.getDouble(Params.vision_i);
			Params.vision_d = visionD.getDouble(Params.vision_d);

			Params.camera_climb = cameraClimb.getDouble(Params.camera_climb);
			Params.camera_tape = cameraTape.getDouble(Params.camera_tape);
			Params.camera_cargo = cameraCargo.getDouble(Params.camera_cargo);


			Params.ARM_STOW_SETPOINT = armStow.getDouble(Params.ARM_STOW_SETPOINT);
			Params.ARM_LOW_CARGO_SETPOINT = armLowCargo.getDouble(Params.ARM_LOW_CARGO_SETPOINT);
			Params.ARM_MID_CARGO_SETPOINT = armMidCargo.getDouble(Params.ARM_MID_CARGO_SETPOINT);
			Params.ARM_HIGH_CARGO_SETPOINT = armHighCargo.getDouble(Params.ARM_HIGH_CARGO_SETPOINT);
			Params.ARM_LOAD_HATCH_SETPOINT = armLoadHatch.getDouble(Params.ARM_LOAD_HATCH_SETPOINT);
			Params.ARM_SCORE_HATCH_SETPOINT = armScoreHatch.getDouble(Params.ARM_SCORE_HATCH_SETPOINT);
			Params.ARM_FLOOR_HATCH_SETPOINT = armFloorHatch.getDouble(Params.ARM_FLOOR_HATCH_SETPOINT);

			Params.MAX_ARM_PID_OUT = armMaxSpeed.getDouble(Params.MAX_ARM_PID_OUT);
			Params.drive_p = driveP.getDouble(Params.drive_p);
			Params.drive_i = driveI.getDouble(Params.drive_i);
			Params.drive_d = driveD.getDouble(Params.drive_d);
			
			Params.new_drive_p = gyroP.getDouble(Params.new_drive_p);
			Params.new_drive_i = gyroI.getDouble(Params.new_drive_i);
			Params.new_drive_d = gyroD.getDouble(Params.new_drive_d);
			Params.TIME_DELAY = autoDelay.getDouble(Params.TIME_DELAY);
			Params.MAX_SPEED = maxSpeed.getDouble(Params.MAX_SPEED);
	
		} catch (Exception e) {
			
		}
	}

	
}
