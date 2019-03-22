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

	private NetworkTableEntry armP;
	private NetworkTableEntry armI;
	private NetworkTableEntry armD;
	private NetworkTableEntry armF;
	private NetworkTableEntry armStow;
	private NetworkTableEntry armLowCargo;
	private NetworkTableEntry armHighCargo;
	private NetworkTableEntry armMiscCargo;
	private NetworkTableEntry armBackCargo;

	private NetworkTableEntry armMaxSpeed;

	private NetworkTableEntry visionP;
	private NetworkTableEntry visionI;
	private NetworkTableEntry visionD;




	private NetworkTableEntry autoDelay;
	private ShuffleboardLayout armPID;
	private ShuffleboardLayout armAngles;

	private ShuffleboardLayout visionPID;


	// TODO Check if Dashboard Variables or Params works better
	/*** Get Instance of preferences and Update input ***/
	public DashboardInput() {
		// Get overall Input from preferences
		try {
			parameterTab = Shuffleboard.getTab("Parameters");

			armPID = parameterTab.getLayout("Arm PIDF", BuiltInLayouts.kList).withSize(1, 4);
			visionPID = parameterTab.getLayout("Vision PID", BuiltInLayouts.kList).withSize(1, 3);
			armAngles = parameterTab.getLayout("Arm Angles", BuiltInLayouts.kList).withSize(1, 5);
			maxSpeed = parameterTab.addPersistent("Max Speed", Params.MAX_SPEED).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();

			armP = armPID.addPersistent("Arm P", Params.arm_p).getEntry();
			armI = armPID.addPersistent("Arm I", Params.arm_i).getEntry();
			armD = armPID.addPersistent("Arm D", Params.arm_d).getEntry();
			armF = armPID.addPersistent("Arm F", Params.arm_f).getEntry();

			armStow = armAngles.addPersistent("Arm Stow Angle", Params.ARM_STOW_SETPOINT).getEntry();
			armLowCargo = armAngles.addPersistent("Arm Cargo Bay Angle", Params.ARM_LOW_CARGO_SETPOINT).getEntry();
			armMiscCargo = armAngles.addPersistent("Arm Custom Cargo Angle", Params.ARM_MISC_CARGO_SETPOINT).getEntry();
			armHighCargo = armAngles.addPersistent("Arm Rocket Cargo Angle", Params.ARM_HIGH_CARGO_SETPOINT).getEntry();
			armBackCargo = armAngles.addPersistent("Arm Back Cargo Angle", Params.ARM_BACK_CARGO_SETPOINT).getEntry();

			
			armMaxSpeed = parameterTab.addPersistent("Arm Max Speed", Params.MAX_ARM_PID_OUT).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();


			visionP = visionPID.addPersistent("Vision P", Params.vision_p).getEntry();
			visionI = visionPID.addPersistent("Vision I", Params.vision_i).getEntry();
			visionD = visionPID.addPersistent("Vision D", Params.vision_d).getEntry();


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
			Params.arm_f = armF.getDouble(Params.arm_f);
			Params.vision_p = visionP.getDouble(Params.vision_p);
			Params.vision_i = visionI.getDouble(Params.vision_i);
			Params.vision_d = visionD.getDouble(Params.vision_d);

		

			Params.ARM_STOW_SETPOINT = armStow.getDouble(Params.ARM_STOW_SETPOINT);
			Params.ARM_LOW_CARGO_SETPOINT = armLowCargo.getDouble(Params.ARM_LOW_CARGO_SETPOINT);
			Params.ARM_MISC_CARGO_SETPOINT = armMiscCargo.getDouble(Params.ARM_MISC_CARGO_SETPOINT);
			Params.ARM_HIGH_CARGO_SETPOINT = armHighCargo.getDouble(Params.ARM_HIGH_CARGO_SETPOINT);
			Params.ARM_BACK_CARGO_SETPOINT = armBackCargo.getDouble(Params.ARM_BACK_CARGO_SETPOINT);

			Params.MAX_ARM_PID_OUT = armMaxSpeed.getDouble(Params.MAX_ARM_PID_OUT);

			Params.MAX_SPEED = maxSpeed.getDouble(Params.MAX_SPEED);
	
		} catch (Exception e) {
			
		}
	}

	
}
