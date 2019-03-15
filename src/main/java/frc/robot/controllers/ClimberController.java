package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pid.*;
import frc.robot.Params;
import frc.robot.hardware.*;
import frc.robot.hardware.RemoteControl.Joysticks;

/*** Controls mechanism for both cargo and hatch */
public class ClimberController extends Subsystem {
	private VictorSP climberMotorA, climberMotorB;
	private SpeedControllerGroup climbMotors;
	private RemoteControl humanControl;

	public DigitalInput climbDoneButton;


	private DriveController drive;
	/*** Enum that lets us shift our status from init to teleop and vice versa ***/
	public enum ArmState {
		kInitialize, kTeleop
	};


	// IS going down positive or negative?
	public ClimberController(RemoteControl humanControl, DriveController driveController) {
		this.humanControl = humanControl;
		this.drive = driveController;
		climbDoneButton = new DigitalInput(Ports.CLIMBER_BUTTON);

		climberMotorA = new VictorSP(Ports.CLIMBER_PORT_A);
		climberMotorB = new VictorSP(Ports.CLIMBER_PORT_B);

		climberMotorA.setInverted(true);
		climbMotors = new SpeedControllerGroup(climberMotorA, climberMotorB);
	}

	public void stop() {
		climbMotors.set(0);

	}

	public void reset() {
	}

	public void update() {
		if(humanControl.climbDesired()) {

			climbMotors.set((humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kLY)));

		}
			


	}


	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
