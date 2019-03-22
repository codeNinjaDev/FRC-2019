package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.controllers.WristController;
/*** Waits for n seconds ***/
public class OuttakePistons extends Command {
	/**
	 * Delays auto
	 * @param seconds Num seconds of delay
	 */
	double goal_time, start_time;
	WristController wrist;
	public OuttakePistons(WristController wrist) {
		this.wrist = wrist;
		
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void execute() {
		
	}

	@Override
	public void end() {
	}

	@Override
	public void initialize() {
		start_time = Timer.getFPGATimestamp();
		wrist.outtakePiston.set(true);
	}
	
	protected void interrupted() {
		end();
	}
}
