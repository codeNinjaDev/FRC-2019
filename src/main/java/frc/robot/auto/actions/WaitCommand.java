package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
/*** Waits for n seconds ***/
public class WaitCommand extends Command {
	/**
	 * Delays auto
	 * @param seconds Num seconds of delay
	 */
	double goal_time, start_time;
	public WaitCommand(double seconds) {
		goal_time = seconds;
		
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
	}
	
	protected void interrupted() {
		end();
	}
}
