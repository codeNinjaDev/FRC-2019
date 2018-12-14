package frc.robot.feed;

import frc.robot.Params;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Allows input from Robot Preferences in SmartDashboard ***/
public class DashboardInput {
	/*** Preferences object to get input ***/
	Preferences preferences;

	// TODO Check if Dashboard Variables or Params works better
	/*** Get Instance of preferences and Update input ***/
	public DashboardInput() {
		// Get overall Input from preferences
		preferences = Preferences.getInstance();

		

		Params.drive_p = preferences.getDouble("DRIVE_P_VALUE", 0);
		Params.drive_i = preferences.getDouble("DRIVE_I_VALUE", 0);
		Params.drive_d = preferences.getDouble("DRIVE_D_VALUE", 0);

		
		// Set Max Speed to preferences Max Speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);

		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);

		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);
		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);

		Params.track_base_width = preferences.getDouble("TRACK_BASE_WIDTH", 0);
		Params.wheel_base_width = preferences.getDouble("WHEEL_BASE_WIDTH", 0);
		Params.dt = preferences.getDouble("DELTA_TIME_MP", 0);
	}

	/*** Updates input from Dashboard ***/
	public void updateInput() {
		// Gets information from dashboard
		preferences = Preferences.getInstance();

		

		Params.drive_p = preferences.getDouble("DRIVE_P_VALUE", 0);
		Params.drive_i = preferences.getDouble("DRIVE_I_VALUE", 0);
		Params.drive_d = preferences.getDouble("DRIVE_D_VALUE", 0);

		// Gets Custom Autonomous Time
		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);

		

		// Gets max speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);
		SmartDashboard.putNumber("Dash Max Speed", DashboardVariables.max_speed);

		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);

		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);

		Params.track_base_width = preferences.getDouble("TRACK_BASE_WIDTH", 0);
		Params.wheel_base_width = preferences.getDouble("WHEEL_BASE_WIDTH", 0);
		Params.dt = preferences.getDouble("DELTA_TIME_MP", 0);

	}

}
