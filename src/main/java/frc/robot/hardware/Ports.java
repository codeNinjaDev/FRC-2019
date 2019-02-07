package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C.Port;
/***************** DEALS WITH PORTS *****************/
public class Ports {
	
	
	public static final int LEFT_DRIVE_MOTOR_A_PWM_PORT            = 6; //front left
	public static final int LEFT_DRIVE_MOTOR_B_PWM_PORT            = 7; //back left
	public static final int RIGHT_DRIVE_MOTOR_A_PWM_PORT           = 5; //front right
	public static final int RIGHT_DRIVE_MOTOR_B_PWM_PORT           = 4; //back right*/

	public static final int LEFT_INTAKE_WHEEL_PORT            = 3; 
	public static final int RIGHT_INTAKE_WHEEL_PORT            = 2; 
	public static final int WRIST_ARM_A_PORT           = 1; 
	public static final int WRIST_ARM_B_PORT           = 0; 

	public static final int CLIMBER_A_PORT           = 9; 
	public static final int CLIMBER_B_PORT           = 8; 


	// ***************** PDP CHANNELS *****************
	public static final int LEFT_DRIVE_MOTOR_A_PDP_CHAN           = 12;
	public static final int LEFT_DRIVE_MOTOR_B_PDP_CHAN           = 13;
	public static final int RIGHT_DRIVE_MOTOR_A_PDP_CHAN          = 14;
	public static final int RIGHT_DRIVE_MOTOR_B_PDP_CHAN          = 15;

	// ***************** DIGITAL I/O PORTS *****************
	public static final int LIGHTS_DIO_PORTS[]                    = {2, 3, 4, 5};
	public static final int RIGHT_DRIVE_ENCODER_PORTS[]            = {6, 7};
	//Top Ports
	public static final int LEFT_DRIVE_ENCODER_PORTS[]           = {8, 9};

	public static final int[] OUTTAKE_SOLENOIDS = {1, 2};
	public static final int[] SHOOTER_SOLENOIDS = {3, 4};

	/**** ANALOG IN */
	public static final int CLIMBER_POT           = 0; 
	public static final int WRIST_POT           = 1; 


	// ***************** MISC *****************

	


	// ***************** JOYSTICK USB PORTS *****************
	//MAKE SURE JOYSTICKS ARE SET TO "D" position on back

	public static final boolean USING_WIN_DRIVER_STATION             = true;
	public static final int DRIVER_JOY_USB_PORT                   = 0;
	public static final int OPERATOR_JOY_USB_PORT                 = 1;
	// ***************** BUTTONS *****************

	//Controller button ports
	public static final int DRIVE_DIRECTION_BUTTON_PORT           = 9;

}
