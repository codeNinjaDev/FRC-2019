package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C.Port;
/***************** DEALS WITH PORTS *****************/
public class Ports {
	
	
	public static final int LEFT_DRIVE_MOTORS_PWM_PORT            = 4; // left
	public static final int RIGHT_DRIVE_MOTORS_PWM_PORT           = 6; //right

	public static final int LEFT_INTAKE_WHEEL_PORT            = 3; 
	public static final int RIGHT_INTAKE_WHEEL_PORT            = 2; 
	public static final int WRIST_ARM_A_PORT           = 1; 
	public static final int WRIST_ARM_B_PORT           = 0; 
	public static final int CAMERA_SERVO           = 7; 
	public static final int CLIMBER_PORT_A           = 8; 
	public static final int CLIMBER_PORT_B           = 9; 


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
	public static final int CLIMBER_BUTTON           = 0; 

	public static final int[] OUTTAKE_SOLENOIDS = {3};
	public static final int[] SHOOTER_SOLENOIDS = {2};

	/**** ANALOG IN */
	public static final int WRIST_POT[]           = {0,1}; 


	// ***************** MISC *****************

	//DIO


	// ***************** JOYSTICK USB PORTS *****************
	//MAKE SURE JOYSTICKS ARE SET TO "D" position on back

	public static final boolean USING_WIN_DRIVER_STATION             = true;
	public static final int DRIVER_JOY_USB_PORT                   = 0;
	public static final int OPERATOR_JOY_USB_PORT                 = 1;
	// ***************** BUTTONS *****************

	//Controller button ports
	public static final int DRIVE_DIRECTION_BUTTON_PORT           = 9;

}
