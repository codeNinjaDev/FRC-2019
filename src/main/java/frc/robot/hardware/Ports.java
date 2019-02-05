package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C.Port;
/***************** DEALS WITH PORTS *****************/
public class Ports {
	
	
	public static final int LEFT_DRIVE_MOTOR_A_PWM_PORT            = 6; //front left
	public static final int LEFT_DRIVE_MOTOR_B_PWM_PORT            = 7; //back left
	public static final int RIGHT_DRIVE_MOTOR_A_PWM_PORT           = 5; //front right
	public static final int RIGHT_DRIVE_MOTOR_B_PWM_PORT           = 4; //back right*/
	// ***************** PDP CHANNELS *****************
	public static final int LEFT_DRIVE_MOTOR_A_PDP_CHAN           = 12;
	public static final int LEFT_DRIVE_MOTOR_B_PDP_CHAN           = 13;
	public static final int RIGHT_DRIVE_MOTOR_A_PDP_CHAN          = 14;
	public static final int RIGHT_DRIVE_MOTOR_B_PDP_CHAN          = 15;

	// ***************** DIGITAL I/O PORTS *****************
	public static final int LIGHTS_DIO_PORTS[]                    = {2, 3, 4, 5};
	public static final int RIGHT_DRIVE_ENCODER_PORTS[]            = {7, 6};
	//Top Ports
	public static final int LEFT_DRIVE_ENCODER_PORTS[]           = {9, 8};

	

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
