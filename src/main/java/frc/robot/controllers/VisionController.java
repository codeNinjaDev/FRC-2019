package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Params;
import frc.robot.hardware.*;

public class VisionController extends Subsystem {
	private boolean is_enabled, driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen;
	public NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw, driveWanted, tapeWanted, cargoWanted,
			videoTimestamp;
	private double targetAngle, timestamp, cargoAngle, tapeAngle;
	private int left_contour, right_contour;
	private RemoteControl humanControl;
	private Timer visionTimer;
	public Servo cameraMount;
	NetworkTableInstance instance;
	NetworkTable chickenVision;
	private static final double kMaxServoAngle = 270.0;
	private static final double kMinServoAngle = 0.0;
  
	public VisionController(RemoteControl humanControl, Timer visionTimer) {
		this.humanControl = humanControl;
		this.visionTimer = visionTimer;
		cameraMount = new Servo(Ports.CAMERA_SERVO);
		instance = NetworkTableInstance.getDefault();
		instance.addConnectionListener(event -> {
			visionTimer.start();
		}, true);
		chickenVision = instance.getTable("ChickenVision");

		tapeDetected = chickenVision.getEntry("tapeDetected");
		cargoDetected = chickenVision.getEntry("cargoDetected");
		tapeYaw = chickenVision.getEntry("tapeYaw");
		cargoYaw = chickenVision.getEntry("cargoYaw");

		driveWanted = chickenVision.getEntry("Driver");
		tapeWanted = chickenVision.getEntry("Tape");
		cargoWanted = chickenVision.getEntry("Cargo");

		videoTimestamp = chickenVision.getEntry("VideoTimestamp");

		is_enabled = tapeVision = cargoVision = false;
		driverVision = true;
	}

	public void reset() {

	}

	public void update() {

		cargoSeen = cargoDetected.getBoolean(false);
		tapeSeen = tapeDetected.getBoolean(false);

		if (tapeSeen)
			tapeAngle = tapeYaw.getDouble(0);
		if (cargoSeen)
			cargoAngle = cargoYaw.getDouble(0);

		if (!humanControl.climbDesired()) {
			if (humanControl.getCargoVisionDesired()) {
				driverVision = false;
				tapeVision = false;
				cargoVision = true;
				cameraMount.setAngle(Params.camera_cargo);
				driveWanted.setBoolean(driverVision);
				tapeWanted.setBoolean(tapeVision);
				cargoWanted.setBoolean(cargoVision);
				SmartDashboard.putBoolean("Cargo detected", cargoDetected.getBoolean(false));

			} else if (humanControl.getTapeVisionDesired()) {
				cameraMount.setAngle(Params.camera_tape);

				driverVision = false;
				tapeVision = true;
				cargoVision = false;

				driveWanted.setBoolean(driverVision);
				tapeWanted.setBoolean(tapeVision);
				cargoWanted.setBoolean(cargoVision);

			} else {
				cameraMount.setAngle(Params.camera_tape);

				driverVision = false;
				tapeVision = true;
				cargoVision = false;

				driveWanted.setBoolean(driverVision);
				tapeWanted.setBoolean(tapeVision);
				cargoWanted.setBoolean(cargoVision);
			}
		} else {
			cameraMount.setAngle(Params.camera_climb);

			driverVision = true;
			tapeVision = false;
			cargoVision = false;

			driveWanted.setBoolean(driverVision);
			tapeWanted.setBoolean(tapeVision);
			cargoWanted.setBoolean(cargoVision);
		}
		SmartDashboard.putNumber("Camera Angle", cameraMount.getAngle());
		SmartDashboard.putNumber("Cargo Yaw", targetAngle);

	}

	public double targetYaw() {
		if (cargoVision) {
			return cargoYaw();
		} else if (tapeVision) {
			return tapeYaw();
		} else {
			return 0;
		}

	}

	public double cargoYaw() {
		update();
		if (cargoVision && cargoSeen)
			return cargoAngle;
		else
			return 0;
	}

	public double tapeYaw() {
		update();
		if (tapeVision && tapeSeen)
			return tapeAngle;
		else
			return 0;
	}

	public void enable() {
		is_enabled = true;
	}

	public void disable() {
		is_enabled = false;
	}

	public boolean isEnabled() {
		return is_enabled;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

	private void startTimer() {
		visionTimer.start();
	}

	  /**
   * Get the servo angle.
   *
   * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
   * test).
   *
   * @return The angle in degrees to which the servo is set.
   */
  public double getAngle() {
    return cameraMount.getPosition() * getServoAngleRange() + kMinServoAngle;
  }

  private double getServoAngleRange() {
    return kMaxServoAngle - kMinServoAngle;
  }

  /**
   * Set the servo angle.
   *
   * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
   * test).
   *
   * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
   * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
   * less than X result in an angle of X being set and angles of more than Y degrees result in an
   * angle of Y being set.
   *
   * @param degrees The angle in degrees to set the servo.
   */
  public void setAngle(double degrees) {
    if (degrees < kMinServoAngle) {
      degrees = kMinServoAngle;
    } else if (degrees > kMaxServoAngle) {
      degrees = kMaxServoAngle;
    }

    cameraMount.setPosition(((degrees - kMinServoAngle)) / getServoAngleRange());
  }
}
