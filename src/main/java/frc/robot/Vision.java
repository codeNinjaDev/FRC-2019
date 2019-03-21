/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Drivetrain;
import frc.robot.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.SynchronousPID;
public class Vision {

    Elevator elevator = new Elevator();

    //Make sure auto vision is called in a timed loop; I have used this before but I have heard good things
    SynchronousPID vPid;
    private NetworkTableEntry tapeDetected, tapeDistance, tapeYaw;
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable vision = instance.getTable("Vision");
    Drivetrain mecanumDrivetrain = new Drivetrain();
    XboxController controller = new XboxController(0);
    XboxController controller2 = new XboxController(1);

    public void init() {
        //Tune these PID Values
        vPid = new SynchronousPID(.4, 0, .01);
        //Max speed when turning from vision
        vPid.setOutputRange(-0.3, 0.3);
        
    }

    //Probably don't use because tape distance doesn't work
    public void auto_vision() {
        NetworkTableEntry tapeDetected = vision.getEntry("tapeDetected");
        NetworkTableEntry tapeDistance = vision.getEntry("tapeDistance");
        NetworkTableEntry tapeYaw = vision.getEntry("tapeYaw");
        if(tapeDetected.getBoolean(true)) {
            if(tapeDistance.getDouble(0) < 50) {
                if(tapeYaw.getDouble(0) < -1) {
                    mecanumDrivetrain.driveRight();
                } else if (tapeYaw.getDouble(0) > 1) {
                    mecanumDrivetrain.driveLeft();
                }
            } else {
                mecanumDrivetrain.driveStraight();
            }
          }

        //   else {
        //     mecanumDrivetrain.driveStraight();
        //   }

    }

    public void assist_vision() {
        vPid.setSetpoint(0); //0 is where the target is aligned
        NetworkTableEntry tapeDetected = vision.getEntry("tapeDetected");
        NetworkTableEntry tapeYaw = vision.getEntry("tapeYaw");
        NetworkTableEntry tapePitch = vision.getEntry("tapePitch");

        double kP = 1.2;

        if((tapeDetected.getBoolean(true)) && (controller2.getBButton())) {
            // if(tapeYaw.getDouble(0) < -1) {
            //     mecanumDrivetrain.rotateLeft();
            // }
            // else if (tapeYaw.getDouble(0) > 1) {
            //     mecanumDrivetrain.rotateRight();
            // }
            double targetAngle = tapeYaw.getDouble(0);
            double output = vPid.calculate(targetAngle);
            mDrive.driveCartesian(0,0,output);
            // if (targetAngle < .5) {
            //     //mecanumDrivetrain.differential(-.4);
            //     // mecanumDrivetrain.dDrive.arcadeDrive(0, -.35);
            //     mDrive.driveCartesian(0,0,-.35)
            // } else if (targetAngle > 1){
            //     // mecanumDrivetrain.dDrive.arcadeDrive(0, .35);
            //     mDrive.driveCartesian(0,0,.35)
            //
            // } else {
            //     // mecanumDrivetrain.dDrive.arcadeDrive(0, 0);
            // }

            // if (tapePitch.getDouble(0) < -1) {
            //     elevator.up();
            // }
            // else if (tapePitch.getDouble(0) > 1) {
            //     elevator.down();
            // }
        } else {
            //Maybe uncomment next line --------
            //mDrive.driveCartesian(0,0,0);

        }


    }



}
