package frc.robot.auto.routines;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.auto.actions.DriveDistanceCommand;
import frc.robot.auto.actions.GyroRotateCommand;
import frc.robot.auto.actions.OuttakePistons;
import frc.robot.auto.actions.VisionAlignCommand;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.VisionController;
import frc.robot.controllers.WristController;

public class RightSideHatch extends CommandGroup {
    private DriveController driveController;
	public RightSideHatch(DriveController driveController, WristController wrist, VisionController vision) {
		this.driveController = driveController;
		addSequential(new DriveDistanceCommand(driveController, 213, 0.7, 4, false));
        addSequential(new GyroRotateCommand(driveController, -90, 2, true));
        addSequential(new VisionAlignCommand(driveController, vision, 2, false));
        addSequential(new DriveDistanceCommand(driveController, 10.5, 0.3, 2, true));
        addSequential(new OuttakePistons(wrist));

	}
}