package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.TeleopDrive;

public class Systems {
    
    private final static Module[] modules = new Module[Constants.DriveTrainConstants.wheels];
      public final ModuleGroup moduleGroup;
    
    
      public final DriveBase driveBase;
      public final Kinematics kinematics;
      public final AutonLoader autonLoader;
      public final TeleopDrive teleopDrive;

    public Systems() {
        for (int i = 0; i < Constants.DriveTrainConstants.wheels; i++)
            modules[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);
        moduleGroup = new ModuleGroup(modules);

        driveBase = new DriveBase(this);
        kinematics = new Kinematics();
        autonLoader = new AutonLoader(this);
        teleopDrive = new TeleopDrive(this);
    }
}
