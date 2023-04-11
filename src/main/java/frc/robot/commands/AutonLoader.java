package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutonLoader {
    HashMap<String, Command> eventMap;
    SwerveAutoBuilder autoBuilder;

    AutonLoader(DriveBase driveBase) {
        eventMap = new HashMap<>();
        eventMap.put("event1", new PrintCommand("event 1 passed"));    
        eventMap.put("event1", new PrintCommand("event 2 passed"));    

        autoBuilder = new SwerveAutoBuilder(
                driveBase::getCurrentPose,
                driveBase::resetOdometry,
                Constants.DriveTrainConstants.TRANSLATION_PID,
                Constants.DriveTrainConstants.ROTATION_PID,
                (chassisSpeeds) -> driveBase.setDriveSpeed(chassisSpeeds),
                eventMap,
                driveBase);
        
    }

    
}