package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.Autons.Mobility;
import frc.robot.Autons.Scoring;

// This file will be used to store auton commands (activating intake, etc.)
public class AutonCommands {

    public final DriveBase m_drivebase;
    public final Manipulator m_manipulator;
    public final Mobility MOBILITY;
    public final Scoring SCORING;

    public AutonCommands(DriveBase dBase, Manipulator manip) {
        m_drivebase = dBase;
        m_manipulator = manip;
        MOBILITY = new Mobility(m_drivebase, m_manipulator);
        SCORING = new Scoring(m_drivebase, m_manipulator);
    }
}