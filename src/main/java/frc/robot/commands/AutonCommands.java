package frc.robot.commands;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.Autons.Elevation;
import frc.robot.Autons.Mobility;
import frc.robot.Autons.Scoring;

// This file will be used to store auton commands (activating intake, etc.)
public class AutonCommands {

    public final DriveBase m_drivebase;
    public final Manipulator m_manipulator;
    public final Elevator m_elevator;
    public final Mobility MOBILITY;
    public final Scoring SCORING;
    public final Elevation ELEVATION;

    public AutonCommands(DriveBase dBase, Manipulator manip, Elevator elevator) {
        m_drivebase = dBase;
        m_manipulator = manip;
        m_elevator = elevator;
        MOBILITY = new Mobility(m_drivebase, m_manipulator);
        SCORING = new Scoring(m_drivebase, m_manipulator);
        ELEVATION = new Elevation(m_drivebase, m_manipulator, m_elevator);
    }
}