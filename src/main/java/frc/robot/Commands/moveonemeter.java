// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveonemeter extends Command {

  private Drivetrain m_drive;

  /** Creates a new moveonemeter. */
  public moveonemeter() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drivetrain.getInstance();
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drive.getLeftMeters() > -1){
      m_drive.driveTeleop(0.3, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveTeleop(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_drive.getLeftMeters() < -1){
      return true;
    }
    return false;
  }
}
