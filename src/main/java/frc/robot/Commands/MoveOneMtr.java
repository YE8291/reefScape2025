// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveOneMtr extends Command {
  /** Creates a new MoveOneMtr. */

  Drivetrain m_Drive;

  public MoveOneMtr() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Drive = Drivetrain.getInstance();
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drive.setPConst(0.7, 0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.driveAuto(-5.3, -5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.driveAuto(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Drive.getPositionMts() >= 14){
      return true;
    }
    return false;
  }
}
