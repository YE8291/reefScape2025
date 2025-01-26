// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class thirthAuto extends Command {

  private Drivetrain m_drive;

  private boolean m_status = false;

  /** Creates a new thirthAuto. */
  public thirthAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drivetrain.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setPose(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_status = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_status;
  }
}
