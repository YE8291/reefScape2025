// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpElevatorSecond extends Command {
  
  private Elevator m_Elevator;

  /** Creates a new UpElevatorSecond. */
  public UpElevatorSecond() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = Elevator.getInstance();
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.setSetpoint(13.5);
    m_Elevator.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Elevator.getPosition() >= 13){
      return true;
    }
    return false;
  }
}
