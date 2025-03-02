// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {

  private Drivetrain m_drivetrain;
  private CommandXboxController m_Controller;
  private ChassisSpeeds chassisSpeeds;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = Drivetrain.getInstance();
    this.m_Controller = controller;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds = new ChassisSpeeds(m_Controller.getLeftY()*12, 0.0, m_Controller.getRightX()*12);
    m_drivetrain.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
