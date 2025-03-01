// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DownElevator;
import frc.robot.Commands.LetL1;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.Take;
import frc.robot.Commands.UpElevator;
import frc.robot.Commands.mausqueherramienta;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private CommandXboxController m_Controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_drivetrain.executeMove(() -> m_Controller.getLeftY(), () -> m_Controller.getRightX()));
    //m_drivetrain.setDefaultCommand(m_drivetrain.defaulrDrive(() -> m_Controller.getLeftY(), () -> m_Controller.getRightX()));
    m_Controller.a().onTrue(new UpElevator());
    m_Controller.b().onTrue(new DownElevator());
    m_Controller.x().whileTrue(new Take());
    m_Controller.rightTrigger().whileTrue(new Shoot());
    m_Controller.leftBumper().whileTrue(new LetL1());
    m_Controller.leftTrigger().whileTrue(new mausqueherramienta());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
