// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DownElevator;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.StopElevator;
import frc.robot.Commands.Take;
import frc.robot.Commands.UpElevator;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private CommandXboxController m_Controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_drivetrain.executeMove(() -> m_Controller.getLeftY(), () -> m_Controller.getLeftX()));
    m_Controller.a().whileTrue(new UpElevator()).whileFalse(new StopElevator());
    m_Controller.b().whileTrue(new DownElevator()).whileFalse(new StopElevator());
    m_Controller.x().whileTrue(new Shoot());
    m_Controller.y().whileTrue(new Take());
  }

  public Command getAutonomousCommand() {
    /*switch (m_autoChooser.getSelected()) {
      case "fba":
        return Commands.sequence(new firstAuto());
      case "sba":
        return Commands.sequence(new secondAuto());
      case "tba":
        return Commands.sequence(new thirthAuto());
      default:
        return Commands.print("No auto command selected");
    }*/
    return new PathPlannerAuto("auto");
    //return Commands.print("No autonomous command configured");
  }
}
