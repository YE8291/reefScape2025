// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.firstAuto;
import frc.robot.Commands.secondAuto;
import frc.robot.Commands.thirthAuto;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private XboxController m_Controller = new XboxController(0);
  private final SendableChooser<String> m_autoChooser; 
  private final String firstBlueAuto = "fba";
  private final String secondBlueAuto = "sba";
  private final String thirthBlueAuto = "tba";

  public RobotContainer() {
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.addOption("First Blue", firstBlueAuto);
    m_autoChooser.addOption("Second Blue", secondBlueAuto);
    m_autoChooser.addOption("Thirth Blue", thirthBlueAuto);
    m_autoChooser.setDefaultOption("No auto", "NA");
    SmartDashboard.putData(m_autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_drivetrain.executeMove(() -> m_Controller.getLeftY(), () -> m_Controller.getLeftX()));
  }

  public Command getAutonomousCommand() {
    switch (m_autoChooser.getSelected()) {
      case "fba":
        return Commands.sequence(new firstAuto());
      case "sba":
        return Commands.sequence(new secondAuto());
      case "tba":
        return Commands.sequence(new thirthAuto());
      default:
        return Commands.print("No auto command selected");
    }
    //return Commands.print("No autonomous command configured");
  }
}
