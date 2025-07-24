// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.DownElevator;
import frc.robot.Commands.LetL1;
import frc.robot.Commands.MoveArmDownWithPulse;
import frc.robot.Commands.MoveArmSecondLevel;
import frc.robot.Commands.MoveArmWithPulse;
import frc.robot.Commands.MoveBack;
import frc.robot.Commands.MoveFromSide;
import frc.robot.Commands.MoveOneMtr;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.ShootAuto;
import frc.robot.Commands.UpElevator;
import frc.robot.Commands.UpElevatorSecond;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private CommandXboxController m_Controller = new CommandXboxController(0);
  private static final String kDefaultAuto = "Center";
  private static final String kCustomAuto = "Side";
  private static final String kSecondAuto = "Algae";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_chooser.setDefaultOption("Center", kDefaultAuto);
    m_chooser.addOption("Side", kCustomAuto);
    m_chooser.addOption("Algae", kSecondAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new DefaultDrive(m_Controller));
    m_Controller.x().onTrue(new DownElevator()); // Cambiar al punto 0
    m_Controller.rightTrigger().whileTrue(new Shoot());
    m_Controller.leftBumper().onTrue(new UpElevator()); // Nivel 2 (2.08 encoder)
    m_Controller.leftTrigger().onTrue(new UpElevatorSecond()); // Nivel 3 gatillo izquierdo (14.13)
    m_Controller.rightBumper().whileTrue(new LetL1());
    m_Controller.a().onTrue(new MoveArmWithPulse());
    m_Controller.b().onTrue(new MoveArmDownWithPulse());
    /*m_Controller.y().onTrue(Commands.sequence(
      new UpElevatorSecond(),
      new MoveArmWithPulse(),
      new DownElevator(),
      new MoveArmDownWithPulse()
    ));*/
  }

  public Command getAutonomousCommand() {
    switch (m_chooser.getSelected()) {
      case "Side":
      return Commands.sequence(
        new MoveFromSide(), 
        new UpElevator(), 
        new ShootAuto(),
        new DownElevator(),
        new MoveBack()
      );

      case "Algae":
      return Commands.sequence(
        new MoveOneMtr(),
        new MoveArmSecondLevel(), 
        new UpElevator(), 
        new ShootAuto(),
        new DownElevator(),
        new MoveBack()
      ); 
    
      default:
        return Commands.sequence(
          new MoveOneMtr(), 
          new UpElevatorSecond(), 
          new ShootAuto(),
          new DownElevator(),
          new MoveBack()
        );
    }
  }
}
