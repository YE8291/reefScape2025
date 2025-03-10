// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.DownElevator;
import frc.robot.Commands.LetL1;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.Take;
import frc.robot.Commands.UpElevator;
import frc.robot.Commands.UpElevatorSecond;
import frc.robot.Commands.mausqueherramienta;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private CommandXboxController m_Controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new DefaultDrive(m_Controller));
    m_Controller.x().onTrue(new DownElevator()); // Cambiar al punto 0
    m_Controller.rightTrigger().whileTrue(new Shoot());
    m_Controller.leftBumper().onTrue(new UpElevator()); // Nivel 2 (4.8 encoder)
    //m_Controller.y().whileTrue(new mausqueherramienta());
    m_Controller.leftTrigger().onTrue(new UpElevatorSecond()); // Nivel 3 gatillo izquierdo (14.08)
    m_Controller.rightBumper().whileTrue(new LetL1());

    //m_Controller.x().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //m_Controller.b().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //m_Controller.y().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //m_Controller.a().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("auto");
    //return Commands.sequence(new moveonemeter());
  }

  public Drivetrain getDrivetrain(){
    return m_drivetrain;
  }
}
