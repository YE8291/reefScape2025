// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Optional<Trajectory<DifferentialSample>> trajectory = Choreo.loadTrajectory("auto");
  private Drivetrain m_drive = Drivetrain.getInstance();
  private final Timer m_tmr = new Timer();

  public Robot() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    if(trajectory.isPresent()){
      Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

      if(initialPose.isPresent()){
        m_drive.setPose(initialPose.get());
      }
    }
    m_drive.setGyroCompesation(!isRedAlliance());
    m_tmr.reset();
  }

  @Override
  public void autonomousPeriodic() {
    if(trajectory.isPresent()){
      Optional<DifferentialSample> sample = trajectory.get().sampleAt(m_tmr.get(), isRedAlliance());

      if(sample.isPresent()){
        m_drive.followTrajectory(sample.get());
      }
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public boolean isRedAlliance(){
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
