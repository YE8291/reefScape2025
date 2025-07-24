// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmSecondLevel extends Command {
  /** Creates a new MoveArmSecondLevel. */

  private Drivetrain m_drive;
  private Arm m_arm;
  private double posMeters;
  private boolean isEnded = false;
  private boolean isArmActive = false;
  private Timer m_tmr;

  public MoveArmSecondLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drivetrain.getInstance();
    m_arm = Arm.getInstance();
    addRequirements(m_drive);
    addRequirements(m_arm);
    m_tmr = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    posMeters = m_drive.getPositionMts();
    m_tmr.stop();
    m_tmr.reset();
    m_tmr.start();
    m_arm.setSetpoint(1.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isArmActive){
      m_arm.enablePID();
      isArmActive = true;
    }
    if(m_tmr.get() > 1){
      if(m_drive.getPositionMts() >= (posMeters - 2) && isArmActive){
        m_drive.driveAuto(3, 3.15);
      }else{
        if(m_drive.getRotations() <= 0.95 || m_drive.getRotations() >= -0.95){
          m_drive.driveAuto(-2.5, 2.5);
        }else{
          if(m_drive.getPositionMts() < (posMeters - 0.3)){
            m_drive.driveAuto(-2, -2.12);
          }else{
            m_arm.setSetpoint(0);
            if(m_arm.getPosition() > 0.4){
              isEnded = true;
            }
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isEnded;
  }
}
