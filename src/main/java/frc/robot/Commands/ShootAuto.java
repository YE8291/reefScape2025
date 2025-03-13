// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAuto extends Command {
  /** Creates a new ShootAuto. */

  Shooter m_Shooter;
  Timer m_tmr;

  public ShootAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = Shooter.getInstance();
    addRequirements(m_Shooter);
    m_tmr = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tmr.stop();
    m_tmr.reset();
    m_tmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.let();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_tmr.get() < 1.5){
      return false;
    }
    return true;
  }
}
