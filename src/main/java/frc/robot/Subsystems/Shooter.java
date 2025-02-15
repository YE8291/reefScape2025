// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConst;

public class Shooter extends SubsystemBase {
  
  private SparkMax m_leaderMotor;
  private SparkMax m_slaveMotor;

  private static Shooter m_Shooter;

  /** Creates a new Shooter. */
  public Shooter() {
    m_leaderMotor = new SparkMax(ShooterConst.k_firstEng, ShooterConst.k_motorType);
    m_slaveMotor = new SparkMax(ShooterConst.k_secondEng, ShooterConst.k_motorType);

    SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    SparkMaxConfig m_slaveConfig = new SparkMaxConfig();

    m_leaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast);
    m_slaveConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    m_leaderMotor.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_slaveMotor.configure(m_slaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void shoot(){
    m_leaderMotor.set(0.18);
  }

  public void take(){
    m_leaderMotor.set(-0.18);
  }

  public void stop(){
    m_leaderMotor.set(0);
  }

  public static Shooter getInstance(){
    if(m_Shooter == null){
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
