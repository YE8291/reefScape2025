// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConst;

public class Elevator extends SubsystemBase {

  private SparkMax m_firstMotor;
  private SparkMaxSim m_firstMotorSim;

  //private RelativeEncoder m_encoder;

  private PIDController m_pid;
  private boolean m_isEnable = false;

  private static Elevator m_Elevator;

  /** Creates a new Elevator. */
  public Elevator() {
    m_firstMotor = new SparkMax(ElevatorConst.k_firstEng, ElevatorConst.k_motorType);
    //m_firstMotorSim = new SparkMaxSim(m_firstMotor, DCMotor.getNEO(1));

    SparkMaxConfig leaderMotor = new SparkMaxConfig();

    leaderMotor.idleMode(IdleMode.kBrake).smartCurrentLimit(50);

    m_firstMotor.configure(leaderMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    //m_encoder = m_firstMotor.getEncoder();

    m_pid = new PIDController(1, 0, 0.1);
  }

  public void move(double vel){
    m_firstMotor.set(vel);
    //m_firstMotorSim.iterate(vel, 12, 1);
  }

  public void setSetpoint(double sp){
    m_pid.setSetpoint(sp);
  }

  public void enablePID(){
    m_isEnable = true;
  }

  public void disablePID(){
    m_isEnable = false;
  }

  public double getSetpoint(){
    return m_pid.getSetpoint();
  }

  /*public double getPosition(){
    return m_firstMotorSim.getPosition();
  }*/

  public static Elevator getInstance(){
    if(m_Elevator == null){
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Elevator Encoder", m_encoder.getPosition());
    /*double output;
    if(m_isEnable){
      output = m_pid.calculate(m_firstMotorSim.getPosition());
    }else{
      output = 0;
    }
    move(output);*/
  }
}
