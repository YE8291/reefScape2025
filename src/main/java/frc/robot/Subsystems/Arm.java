// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private static Arm m_arm;

  private SparkMax m_motor;
  private RelativeEncoder m_encoder;

  private PIDController m_PidController;

  private boolean m_active = false;

  /** Creates a new Arm. */
  public Arm() {
    m_motor = new SparkMax(8, MotorType.kBrushed);

    SparkMaxConfig configs = new SparkMaxConfig();

    configs.smartCurrentLimit(40).idleMode(IdleMode.kBrake);

    m_PidController = new PIDController(0.6, 0, 0.16);
    m_encoder = m_motor.getEncoder();
  }

  public static Arm getInstance(){
    if(m_arm == null){
      m_arm = new Arm();
    }
    return m_arm;
  }

  public void up(){
    m_motor.set(0.4);
  }

  public void down(){
    m_motor.set(-0.4);
  }

  public void stop(){
    m_motor.set(0);
  }

  public void enablePID(){
    m_active = true;
  }

  public void disablePID(){
    m_active = false;
  }

  public void setSetpoint(double setpoint){
    m_PidController.setSetpoint(setpoint);
  }

  public double getPosition(){
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output;
    if(m_active){
      output = m_PidController.calculate(m_encoder.getPosition());
    }else{
      output = 0;
    }
    m_motor.set(output);
    SmartDashboard.putNumber("Arm Encoder", m_encoder.getPosition());
  }
}
