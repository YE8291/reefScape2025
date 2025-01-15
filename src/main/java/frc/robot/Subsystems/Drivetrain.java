// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConst;

import java.util.function.DoubleSupplier;

// Drivetrain Subsystem is the system to drive the chassis of the robot
public class Drivetrain extends SubsystemBase {

  // The object motors used in the robot, ours robots have 4 motors in the drivetrain
  private SparkMax m_leftFront;
  private SparkMax m_leftBack;
  private SparkMax m_rigthFront;
  private SparkMax m_rigthBack;

  // The object encoders included in all motor in the drivetrain, to use odometry
  private RelativeEncoder m_leftFrontEnc;
  private RelativeEncoder m_leftBackEnc;
  private RelativeEncoder m_rigthFrontEnc;
  private RelativeEncoder m_rigthBackEnc;

  // This object is used to manage the 4 motors as 1 system 
  private DifferentialDrive m_drive;

  // This object simulate the drivetrain, only for use in the simulator
  private DifferentialDrivetrainSim m_driveSim;

  // This constant is used for only have one instance of this subsystem
  private static Drivetrain m_Drivetrain;

  // This object is used to create a representation 2d of the field, for put data in this representation
  private Field2d m_field;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Create the motor objects, with can id and motortype
    m_leftBack = new SparkMax(DrivetrainConst.k_leftBack, DrivetrainConst.k_motorType);
    m_leftFront = new SparkMax(DrivetrainConst.k_leftFront, DrivetrainConst.k_motorType);
    m_rigthBack = new SparkMax(DrivetrainConst.k_rigthBack, DrivetrainConst.k_motorType);
    m_rigthFront = new SparkMax(DrivetrainConst.k_rigthFront, DrivetrainConst.k_motorType);
    
    // Create configs objects for each motor
    SparkMaxConfig globalConfigs = new SparkMaxConfig();
    SparkMaxConfig rigthLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfigs = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfigs = new SparkMaxConfig();

    // Apply the differents configs, for after apply each motor 
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kCoast);
    rigthLeaderConfig.apply(globalConfigs).inverted(true);
    leftFollowerConfigs.apply(globalConfigs).follow(m_leftBack);
    rightFollowerConfigs.apply(globalConfigs).follow(m_rigthBack);

    // Apply the config for each motor
    m_leftBack.configure(globalConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFront.configure(leftFollowerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rigthBack.configure(rigthLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rigthFront.configure(rightFollowerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the config for the drivetrain and control all the motors as one motor
    m_drive = new DifferentialDrive(m_leftBack, m_rigthBack);

    // Get the included encoder with each motor
    m_leftFrontEnc = m_leftFront.getEncoder();
    m_leftBackEnc = m_leftBack.getEncoder();
    m_rigthFrontEnc = m_rigthFront.getEncoder();
    m_rigthBackEnc = m_rigthBack.getEncoder();

    // Create the simulated drivetrain, only for use with the simulator
    m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      10.71, 
      7.5, 
      15, 
      Units.inchesToMeters(6), 
      0.7112, 
      null
    );

    // Create the object that represent the game field
    m_field = new Field2d();
    
    // Put the robot start position in the game field
    m_field.setRobotPose(1.5, 5.5, Rotation2d.k180deg);
    m_driveSim.setPose(new Pose2d(1.5, 5.5, Rotation2d.k180deg));
  }

  // Method used for get the only one instance of the subsystem
  public static Drivetrain getInstance(){
    if(m_Drivetrain == null){
      // Create the instance, only if the instance has not been created
      m_Drivetrain = new Drivetrain();
    }
    // return the same instance subsystem for each call
    return m_Drivetrain;
  }
  
  // This method is used to execute the move of the drivetrain
  public void move(double linear, double rotation){
    // Execute the move of the drivetrain
    m_drive.arcadeDrive(linear, rotation);
    // This code is used for the simulation
    // This vars save the cleanest double for velocity in each side 
    double linearSpeed = Math.copySign(linear * linear, linear);
    double rotationSpeed = Math.copySign(rotation * rotation, rotation);
    // Apply the velocity to each side (because the simulated drivetrain don't have a direct method to do this)
    m_driveSim.setInputs((linearSpeed + rotationSpeed)*6  , (linearSpeed - rotationSpeed)*6);
    // Update the simulator
    m_driveSim.update(0.02);
    // Update the pose in the 2d field
    m_field.setRobotPose(m_driveSim.getPose());
  }

  // Command used to call the movement, and asign this as the default command for the sybsystem
  public Command executeMove(DoubleSupplier linear, DoubleSupplier rotation){
    return run(() -> move(-linear.getAsDouble(), rotation.getAsDouble()));
  }

  // Method to get the data readed for the physic encoders, in a array
  public double[] getEncoders(){
    double[] positions = {m_leftFrontEnc.getPosition(), m_leftBackEnc.getPosition(), m_rigthFrontEnc.getPosition(), m_rigthBackEnc.getPosition()};
    return positions;
  }

  // Method to get the data readed from the simulated encoders in a array
  public double[] getSimulatedEncoders(){
    double[] positions = {m_driveSim.getLeftPositionMeters(), m_driveSim.getRightPositionMeters()};
    return positions;
  }

  @Override
  public void periodic() {
    // Show the data in the smartdashboard
    SmartDashboard.putNumber("sim left enc", m_driveSim.getLeftPositionMeters());
    SmartDashboard.putNumber("sim rigth enc", m_driveSim.getRightPositionMeters());
    SmartDashboard.putData("field", m_field);
  }
}
