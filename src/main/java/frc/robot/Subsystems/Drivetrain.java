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
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlueCenter;

import static frc.robot.Constants.DrivetrainConst;

// Drivetrain Subsystem is the system to drive the chassis of the robot
public class Drivetrain extends SubsystemBase {

  // The object motors used in the robot, ours robots have 4 motors in the drivetrain
  private SparkMax m_leftFront;
  private SparkMax m_leftBack;
  private SparkMax m_rigthFront;
  private SparkMax m_rigthBack;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  // This object is used to manage the 4 motors as 1 system 
  private DifferentialDrive m_drive;

  // This constant is used for only have one instance of this subsystem
  private static Drivetrain m_Drivetrain;

  // This object is used to create a representation 2d of the field, for put data in this representation
  private Field2d m_field;

  private PIDController m_leftPid;
  private PIDController m_rigthPid;

  private AHRS m_gyro;

  private DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics m_kinematics;

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

    m_rightEncoder = m_rigthBack.getEncoder();
    m_leftEncoder = m_leftFront.getEncoder();

    // Apply the differents configs, for after apply each motor 
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kBrake).encoder.countsPerRevolution(2048).inverted(false);
    rigthLeaderConfig.apply(globalConfigs).inverted(true).encoder.inverted(true);

    leftFollowerConfigs.apply(globalConfigs).follow(m_leftBack);
    rightFollowerConfigs.apply(globalConfigs).follow(m_rigthBack);

    // Apply the config for each motor
    m_leftBack.configure(globalConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftFront.configure(leftFollowerConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rigthBack.configure(rigthLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rigthFront.configure(rightFollowerConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Create the config for the drivetrain and control all the motors as one motor
    m_drive = new DifferentialDrive(m_leftBack, m_rigthBack);

    m_drive.setSafetyEnabled(false);

    // Create the object that represent the game field
    m_field = new Field2d();
    
    m_gyro = new AHRS(DrivetrainConst.k_gyro);
    m_gyro.setAngleAdjustment(0);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_gyro.getAngle()), 0, 0, 
    BlueCenter.K_POSE2D);
    
    m_kinematics = new DifferentialDriveKinematics(DrivetrainConst.k_robotWidth);

    m_leftPid = new PIDController(0.7, 0, 0.01);
    m_rigthPid = new PIDController(0.7, 0, 0.01);

    //feedsFeedforward = new SimpleMotorFeedforward(1.0636, 1.2066, 0.41989);

    resetEncoders();
    resetPose(BlueCenter.K_POSE2D);
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
  public void driveAuto(double position, double position2){
    //m_leftBack.setVoltage(-m_leftPid.calculate(m_leftEncoder.getPosition(), position));
    //m_rigthBack.setVoltage(-m_rigthPid.calculate(m_rightEncoder.getPosition(), position));
    m_leftBack.setVoltage(position);
    m_rigthBack.setVoltage(position2);
  }

  public void driveTeleop(double xSpeed, double zRotation){
    m_drive.arcadeDrive(xSpeed, zRotation, true);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d newPose){
    m_odometry.resetPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    double estimatedSpeed = (m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity()) / 2;
    double estimatedRotation = Math.toDegrees(m_gyro.getRate());

    return new ChassisSpeeds(estimatedSpeed, 0, estimatedRotation);
  }

  public DifferentialDriveWheelSpeeds getWheelsSpeeds(){
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    return wheelSpeeds;
  }

  public double getVelocityMtsPerSecond(RelativeEncoder encoder){
    double velocity = (encoder.getVelocity() * Math.PI * DrivetrainConst.k_wheelDiam) / 60;
    return velocity;
  }

  public double getVelocityRight(){
    return (m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity())/2;
  }

  public double getPositionRight(){
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition())/2;
  }

  public double getPositionMts(){
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition())/2;
  }

  public double getVoltageRight(){
    return (m_leftBack.getBusVoltage() + m_rigthBack.getBusVoltage())/2;
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void setSetpoint(double setpoint){
    m_leftPid.setSetpoint(setpoint);
    m_rigthPid.setSetpoint(setpoint);
  }

  public double getSetpoint(){
    return m_leftPid.getSetpoint();
  }

  public void setPConst(double kp_left, double kp_rigth){
    m_rigthPid.setP(kp_rigth);
    m_leftPid.setP(kp_left);
  }

  @Override
  public void periodic() {
    // Show the data in the smartdashboard
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()),-((m_leftEncoder.getPosition()/2)*Math.PI*Units.inchesToMeters(6)), -((m_rightEncoder.getPosition()/2)*Math.PI*Units.inchesToMeters(6)));
    m_field.setRobotPose(new Pose2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation()));
    SmartDashboard.putData("odometry", m_field);
    SmartDashboard.putNumber("Left Side", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Side", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("Encoder Value", getPositionMts());
  }
}
