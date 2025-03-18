// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConst;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;

import choreo.trajectory.DifferentialSample;

// Drivetrain Subsystem is the system to drive the chassis of the robot
public class Drivetrain extends SubsystemBase {

  private static Drivetrain m_Drivetrain; 

  private SparkMax m_leaderLeft;
  private SparkMax m_leaderRight;

  private SparkMax m_followerLeft;
  private SparkMax m_followerRight;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rigthEncoder;

  private DifferentialDrive m_drive;

  private AHRS m_gyro;

  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDrivePoseEstimator m_odometry;
  private Field2d m_field;

  private LTVUnicycleController controller;

  private PIDController m_pidTank;

  private Vision m_vision;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_leaderLeft = new SparkMax(DrivetrainConst.kLeftBack, DrivetrainConst.kMotorType);
    m_leaderRight = new SparkMax(DrivetrainConst.kRigthBack, DrivetrainConst.kMotorType);
    m_followerLeft = new SparkMax(DrivetrainConst.kLeftFront, DrivetrainConst.kMotorType);
    m_followerRight = new SparkMax(DrivetrainConst.kRigthFront, DrivetrainConst.kMotorType);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).voltageCompensation(0.3)
    .encoder.countsPerRevolution(DrivetrainConst.kCPR).velocityConversionFactor(DrivetrainConst.kVelFactor).positionConversionFactor(DrivetrainConst.kPosFactor);
    rightLeaderConfig.apply(globalConfig).inverted(false).disableVoltageCompensation()
    .encoder.countsPerRevolution(DrivetrainConst.kCPR).velocityConversionFactor(DrivetrainConst.kVelFactor).positionConversionFactor(DrivetrainConst.kPosFactor);

    leftFollowerConfig.apply(globalConfig).follow(m_leaderLeft).voltageCompensation(0.3);
    rightFollowerConfig.apply(rightLeaderConfig).follow(m_leaderRight).disableVoltageCompensation();

    m_leaderLeft.configure(globalConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_leaderRight.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_followerLeft.configure(leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_followerRight.configure(rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_leftEncoder = m_leaderLeft.getEncoder();
    m_rigthEncoder = m_leaderRight.getEncoder();
    resetEncoders();

    m_drive = new DifferentialDrive(m_leaderLeft, m_leaderRight);
    m_drive.setSafetyEnabled(false);

    m_gyro = new AHRS(DrivetrainConst.kGyro);
    resetGyro(); 

    m_kinematics = new DifferentialDriveKinematics(DrivetrainConst.kRobotWidth);
    m_odometry = new DifferentialDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rigthEncoder.getPosition(), getPose());

    controller = new LTVUnicycleController(0.02);

    m_pidTank = new PIDController(0.1, 0, 0);

    m_vision = Vision.getInstance();

    SmartDashboard.putData(m_field);
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

  public void driveTeleop(double xSpeed, double zRot){
    //m_drive.arcadeDrive(xSpeed, zRot, true);
    driveAuto(new ChassisSpeeds(xSpeed, 0, zRot));
  }

  public void driveAuto(ChassisSpeeds drive){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(drive);
    double leftSide = m_pidTank.calculate(m_leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rigthSide = m_pidTank.calculate(m_rigthEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);

    m_leaderLeft.setVoltage(leftSide);
    m_leaderRight.setVoltage(rigthSide);
  }

  public void followTrajectory(DifferentialSample sample){
    Pose2d pose = getPose();
    ChassisSpeeds ff = sample.getChassisSpeeds();
    ChassisSpeeds speeds = controller.calculate(
      pose,
      sample.getPose(),
      ff.vxMetersPerSecond, 
      ff.omegaRadiansPerSecond
    );

    driveAuto(speeds);
  }

  public void resetGyro(){
    m_gyro.reset();
  }
  
  public void setGyroCompesation(boolean isBlue){
    if(isBlue){
      m_gyro.setAngleAdjustment(180);
    }else{
      m_gyro.setAngleAdjustment(0);
    }
  }
  
  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

  public Pose2d getPose(){
    return m_odometry.getEstimatedPosition();
  }

  public void setPose(Pose2d pose){
    m_odometry.resetPose(pose);
  }

  public void updateOdometry(){
    double latency = m_vision.getLatency();
    if(m_vision.isValid() && latency < 0.2){
      m_odometry.addVisionMeasurement(m_vision.robotPoseVision(), System.currentTimeMillis()/1000 - latency);
    }
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rigthEncoder.getPosition());
    m_field.setRobotPose(m_odometry.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    updateOdometry(); 
    SmartDashboard.putNumber("Left Encoder Vel", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Vel", m_rigthEncoder.getVelocity());
    SmartDashboard.putNumber("Left Encoder Pos", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Vel", m_rigthEncoder.getVelocity());
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
  }
}
