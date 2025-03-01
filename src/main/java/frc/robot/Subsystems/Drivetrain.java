// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.firstAutoPose;

import static frc.robot.Constants.DrivetrainConst;

import java.util.function.DoubleSupplier;

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

  //private PIDController m_pid;

  private AHRS m_gyro;

  private DifferentialDriveOdometry m_odometry;

  private DifferentialDriveKinematics m_kinematics;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_gyro = new AHRS(NavXComType.kMXP_SPI);

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
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kCoast).closedLoopRampRate(2.5);
    rigthLeaderConfig.apply(globalConfigs).inverted(true);
    leftFollowerConfigs.apply(globalConfigs).follow(m_leftBack);
    rightFollowerConfigs.apply(globalConfigs).follow(m_rigthBack);

    // Apply the config for each motor
    m_leftBack.configure(globalConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftFront.configure(leftFollowerConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rigthBack.configure(rigthLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rigthFront.configure(rightFollowerConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_rightEncoder = m_rigthBack.getEncoder();
    m_leftEncoder = m_leftFront.getEncoder();
    
    // Create the config for the drivetrain and control all the motors as one motor
    m_drive = new DifferentialDrive(m_leftBack, m_rigthBack);

    m_drive.setSafetyEnabled(false);

    // Create the object that represent the game field
    m_field = new Field2d();
    
    //m_pid = new PIDController(0.01, 0, 0.00001);
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_gyro.getAngle()), 0, 0, firstAutoPose.K_POSE2D);
    
    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(this::getPose, this::resetPose, this::getRobotRelativeSpeeds, (speeds, feedforwards) -> executeDrive(speeds), new PPLTVController(0.02), config, () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }, this);
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
    m_drive.arcadeDrive(-linear, rotation);
  }

  public void drive(ChassisSpeeds chassis){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassis);
    m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // Command used to call the movement, and asign this as the default command for the sybsystem
  public Command executeMove(DoubleSupplier linear, DoubleSupplier rotation){
    return run(() -> move(-linear.getAsDouble(), rotation.getAsDouble()));
  }

  public Command executeDrive(ChassisSpeeds chassis){
    return run(() -> drive(chassis));
  }

  public Command defaulrDrive(DoubleSupplier linear, DoubleSupplier rotation){
    ChassisSpeeds m_chassis = new ChassisSpeeds(linear.getAsDouble(), 0.0, rotation.getAsDouble());
    return run(() -> drive(m_chassis));
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d newPose){
    m_odometry.resetPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    DifferentialDriveWheelSpeeds wheelsSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    return m_kinematics.toChassisSpeeds(wheelsSpeeds);
  }

  @Override
  public void periodic() {
    // Show the data in the smartdashboard
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), new DifferentialDriveWheelPositions(m_leftEncoder.getPosition(), m_rightEncoder.getPosition()));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("odometry", m_field);
    SmartDashboard.putNumber("Left Side", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Side", m_rightEncoder.getPosition());
  }
}
