// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  private double leftSpeed;
  private double rigthSpeed;

  private SimpleMotorFeedforward feedsFeedforward;

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
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kBrake).closedLoopRampRate(2.5);
    rigthLeaderConfig.apply(globalConfigs).inverted(true).encoder.velocityConversionFactor((Math.PI * Units.inchesToMeters(6))/60);
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
    
    m_gyro = new AHRS(DrivetrainConst.k_gyro);
    m_gyro.setAngleAdjustment(BlueCenter.k_rotation.getDegrees());

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_gyro.getAngle()), getLeftMeters(), getRigthMeters(), BlueCenter.K_POSE2D);
    
    m_kinematics = new DifferentialDriveKinematics(DrivetrainConst.k_robotWidth);

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(this::getPose, this::resetPose, this::getRobotRelativeSpeeds, (speeds, feedforwards) -> driveAuto(speeds, feedforwards), new PPLTVController(0.02), config, () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }, this);

    m_leftPid = new PIDController(0.1, 0, 0);
    m_rigthPid = new PIDController(0.1, 0, 0);

    feedsFeedforward = new SimpleMotorFeedforward(4.87, 0.238, 0.665);
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
  public void driveAuto(ChassisSpeeds chassis, DriveFeedforwards feedforwards){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassis);

    //leftSpeed = m_leftPid.calculate(m_leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
    //rigthSpeed = m_rigthPid.calculate(m_rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
    leftSpeed = m_leftPid.calculate(m_leftBack.getBusVoltage(), feedsFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    rigthSpeed = m_rigthPid.calculate(m_rigthBack.getBusVoltage(), feedsFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));

    m_leftBack.setVoltage(wheelSpeeds.leftMetersPerSecond);
    m_rigthBack.setVoltage(wheelSpeeds.rightMetersPerSecond);

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
    DifferentialDriveWheelSpeeds wheelsSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    return m_kinematics.toChassisSpeeds(wheelsSpeeds);
  }

  public double encoderCountsToMts(double encoderCounts){
    double wheelRotation = encoderCounts;
    double distance = wheelRotation * (Math.PI * DrivetrainConst.k_wheelDiam);
    return distance;
  }

  public double getLeftMeters(){
    return encoderCountsToMts(m_leftEncoder.getPosition());
  }

  public double getRigthMeters(){
    return encoderCountsToMts(m_rightEncoder.getPosition()); // Lectura en tics
  }

  public double encoderVelToMeterPerSecond(double encoderVel){
    double vel = (encoderVel*(Units.inchesToMeters(6)*Math.PI))/60;
    return vel;
  }

  @Override
  public void periodic() {
    // Show the data in the smartdashboard
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), getLeftMeters(), getRigthMeters());
    m_field.setRobotPose(new Pose2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation()));
    SmartDashboard.putData("odometry", m_field);
    SmartDashboard.putNumber("Left Side", leftSpeed);
    SmartDashboard.putNumber("Right Side", rigthSpeed);
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("LEFT PID", getLeftMeters());
    SmartDashboard.putNumber("RIGTH PID", getRigthMeters());
  }
}
