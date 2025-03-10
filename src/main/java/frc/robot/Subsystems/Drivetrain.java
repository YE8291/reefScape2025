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

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BlueCenter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
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

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  SysIdRoutine rutina = new SysIdRoutine(
    new SysIdRoutine.Config(),
     new SysIdRoutine.Mechanism(voltage -> {
      m_rigthBack.setVoltage(voltage);
      m_leftBack.setVoltage(voltage);
     }, 
     log -> {
      log.motor("Motor Izq chasis")
      .voltage(m_appliedVoltage.mut_replace(m_leftBack.getAppliedOutput() * m_leftBack.getBusVoltage(), Volts))
      .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
      .linearVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity() / 60, MetersPerSecond));

      log.motor("Motor Der chasis").voltage(m_appliedVoltage.mut_replace(m_rigthBack.getAppliedOutput() * m_rigthBack.getBusVoltage(), Volts))
      .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(), Meters))
      .linearVelocity(m_velocity.mut_replace(m_rightEncoder.getVelocity() / 60, MetersPerSecond));
     }, 
     this));

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
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kBrake).closedLoopRampRate(2.5).encoder.velocityConversionFactor(Math.PI * Units.inchesToMeters(6)).
    positionConversionFactor(Math.PI * Units.inchesToMeters(6));
    rigthLeaderConfig.apply(globalConfigs).inverted(true);

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
    m_gyro.setAngleAdjustment(BlueCenter.k_rotation.getDegrees());

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_gyro.getAngle()), 0, 0, 
    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
    
    m_kinematics = new DifferentialDriveKinematics(DrivetrainConst.k_robotWidth);

    RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        (speeds, feedsFeedForward) -> driveAuto(speeds), 
        new PPLTVController(0.02),
        config, 
        () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    m_leftPid = new PIDController(0, 0, 0);
    m_rigthPid = new PIDController(0, 0, 0);

    feedsFeedforward = new SimpleMotorFeedforward(1.0636, 1.2066, 0.41989);

    resetEncoders();
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
  public void driveAuto(ChassisSpeeds chassis){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassis);

    //leftSpeed = m_leftPid.calculate(m_leftEncoder.getVelocitthlSpeeds.leftMetersPerSecond);
    //rigthSpeed = m_rigthPid.calculate(m_rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
    
    double leftFF = feedsFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFF = feedsFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    m_leftBack.setVoltage(leftFF);
    m_rigthBack.setVoltage(rightFF);

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

  public double getVoltageRight(){
    return (m_leftBack.getBusVoltage() + m_rigthBack.getBusVoltage())/2;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return rutina.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return rutina.dynamic(direction);
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Show the data in the smartdashboard
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()),m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    m_field.setRobotPose(new Pose2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation()));
    SmartDashboard.putData("odometry", m_field);
    SmartDashboard.putNumber("Left Side", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Side", rigthSpeed);
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }
}
