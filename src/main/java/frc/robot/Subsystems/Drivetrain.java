// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConst;

// Drivetrain Subsystem is the system to drive the chassis of the robot
public class Drivetrain extends SubsystemBase {

  // The object motors used in the robot, ours robots have 4 motors in the drivetrain
  private SparkMax m_leftFront;
  private SparkMax m_leftBack;
  private SparkMax m_rigthFront;
  private SparkMax m_rigthBack;

  // This object is used to manage the 4 motors as 1 system 
  private DifferentialDrive m_drive;

  // This object simulate the drivetrain, only for use in the simulator
  private DifferentialDrivetrainSim m_driveSim;

  // This constant is used for only have one instance of this subsystem
  private static Drivetrain m_Drivetrain;

  // This object is used to create a representation 2d of the field, for put data in this representation
  private Field2d m_field;

  private PIDController m_pidLeft;
  private PIDController m_pidRight;

  private AnalogGyroSim m_gyroSim;

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

    // Apply the differents configs, for after apply each motor 
    globalConfigs.smartCurrentLimit(50).idleMode(IdleMode.kCoast);
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

    // Create the simulated drivetrain, only for use with the simulator
    m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getCIM(2),
      10.71, 
      7.5, 
      43, 
      Units.inchesToMeters(6), 
      Units.inchesToMeters(27), 
      null
    );

    // Create the object that represent the game field
    m_field = new Field2d();

    m_gyroSim = new AnalogGyroSim(1);
    m_gyroSim.setAngle(0);
    
    Pose2d pose = new Pose2d(8, 3.85, Rotation2d.k180deg);
    m_driveSim.setPose(pose);

    m_field.setRobotPose(pose);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.k180deg, 0, 0, pose);
    
    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(this::getPose, this::resetPose, this::getRobotRelativeSpeeds, (speeds, feedforwards) -> drive(speeds), new PPLTVController(0.02), config, () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }, this);

    m_pidLeft = new PIDController(1.5, 0, 0.2);
    m_pidRight = new PIDController(1.5, 0, 0.2);
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
    // This code is used for the simulation
    // This vars save the cleanest double for velocity in each side 
    double linearSpeed = Math.copySign(linear * linear, linear);
    double rotationSpeed = Math.copySign(rotation * rotation, rotation);
    // Apply the velocity to each side (because the simulated drivetrain don't have a direct method to do this)
    m_driveSim.setInputs((linearSpeed + rotationSpeed)*12  , (linearSpeed - rotationSpeed)*12);
  }

  public void drive(ChassisSpeeds drive){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(drive);

    double leftSpeed = m_pidLeft.calculate(m_driveSim.getLeftVelocityMetersPerSecond(), wheelSpeeds.leftMetersPerSecond);
    double rigthSpeed = m_pidRight.calculate(m_driveSim.getRightVelocityMetersPerSecond(), wheelSpeeds.rightMetersPerSecond);

    m_driveSim.setInputs(leftSpeed, rigthSpeed);
  }

  public Pose2d getPose(){
    return m_driveSim.getPose();
  }

  public void resetPose(Pose2d newPose){
    m_driveSim.setPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    DifferentialDriveWheelSpeeds wheelsSpeeds = new DifferentialDriveWheelSpeeds(m_driveSim.getLeftVelocityMetersPerSecond(), m_driveSim.getRightVelocityMetersPerSecond());
    return m_kinematics.toChassisSpeeds(wheelsSpeeds);
  }

  @Override
  public void periodic() {
    m_driveSim.update(0.02);
    // Show the data in the smartdashboard
    SmartDashboard.putNumber("sim left pos", m_driveSim.getLeftPositionMeters());
    SmartDashboard.putNumber("sim rigth pos", m_driveSim.getRightPositionMeters());
    SmartDashboard.putNumber("sim left vel", m_driveSim.getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("sim rigth vel", m_driveSim.getRightVelocityMetersPerSecond());
    SmartDashboard.putNumber("Gyro", m_driveSim.getHeading().getDegrees());
    m_odometry.update(Rotation2d.fromDegrees(m_driveSim.getHeading().getDegrees()), m_driveSim.getLeftPositionMeters(), m_driveSim.getRightPositionMeters());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("field", m_field);
  }
}
