// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private static Vision m_vision;

  private NetworkTable limelight;
  private double tl;
  private boolean tv;

  /** Creates a new Vision. */
  public Vision() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public Pose2d robotPoseVision(){
    double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[6]);
    Pose2d pose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));

    return pose;
  }

  public double getLatency(){
    tl = limelight.getEntry("tl").getDouble(0)/1000.0;
    return tl;
  }

  public boolean isValid(){
    tv = limelight.getEntry("tv").getDouble(0) == 1;
    return tv;
  }

  public static Vision getInstance(){
    if(m_vision == null){
      m_vision = new Vision();
    }
    return m_vision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
