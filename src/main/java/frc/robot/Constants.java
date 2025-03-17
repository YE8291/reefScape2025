// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class DrivetrainConst{
        public final static int k_leftBack = 3;
        public final static int k_leftFront = 4;
        public final static int k_rigthBack = 1;
        public final static int k_rigthFront = 2;
        public final static int k_maxSpeed = 3;
        public final static MotorType k_motorType = MotorType.kBrushed;
        public final static NavXComType k_gyro = NavXComType.kMXP_SPI;
        public final static double k_robotWidth = Units.inchesToMeters(27);
        public final static double k_gearRatio = 8.460;
        public final static double k_wheelDiam = Units.inchesToMeters(6);
        public final static double kS = 0;
        public final static double kV = 0;
        public final static double kA = 0;
    }

    public static class ElevatorConst{
        public final static int k_firstEng = 5;
        public final static MotorType k_motorType = MotorType.kBrushed;
        public final static double kP = 0;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static int k_SensorChannel = 1;
    }

    public static class ShooterConst{
        public final static int k_firstEng = 6;
        public final static int k_secondEng = 7;
        public final static MotorType k_motorType = MotorType.kBrushless;
    }

    public static class BlueCenter{
        public final static int k_xPose = 8;
        public final static double k_yPose = 3.85;
        public final static Rotation2d k_rotation = Rotation2d.kZero;
        public final static Pose2d K_POSE2D = new Pose2d(k_xPose, k_yPose, k_rotation);
    }
}
