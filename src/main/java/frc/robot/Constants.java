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
        public final static int kLeftBack = 3;
        public final static int kLeftFront = 4;
        public final static int kRigthBack = 1;
        public final static int kRigthFront = 2;
        public final static int kMaxSpeed = 3;
        public final static MotorType kMotorType = MotorType.kBrushed;
        public final static NavXComType kGyro = NavXComType.kMXP_SPI;
        public final static double kRobotWidth = Units.inchesToMeters(27);
        public final static double kGearRatio = 8.460;
        public final static double kWheelDiam = Units.inchesToMeters(6);
        public final static double kVelFactor = (Math.PI * kWheelDiam) / 60;
        public final static double kPosFactor = Math.PI * kWheelDiam;
        public final static int kCPR = 2048;
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
