// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {
    public static class DrivetrainConst{
        public final static int k_leftBack = 1;
        public final static int k_leftFront = 2;
        public final static int k_rigthBack = 3;
        public final static int k_rigthFront = 4;
        public final static MotorType k_motorType = MotorType.kBrushless;
    }

    public static class firstAutoPose{
        public final static int k_xPose = 8;
        public final static int k_yPose = 4;
        public final static Rotation2d k_rotation = Rotation2d.kZero;
        public final static Pose2d K_POSE2D = new Pose2d(k_xPose, k_yPose, k_rotation);
    }

    public static class secondAutoPose{
        public final static int k_xPose = 8;
        public final static int k_yPose = 1;
        public final static Rotation2d k_rotation = Rotation2d.kZero;
        public final static Pose2d K_POSE2D = new Pose2d(k_xPose, k_yPose, k_rotation);
    }

    public static class thirthAutoPose{
        public final static int k_xPose = 8;
        public final static int k_yPose = 7;
        public final static Rotation2d k_rotation = Rotation2d.kZero;
        public final static Pose2d K_POSE2D = new Pose2d(k_xPose, k_yPose, k_rotation);
    }
}
