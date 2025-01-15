// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class Constants {
    public static class DrivetrainConst{
        public final static int k_leftBack = 1;
        public final static int k_leftFront = 2;
        public final static int k_rigthBack = 3;
        public final static int k_rigthFront = 4;
        public final static MotorType k_motorType = MotorType.kBrushless;
    }
}
