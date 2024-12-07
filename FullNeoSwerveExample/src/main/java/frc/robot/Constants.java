// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static double kCubic = 0.95;
    public static double kLinear = 0.05;
    public static double kDeadband = 0.02;
  }

  public static class GlobalConstants {
    public static final double kLoopTime = 0.020;
  }

  public static class ModuleConstants {
    public static final double kPinion = 12.0;
    public static final double kOutput = 18.0;
    public static final double kWheelDiameter = /*put in inches */ 3.875 / 39.37;
    public static final double kGearRatio = (32.0/kPinion) * (kOutput/24.0) * (45.0/15.0);
    public static final double kDrivingVeloConversionFactor = (1.0/kGearRatio) * (1.0/60.0) * kWheelDiameter * Math.PI;
    public static final double kDrivingPoseConversionFactor = kDrivingVeloConversionFactor * 60;
  }

  public static class DriveConstants{

    public static final class FrontLeft{
      public static final int kModuleID = 4;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength/2,kWheelBaseWidth/2);
    }

     public static final class FrontRight{
      public static final int kModuleID = 1;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength/2,(kWheelBaseWidth/2) * -1);
    }

     public static final class RearLeft{
      public static final int kModuleID = 3;
      public static final Translation2d kLocation = new Translation2d((kWheelBaseLength/2) * -1,kWheelBaseWidth/2);
    }

     public static final class RearRight{
      public static final int kModuleID = 2;
      public static final Translation2d kLocation = new Translation2d((kWheelBaseLength/2) * -1,(kWheelBaseWidth/2) * -1);
    }

    public static final double kFrameLength = 32.0;
    public static final double kFrameWidth = 27.0;
    public static final double kWheelBaseWidth = kFrameWidth - 5.0;
    public static final double kWheelBaseLength = kFrameLength - 5.0;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      FrontLeft.kLocation,
      FrontRight.kLocation,
      RearLeft.kLocation,
      RearRight.kLocation
    );
    

  }
}
