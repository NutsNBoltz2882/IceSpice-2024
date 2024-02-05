// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity
 */
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;

public final class Constants {

  public static final class ModuleConstants{
    //fix all of these dimensions
    public static final double kWheelDiameterMeters = 0.102; // 4 in converted to m cause units class wont work
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4286;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
  }
    public static final class DriveConstants{
      //fix all port numbers and max speeds
       
      public static final double kTrackWidth = 0.3683;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.5461;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
      
      
      
      public static final int kFrontLeftDriveMotorPort = 8;
      public static final int kBackLeftDriveMotorPort = 3;
      public static final int kFrontRightDriveMotorPort = 6;
      public static final int kBackRightDriveMotorPort = 5;

      public static final int kFrontLeftTurningMotorPort = 7;
      public static final int kBackLeftTurningMotorPort = 1;
      public static final int kFrontRightTurningMotorPort = 4;
      public static final int kBackRightTurningMotorPort = 2;

      public static final boolean kFrontLeftTurningEncoderReversed = true;
      public static final boolean kBackLeftTurningEncoderReversed = true;
      public static final boolean kFrontRightTurningEncoderReversed = true;
      public static final boolean kBackRightTurningEncoderReversed = true;

      public static final boolean kFrontLeftDriveEncoderReversed = true;
      public static final boolean kBackLeftDriveEncoderReversed = true;
      public static final boolean kFrontRightDriveEncoderReversed = false;
      public static final boolean kBackRightDriveEncoderReversed = false;

      public static final int kFrontLeftDriveAbsoluteEncoderID = 3;
      public static final int kBackLeftDriveAbsoluteEncoderID = 2;
      public static final int kFrontRightDriveAbsoluteEncoderID = 4;
      public static final int kBackRightDriveAbsoluteEncoderID = 1;

      public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

      public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
      public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
      public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
      public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

      public static final double kPhyscialMaxSpeedMetersPerSecond = 5;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.0;

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhyscialMaxSpeedMetersPerSecond / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    }

    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDeadband = 0.05;
      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;


    }
}
