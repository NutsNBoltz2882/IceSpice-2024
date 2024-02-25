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
    // fix all of these dimensions - double checking these values
    //wheel diameter in meters WHEEL_DIAMETER_METERS
    // we have 4in (4") wheels to convert to meters because units class won't work
    //public static final double kWheelDiameterMeters = 0.102;  // .1016 .. rounded to 102
    public static final double kWheelDiameterMeters = 0.1016;  // .1016 .. rounded to 102

    // Circumference of the wheel = (C = pi * diameter)
    // ****** For NEOs, pulsesPerRotation = 42
    // Mk4 wheelDiameter = Units.inchesToMeters(4.0)

    //metersPerPulse = gearRatio * (Math.PI * wheelDiameter) / pulsesPerRotation;
    //wheelPositionMeters = encoderPosition * metersPerPulse (you should probably just use the set conversion factor methods on the encoder object)

    // Turning Motor Gear Ratio Calculations
    // mk4i 150/7:1 for every 150 spins of the motor shaft,the swerve module will spin 7 times.
    // 0.04667 swerve rotation / motor rotations or 21.4286 motor rotations / swerve rotations

    // mk4i l2 swerve modules
    public static final double kDriveMotorGearRatio = 1 / 6.75;  // l2 6.75:1 // overall gear ratio
        public static final double kTurningMotorGearRatio = 1 / 21.4286; // = 0.04667

        // Wheel circumference * overall gear ratio
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.05;
  }
    public static final class DriveConstants{ 
       
      //public static final double kTrackWidth = 0.3683;
        // Distance between right and left wheels
        //public static final double kWheelBase = 0.5461;
      //fix all port numbers and max speeds

      /**
       * The left-to-right distance between the drivetrain wheels
       * Should be measured from center to center.
       */
      // Distance between right and left wheels
      //public static final double kTrackWidth = 0.3683; // wasn't correct
      public static final double kTrackWidth = 0.40005; // 15.75 converted

      /**
       * The front-to-back distance between the drivetrain wheels.
       * Should be measured from center to center.
       */
        // Distance between front and back wheels
        //public static final double kWheelBase = 0.5461; // wasn't correct was inside wheel to wheel not center to center
        public static final double kWheelBase = 0.65405; // 25.75 inches converted in meters

        /*public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));*/

      // x,y can coders are from the center of the robot
      // These values are off a bit but D said should be ok
      // the can coders are exactly 8 inches from the center
      public static final Translation2d kFrontLeftLocation = new Translation2d(0.3302, 0.2032);
      //public static final Translation2d kFrontLeftLocation = new Translation2d(0.2032, 0.3302);
      public static final Translation2d kFrontRightLocation = new Translation2d(0.3302,-0.2032);
      //public static final Translation2d kFrontRightLocation = new Translation2d(0.2032, -0.3302);
      public static final Translation2d kBackLeftLocation = new Translation2d(-0.3302, 0.2032);
      //public static final Translation2d kBackLeftLocation = new Translation2d(-0.2032, 0.3302);
      public static final Translation2d kBackRightLocation = new Translation2d(-0.3302, -0.2032);
      //public static final Translation2d kBackRightLocation = new Translation2d(-0.2032, -0.3302);
      
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation
      );
      
      
      public static final int kFrontLeftDriveMotorPort = 8;
      public static final int kBackLeftDriveMotorPort = 6;
      public static final int kFrontRightDriveMotorPort = 3;
      public static final int kBackRightDriveMotorPort = 5;

      public static final int kFrontLeftTurningMotorPort = 10;
      public static final int kBackLeftTurningMotorPort = 7;
      public static final int kFrontRightTurningMotorPort = 2;
      public static final int kBackRightTurningMotorPort = 4;

      public static final boolean kFrontLeftTurningEncoderReversed = true;
      public static final boolean kBackLeftTurningEncoderReversed = true;
      public static final boolean kFrontRightTurningEncoderReversed = true;
      public static final boolean kBackRightTurningEncoderReversed = true;

      public static final boolean kFrontLeftDriveEncoderReversed = false;//was true
      public static final boolean kBackLeftDriveEncoderReversed = false;
      public static final boolean kFrontRightDriveEncoderReversed = true;
      public static final boolean kBackRightDriveEncoderReversed = true;

      // pretend all absolute encoder vals are can coder
      // these sit on top of the swerve drives there are only 4
      public static final int kFrontLeftDriveAbsoluteEncoderID = 3;
      public static final int kBackLeftDriveAbsoluteEncoderID = 2;
      public static final int kFrontRightDriveAbsoluteEncoderID = 4;
      public static final int kBackRightDriveAbsoluteEncoderID = 1;

      public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
      public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
      public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

      // verify these values
      //startup vals for encoders
      public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.1282;
      //public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(35.2);
      public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.2773;
      public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.1354;
      public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.1098;

      public static final double kFLTurningOffset = 0;

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
    public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kRightID = 5;
    public static final int kLeftID = 6;

    // Current limit for launcher and feed wheels
    public static final int kLeftCurrentLimit = 80;
    public static final int kRightCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kLauncherDelay = 1;
  }
}
