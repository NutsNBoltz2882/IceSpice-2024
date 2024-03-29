// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
//import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
     final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, 
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderID,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

     final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, 
      DriveConstants.kBackLeftDriveEncoderReversed,
      DriveConstants.kBackLeftTurningEncoderReversed, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderID,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

     final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, 
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderID,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

     final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, 
      DriveConstants.kBackRightDriveEncoderReversed,
      DriveConstants.kBackRightTurningEncoderReversed, 
      DriveConstants.kBackRightDriveAbsoluteEncoderID,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

      private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  public SwerveSubsystem() {
     new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){
      }
     }).start();

  }
  public void zeroMotors(){
    //if this works, repeat for rest of modules
    frontLeft.zeroModule(frontLeft.getTurningPosition(), DriveConstants.kFLTurningOffset);
  }

  public void zeroHeading(){
    gyro.zeroYaw();
  }
  public void resetAllEncoders(){
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2D(){
    return Rotation2d.fromDegrees(getHeading());
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putNumber("FL encoder", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("FR encoder", backLeft.getTurningPosition());
    SmartDashboard.putNumber("BL encoder", frontRight.getTurningPosition());
    SmartDashboard.putNumber("BR encoder", backRight.getTurningPosition());

  
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhyscialMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }
}
