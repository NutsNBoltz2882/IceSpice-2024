// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final CANcoder turningEncoder;

  private final PIDController turningPidController;

  //private final CANcoder CANcoder;

  private final boolean CANCoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
  int CANCoderID, double CANCoderOffset, boolean CANCoderReversed) {
    this.absoluteEncoderOffsetRad = CANCoderOffset;
    this.CANCoderReversed = CANCoderReversed;
    //CANcoder = new CANcoder(CANCoderID);
    
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless); 
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = new CANcoder(CANCoderID);
   
     driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);


    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = turningEncoder.getSupplyVoltage().getValueAsDouble();
    angle *= 2.0 * Math.PI;
    return angle * (CANCoderReversed ? -1.0 : 1.0);
  }
  public void zeroModule(double pos, double offset){
     if(pos != offset){
      turningMotor.set(.5);

        if(pos == offset){
          stop();
        }
     }
    }


  public void resetEncoders(){
    driveEncoder.setPosition(0);
    //hoping and praying wheels are correctly aligned each time...
    turningEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhyscialMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + turningEncoder.getDeviceID() + "] state", state.toString());
  }
  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
