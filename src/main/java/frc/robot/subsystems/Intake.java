// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax jointMotor;
  private final RelativeEncoder jointEncoder;

  private final CANSparkMax rollerMotor;
  private final RelativeEncoder rollerEncoder;

  /** Creates a new Intake. */
  public Intake(int jointMotorPort, int rollerMotorPort) {
    jointMotor = new CANSparkMax(jointMotorPort, MotorType.fromId(jointMotorPort));
    jointEncoder = jointMotor.getEncoder();

    rollerMotor = new CANSparkMax(rollerMotorPort, MotorType.fromId(rollerMotorPort));
    rollerEncoder = rollerMotor.getEncoder();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendIntake(double spd){
    if((jointEncoder.getPosition()>IntakeConstants.intakeMin)&&(jointEncoder.getPosition()<IntakeConstants.intakeMax))
    jointMotor.set(spd);
    else {jointMotor.set(0);}
  }
}
