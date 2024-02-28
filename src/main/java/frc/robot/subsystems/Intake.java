// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax rollerMotor;
  private final CANSparkMax liftMotor;

  private final RelativeEncoder liftEncoder;
  private final SparkMaxAlternateEncoder.Type encType = SparkMaxAlternateEncoder.Type.kQuadrature;

    public Intake(CANSparkMax rm, CANSparkMax lm) {
    rollerMotor = rm;
    liftMotor = lm;
    liftEncoder = liftMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setRollerSpd(double spd){
    rollerMotor.set(spd);
  }
  public void setLiftSpd(double spd){
    //use if statement to set limit
    liftMotor.set(spd);
  }
}
