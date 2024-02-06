// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */

  private CANSparkMax liftMotor;
  private RelativeEncoder liftEncoder;
  
  private CANSparkMax rollerMotor;
  private RelativeEncoder rollerEncoder;
  public GroundIntake(int liftMotorID, int rollerMotorID) {
    liftMotor = new CANSparkMax(liftMotorID, MotorType.fromId(liftMotorID));
    liftEncoder = liftMotor.getEncoder();

    rollerMotor = new CANSparkMax(rollerMotorID, MotorType.fromId(rollerMotorID));
    rollerEncoder = rollerMotor.getEncoder();
  }

  public Command getLiftCommand(){
    return this.startEnd(
      () -> {
        setLift(.4);
      }, 
      () -> {
        stopLift();
      });
  }
  public Command getRollerCommand(){
    return this.startEnd(
      () -> {
        setRoller(.8);
      }, 
      () -> {
        stopRoller();
      });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLift(double spd){
    liftMotor.set(spd);
  }

  public void stopLift(){
    liftMotor.set(0);
  }

  public void setRoller(double spd){
    rollerMotor.set(spd);
  }

  public void stopRoller(){
    rollerMotor.set(0);
  }
}
