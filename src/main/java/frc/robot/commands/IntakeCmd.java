// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;


public class IntakeCmd extends Command {
  /** Creates a new IntakeCmd. */
  private final Intake intake;
  private final Supplier<Boolean> rollerIn, rollerOut, liftUp, liftDown;
  public IntakeCmd(Intake in, Supplier<Boolean> rollerIn, Supplier<Boolean> rollerOut, Supplier<Boolean> liftUp, Supplier<Boolean> liftDown) {
    intake = in;
    this.rollerIn = rollerIn;
    this.rollerOut = rollerOut;
    this.liftUp = liftUp;
    this.liftDown = liftDown;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(liftUp.get() == true)
      intake.setLiftSpd(.4);
    if(liftDown.get() == true)
      intake.setLiftSpd(-.4);
    if(rollerIn.get() == true)
      intake.setRollerSpd(IntakeConstants.rollerInSpd);
    if(rollerOut.get() == true)
      intake.setRollerSpd(IntakeConstants.rollerOutSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set encoder limits here
    if(intake.getEncVal() < IntakeConstants.encoderMin || intake.getEncVal() > IntakeConstants.encoderMax){
      intake.stopLift();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
