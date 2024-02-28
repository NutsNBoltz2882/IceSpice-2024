// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Intake intake = new Intake(IntakeConstants.rollerMotorID, IntakeConstants.liftMotorID);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_intakeController = 
      new CommandXboxController((OperatorConstants.kIntakeControllerPort));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveDrive(
      swerveSubsystem,
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> m_driverController.getRightX(),
      () -> !m_driverController.start().getAsBoolean()));
    
    intake.setDefaultCommand(new IntakeCmd(
      intake, 
      () -> m_intakeController.leftBumper().getAsBoolean(),
      () -> m_intakeController.rightBumper().getAsBoolean(),
      () -> m_intakeController.getLeftY()));
    
      /*swerveSubsystem.setDefaultCommand(new RunCommand(
        () ->  
        new SwerveDrive(
           -m_driverController.getRawAxis(OperatorConstants.kDriverYAxis),
            m_driverController.getRawAxis(OperatorConstants.kDriverXAxis),
            m_driverController.getRawAxis(OperatorConstants.kDriverRotAxis),
            m_driverController.a().getAsBoolean()), 
            swerveSubsystem));*/
        
      // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
     if(m_driverController.a().getAsBoolean())
       swerveSubsystem.zeroHeading();

    if(m_intakeController.a().getAsBoolean());
      new IntakeDown();

    /*if(m_intakeController.leftBumper().getAsBoolean())
      intake.setRollerSpd(.8);

    if(m_intakeController.rightBumper().getAsBoolean())
      intake.setRollerSpd(-.8);*/
    
    

     // chat what the flip m_intakeController.leftBumper(() -> intake.setRollerSpd(.8));
   
     
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
