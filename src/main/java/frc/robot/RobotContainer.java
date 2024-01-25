// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Intake intake = new Intake(IntakeConstants.kjointMotorPort, IntakeConstants.kRollerMotorPort);

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_intakeController = 
      new XboxController(OperatorConstants.kIntakeControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveDrive(
      swerveSubsystem,
      () -> -m_driverController.getRawAxis(OperatorConstants.kDriverYAxis),
      () -> m_driverController.getRawAxis(OperatorConstants.kDriverXAxis),
      () -> m_driverController.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !m_driverController.getRawButton(OperatorConstants.kDriverFieldOrientedButtonIdx)));

      //intake.setDefaultCommand(() -> intake.extendIntake(m_intakeController.getLeftY()));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    if(m_driverController.getStartButton())
      swerveSubsystem.zeroHeading();
    
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
