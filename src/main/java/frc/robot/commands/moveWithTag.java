package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimeLight;




public class moveWithTag extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private LimeLight m_LimeLight;
    private SwerveSubsystem m_Drivetrain;
 
    /**
     * Creates a new AutoDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public void MoveWithTag(LimeLight subsystem1, SwerveSubsystem subsystem2) {
        m_LimeLight = subsystem1;
      // Use addRequirements() here to declare subsystem dependencies.

      m_Drivetrain = subsystem2;


      addRequirements(subsystem1, subsystem2);
    }
 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
 
    // Called every time the scheduler runs while the command is scheduled.
    @Override
public void execute() {
    // Check if there's a valid target
    if (!m_LimeLight.m_ValidTarget) {
        return; // No valid target, do nothing
    }


    // Calculate the necessary adjustments for alignment
    double turnAdjust = 0.0;
    double moveAdjust = 0.0;


    // Determine turn direction based on horizontal offset (tx)
    if (Math.abs(m_LimeLight.tx) > DrivetrainConstants.k_AutoCorrectTurn) {
        turnAdjust = m_LimeLight.tx > 0 ? -DrivetrainConstants.k_AutoCorrectSpeed : DrivetrainConstants.k_AutoCorrectSpeed;
    }


    // Determine move direction based on target area (ta)
    if (Math.abs(m_LimeLight.ta - DrivetrainConstants.k_AutoCorrectDist) > 0.1) {
        moveAdjust = m_LimeLight.ta > DrivetrainConstants.k_AutoCorrectDist ? -DrivetrainConstants.k_AutoCorrectSpeed : DrivetrainConstants.k_AutoCorrectSpeed;
    }


    // Command the drivetrain to adjust position and orientation
    m_Drivetrain.arcadeDrive(moveAdjust, turnAdjust);
}


 
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
 
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}




