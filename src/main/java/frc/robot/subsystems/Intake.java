package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The Intake subsystem for handling intake functionality with an integrated encoder.
 */
public class Intake extends SubsystemBase {

    // Define constants for the subsystem
    public static final int INTAKE_MOTOR_PORT = 2;
    public static final int ENCODER_CHANNEL_A = 0;
    public static final int ENCODER_CHANNEL_B = 1;
    public static final boolean REVERSE_ENCODER_DIRECTION = false;

    // Declare the motor controller and encoder for the intake
    private PWMSparkMax intakeMotor;
    private Encoder intakeEncoder;

    /**
     * Constructor for the Intake subsystem.
     */
    public Intake() {
        // Initialize the intake motor controller
        intakeMotor = new PWMSparkMax(INTAKE_MOTOR_PORT);

        // Initialize the encoder
        intakeEncoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B, REVERSE_ENCODER_DIRECTION, EncodingType.k4X);
        intakeEncoder.setDistancePerPulse(360.0 / 42.0); // Adjust based on your encoder's specifications

        // Add the intake motor controller and encoder to the subsystem
        addChild("Intake Motor", intakeMotor);
        addChild("Intake Encoder", intakeEncoder);

        // Set the motor inversion if needed
        intakeMotor.setInverted(false);

        // Add the intake encoder to Shuffleboard with automatic updates
        Shuffleboard.getTab("Intake")
            .add("Intake Position", intakeEncoder)
            .withWidget(BuiltInWidgets.kEncoder)
            .withPosition(0, 0)
            .withSize(2, 1);
    }

    /**
     * Periodic method called once per scheduler run.
     */
    @Override
    public void periodic() {
        // Add any periodic tasks or updates here

        // Get the current position in degrees
        double degrees = intakeEncoder.get();

        // Print or use the degrees as needed
        System.out.println("Intake Position (Degrees): " + degrees);
    }

    /**
     * Simulation-specific periodic method called once per scheduler run when in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Add simulation-specific tasks or updates here
    }

    /**
     * Method for moving the intake arm. Implement the logic as needed.
     */
    // ...

    /**
     * Method for moving the intake arm based on a target position.
     * @param targetPosition The target position in degrees.
     */
    public void moveArmToPosition(double targetPosition) {
        // Get the current position in degrees
        double currentPos = intakeEncoder.get();

        // Print or use the current position as needed
        System.out.println("Intake Current Position (Degrees): " + currentPos);

        // Example: Adjust motor speed based on the error between current position and target position
        double error = targetPosition - currentPos;
        double kP = 0.01; // Adjust this proportional constant as needed

        // Example: If the intake is below the target position, move the arm forward
        if (Math.abs(error) > 1.0) { // Check if the error is greater than 1 degree to avoid constant movement
            // Calculate motor speed based on the error
            double motorSpeed = kP * error;

            // Set the motor speed to move the arm
            intakeMotor.set(motorSpeed);
        } else {
            // Stop the motor if the intake is at or above the target position
            intakeMotor.stopMotor();
        }
    }
    public double getIntakeEncoderPosition() {
        return intakeEncoder.get();
    }
    public void moveArm(double speed) {
        intakeMotor.set(speed);
    }
    public void stopArm() {
        intakeMotor.stopMotor();
    }

    // Add additional methods for controlling the subsystem as needed
}
