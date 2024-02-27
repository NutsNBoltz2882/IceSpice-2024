package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// arm subsys
public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    // Define constants for the subsystem
    public static final int INTAKE_MOTOR_PORT = 2;
    public static final int ENCODER_CHANNEL_A = 0;
    public static final int ENCODER_CHANNEL_B = 1;
    public static final boolean REVERSE_ENCODER_DIRECTION = false;
    private static final double kP = 0.1; // Proportional gain
    private static final double kI = 0.005; // Integral gain
    private static final double kD = 0.01; // Derivative gain
    private static final double kToleranceCounts = 10.0; // Tolerance for reaching target position in counts
    private double originalPosition = 0.0;
    private  SparkAbsoluteEncoder encoder;
    private DoubleSupplier intkpos = () ->  encoder.getPosition()*(360/42);
    // Declare the motor controller and encoder for the intake
    
    private Encoder intakeEncoder;

    /**
     * Constructor for the Intake subsystem.
     */
    public Intake() {
        // Initialize the intake motor controller
        intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
        encoder = intakeMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.fromId(INTAKE_MOTOR_PORT));
        

        // Initialize the encoder
       // intakeEncoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B, REVERSE_ENCODER_DIRECTION, EncodingType.k4X);
         // Adjust based on your encoder's specifications
         encoder.setPositionConversionFactor(360/42);
        //calculated degrees for what i need it to do 
        // Add the intake motor controller and encoder to the subsystem
       // addChild("Intake Motor", intakeMotor);
        addChild("Intake Encoder", intakeEncoder);

        // Set the motor inversion if needed
        intakeMotor.setInverted(true);

        // Add the intake encoder to Shuffleboard with automatic updates
        
           // double degrees = intakeEncoder.get();
        
    }

    /**
     * Periodic method called once per scheduler run.
     */
    @Override
    public void periodic() {
        // Add any periodic tasks or updates here
        
        // Get the current position in degrees
        
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
        double currentPos = encoder.getPosition();
    
        // Print or use the current position as needed
       // System.out.println("Intake Current Position (Degrees): " + currentPos);
    
        // Example: Adjust motor speed based on the error between current position and target position
        double ppos = targetPosition - currentPos;
        double kP = 0.01; // Adjust this proportional constant as needed
    
        // Example: If the intake is below the target position, move the arm forward
        if (Math.abs(ppos) > 1.0) { // Check if the error is greater than 1 degree to avoid constant movement
            // Calculate motor speed based on the error
            double motorSpeed = kP * ppos;
    
            // Set the motor speed to move the arm
            intakeMotor.set(motorSpeed);
        } else {
            // Stop the motor if the intake is at or above the target position
            intakeMotor.stopMotor();
        }
    }
    public double getIntakeEncoderPosition() {
        return encoder.getPosition();
        //gets the intake position
    }
    public void moveArm(double speed) {
        intakeMotor.set(speed);
        //moves the arm
    }
    public void stopArm() {
        intakeMotor.stopMotor();
        //stops the arm
    }
    public DoubleSupplier getpos(){
        if(encoder != null){
             return intkpos;
        }
        else{return () -> 360;}
       
    }

    // Add additional methods for controlling the subsystem as needed
}
