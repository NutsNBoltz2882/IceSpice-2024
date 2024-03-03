package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// arm subsys
public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    // Define constants for the subsystem
    public static final int INTAKE_MOTOR_PORT = 2;
    public static final int ENCODER_CHANNEL_A = 0;
    public static final int ENCODER_CHANNEL_B = 1;
    public static final boolean REVERSE_ENCODER_DIRECTION = false;
    private static double kP = 0.1; // Proportional gain
    private static final double kI = 0.005; // Integral gain
    private static final double kD = 0.01; // Derivative gain
    private static final double kToleranceCounts = 10.0; // Tolerance for reaching target position in counts
    private double originalPosition = 0.0;
    
    private SparkPIDController intakePID;
    public  boolean armStatus = false;
    
    // Declare the motor controller and intakeEncoder for the intake
    
    public RelativeEncoder intakeEncoder;
    private DoubleSupplier intkpos = () ->  intakeEncoder.getPosition();
    /**
     * Constructor for the Intake subsystem.
     */
    public Intake() {
        // Initialize the intake motor controller
        intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.fromId(13), 8192);
        intakePID = intakeMotor.getPIDController();
        

        // Initialize the encoder
       // intakeEncoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B, REVERSE_ENCODER_DIRECTION, EncodingType.k4X);
         // Adjust based on your encoder's specifications
         intakeEncoder.setPositionConversionFactor(360/8192);
        //calculated degrees for what i need it to do 
        // Add the intake motor controller and encoder to the subsystem
       // addChild("Intake Motor", intakeMotor);
       // addChild("Intake Encoder", intakeEncoder);

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
        intakePID.setP(kP);
        intakePID.setI(kI);
        intakePID.setD(kD);
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
        double currentPos = intakeEncoder.getPosition();
    
        // Print or use the current position as needed
       // System.out.println("Intake Current Position (Degrees): " + currentPos);
    
        // Example: Adjust motor speed based on the error between current position and target position
        double remaningDistance = targetPosition - currentPos;
        kP = 0.01; // Adjust this proportional constant as needed
    
        // Example: If the intake is below the target position, move the arm forward
        if (Math.abs(remaningDistance) > 1.0) {
             // Check if the remaning distance is greater than 1 degree to avoid constant movement
            // Calculate motor speed based on the error
            double motorSpeed = kP * remaningDistance;
    
            // Set the motor speed to move the arm
            intakeMotor.set(motorSpeed);
            
        } else {
            // Stop the motor if the intake is at or above the target position
            //tells the shuffle/Smart Dashboard if the arm is down 
           
            intakeMotor.stopMotor();
        }
    }
    public double getIntakeEncoderPosition() {
        return intakeEncoder.getPosition();
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
        if(intakeEncoder != null){
             return intkpos;
        }
        else{return () -> 0;}
       
    }
    public Boolean armStatus(boolean status){
        if(status){
            armStatus = ! status;
        }if(armStatus){
            status = ! armStatus;
        }
        return status;
    // Add additional methods for controlling the subsystem as needed
}
}
