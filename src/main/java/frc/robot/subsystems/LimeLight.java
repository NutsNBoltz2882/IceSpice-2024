package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    public boolean m_ValidTarget;

    public LimeLight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");

        // Other initialization if needed
    }

    @Override
    public void periodic() {
        updateLimeLightTracking();
        // Post values to SmartDashboard periodically
        SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
        SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
        SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
    }

    private void updateLimeLightTracking() {
        double targetVisible = tv.getDouble(0.0);
        double targetX = tx.getDouble(0.0);
        double targetY = ty.getDouble(0.0);
        double targetArea = ta.getDouble(0.0);

        m_ValidTarget = targetVisible == 1.0;

        SmartDashboard.putBoolean("Target Aquired", m_ValidTarget);
    }

    public boolean isTargetValid() {
        return m_ValidTarget;
    }

    public double getTargetX() {
        return tx.getDouble(0.0);
    }

    public double getTargetY() {
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }
}
