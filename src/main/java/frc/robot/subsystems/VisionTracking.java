package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import java.util.ArrayList;

public class VisionTracking extends Command {

    NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = get("tv");

    public double get(String var) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0.0);
    }

    public VisionTracking() {
    }

    public void setTracking(boolean tracking) {
        m_limelightTable.getEntry("camMode").setNumber(tracking ? 0 : 1);
        m_limelightTable.getEntry("ledMode").setNumber(tracking ? 0 : 1);
       
      }

    protected void initialize() {
    }

    protected void execute() {
    }

    public double averageValues(String targetName) {
        double target = get("targetName");
        ArrayList<Double> arr = new ArrayList<Double>();
        if (arr.size() >= 5) {
            arr.remove(0);
        }
        arr.add(target);
        double n = 0;
        double sum = 0;
        for (double v : arr) {
            sum += v;
            if (v != 0) {
                n++;
            }
        }
        return sum / n;
    }

    public double pidX() {
        double kP = 1;
        double error = get("tx");
        return kP * error;
    }

    public double pidY() {
        double kP = 0.1;
        double error = -averageValues("ty");
        return kP * error;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    protected void interrupted() {
        end();
    }
}
