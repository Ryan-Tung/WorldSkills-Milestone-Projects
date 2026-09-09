package frc.robot.subsystems;

import com.studica.frc.Lidar;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LidarTest extends SubsystemBase {

    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;

    public LidarTest() {
        lidar = new Lidar(Lidar.Port.kUSB2);

        lidar.clusterConfig(50.0f, 5);
        lidar.enableFilter(Lidar.Filter.kCLUSTER, false);
    }

    public void startScan() {
        lidar.start();
        scanning = true;
    }

    public void stopScan() {
        lidar.stop();
        scanning = false;
    }

    @Override
    public void periodic() {
        if (!scanning) {
            return;
        }

        scanData = lidar.getData();

        if (scanData != null && scanData.distance != null && scanData.angle != null) {
            int length = Math.min(scanData.distance.length, scanData.angle.length);

            if (length > 0) {
                // Calculate step size to cap array at max 250 points across full 360°
                int step = (length > 250) ? (int) Math.ceil((double) length / 250.0) : 1;
                int outputLength = (length + step - 1) / step;

                double[] angles = new double[outputLength];
                double[] distances = new double[outputLength];

                int idx = 0;
                for (int i = 0; i < length && idx < outputLength; i += step) {
                    angles[idx] = scanData.angle[i];
                    distances[idx] = scanData.distance[i];
                    idx++;
                }

                // Publish downsampled arrays spanning 0 to 360 degrees
                NetworkTableInstance.getDefault()
                    .getTable("Lidar")
                    .getEntry("ScanAngles")
                    .setDoubleArray(angles);

                NetworkTableInstance.getDefault()
                    .getTable("Lidar")
                    .getEntry("ScanDistances")
                    .setDoubleArray(distances);

                if (length > 60) {
                    SmartDashboard.putNumber("Angle", scanData.angle[60]);
                    SmartDashboard.putNumber("Distance", scanData.distance[60]);
                }
            }
        }
    }
}