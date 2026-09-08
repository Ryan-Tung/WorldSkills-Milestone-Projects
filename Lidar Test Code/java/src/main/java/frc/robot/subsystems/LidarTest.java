package frc.robot.subsystems;

import com.studica.frc.Lidar;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LidarTest extends SubsystemBase
{

    // Lidar Library
    private Lidar lidar;
    // Lidar Scan Data Storage Class
    private Lidar.ScanData scanData;
    // Dashboard flag to prevent updating when not scanning
    public boolean scanning = true;

    public LidarTest ()
    {
        /**
         * Top USB 2.0 port of VMX = kUSB1
         * Bottom USB 2.0 port of VMX = kUSB2
         */
        lidar = new Lidar(Lidar.Port.kUSB2); //Lidar will start spinning the moment this is called

        // Configure filters
        lidar.clusterConfig(50.0f, 5);
        // lidar.kalmanConfig(1e-5f, 1e-1f, 1.0f);
        // lidar.movingAverageConfig(5);
        // lidar.medianConfig(5);
        // lidar.jitterConfig(50.0f);

        // Enable Filter
        lidar.enableFilter(Lidar.Filter.kCLUSTER, false);
    }

    /**
     * Starts the lidar if it was stopped
     */
    public void startScan()
    {
        lidar.start();
        scanning = true;
    }

    /**
     * Stops the lidar if needed. This will reduce the overhead of CPU and RAM by very little. 
     */
    public void stopScan()
    {
        lidar.stop();
        scanning = false;
    }

    @Override
    public void periodic() {
        if (scanning && scanData != null && scanData.distance != null) {
            // Interleave angles and distances into a single array
            double[] payload = new double[scanData.distance.length * 2];
            for (int i = 0; i < scanData.distance.length; i++) {
                payload[i * 2] = scanData.angle[i];
                payload[i * 2 + 1] = scanData.distance[i];
            }
            
            // Publish to /Lidar/ScanData entry
            NetworkTableInstance.getDefault()
                .getTable("Lidar")
                .getEntry("ScanData")
                .setDoubleArray(payload);
        }
    }
}