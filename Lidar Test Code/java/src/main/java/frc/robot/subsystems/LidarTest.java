// package frc.robot.subsystems;

// import com.studica.frc.Lidar;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class LidarTest extends SubsystemBase
// {

//     // Lidar Library
//     private Lidar lidar;
//     // Lidar Scan Data Storage Class
//     private Lidar.ScanData scanData;
//     // Dashboard flag to prevent updating when not scanning
//     public boolean scanning = true;

//     public LidarTest ()
//     {
//         /**
//          * Top USB 2.0 port of VMX = kUSB1
//          * Bottom USB 2.0 port of VMX = kUSB2
//          */
//         lidar = new Lidar(Lidar.Port.kUSB2); //Lidar will start spinning the moment this is called

//         // Configure filters
//         lidar.clusterConfig(50.0f, 5);
//         // lidar.kalmanConfig(1e-5f, 1e-1f, 1.0f);
//         // lidar.movingAverageConfig(5);
//         // lidar.medianConfig(5);
//         // lidar.jitterConfig(50.0f);

//         // Enable Filter
//         lidar.enableFilter(Lidar.Filter.kCLUSTER, false);
//     }

//     /**
//      * Starts the lidar if it was stopped
//      */
//     public void startScan()
//     {
//         lidar.start();
//         scanning = true;
//     }

//     /**
//      * Stops the lidar if needed. This will reduce the overhead of CPU and RAM by very little. 
//      */
//     public void stopScan()
//     {
//         lidar.stop();
//         scanning = false;
//     }

//     @Override
//     public void periodic() {
//         if (!scanning) {
//             return;
//         }
    
//         // 1. Update scanData FIRST
//         scanData = lidar.getData();
    
//         // 2. Validate scanData and both array payloads before reading
//         if (scanData != null && scanData.distance != null && scanData.angle != null) {
//             int length = Math.min(scanData.distance.length, scanData.angle.length);
    
//             if (length > 0) {
//                 // Interleave angles and distances into a single payload
//                 double[] payload = new double[length * 2];
//                 for (int i = 0; i < length; i++) {
//                     payload[i * 2] = scanData.angle[i];
//                     payload[i * 2 + 1] = scanData.distance[i];
//                 }
    
//                 // Publish payload to NetworkTables
//                 NetworkTableInstance.getDefault()
//                     .getTable("Lidar")
//                     .getEntry("ScanData")
//                     .setDoubleArray(payload);
    
//                 // Safely print single reading to SmartDashboard
//                 // if (length > 60) {
//                 //     SmartDashboard.putNumber("Angle", scanData.angle[60]);
//                 //     SmartDashboard.putNumber("Distance", scanData.distance[60]);
//                 // }
//             }
//         }
//     }
// }


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
                double[] angles = new double[length];
                double[] distances = new double[length];

                for (int i = 0; i < length; i++) {
                    angles[i] = scanData.angle[i];
                    distances[i] = scanData.distance[i];
                }

                // Publish two separate entries to avoid array truncation
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