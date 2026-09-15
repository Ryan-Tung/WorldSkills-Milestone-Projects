// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import com.studica.frc.TitanQuad;
// import com.studica.frc.TitanQuadEncoder;
// import com.studica.frc.Lidar;
// import com.studica.frc.Cobra;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;
// import java.util.ArrayList;
// import java.util.List;

// public class DriveTrain extends SubsystemBase
// {
//     // ============================================================
//     // HARDWARE & SENSORS
//     // ============================================================
//     private Cobra cobra;
//     private final double[] whiteBaseline = new double[4];
//     private static final double TAPE_DELTA_THRESHOLD_VOLTS = 0.5;

//     private Lidar lidar;
//     private Lidar.ScanData scanData;
//     public boolean scanning = true;

//     private TitanQuad leftMotor;
//     private TitanQuad rightMotor;
//     private TitanQuad backMotor;

//     private TitanQuadEncoder leftEncoder;
//     private TitanQuadEncoder rightEncoder;
//     private TitanQuadEncoder backEncoder;

//     private AHRS navx;

//     private boolean navxConnected = false;
//     private boolean lidarConnected = false;

//     // ============================================================
//     // INDEPENDENT LOCALIZATION & DEBUG POSES (X, Y in meters, Heading in deg)
//     // ============================================================
//     // 1. Pure Odometry Pose
//     private double odoX = 0.0, odoY = 0.0, odoHeading = 0.0;
    
//     // 2. Pure LiDAR Pose (Updated via Scan Matching)
//     private double lidarX = 0.0, lidarY = 0.0, lidarHeading = 0.0;

//     // 3. Fused Pose (Odometry + LiDAR filter)
//     private double fusedX = 0.0, fusedY = 0.0, fusedHeading = 0.0;

//     // Fusion weighting factor (0.0 = pure odometry, 1.0 = pure LiDAR correction)
//     private static final double FUSION_ALPHA_POS = 0.35; 
//     private static final double FUSION_ALPHA_HEADING = 0.20; 

//     // Odometry tracking deltas
//     private double previousLeftDistance = 0.0;
//     private double previousRightDistance = 0.0;
//     private double previousBackDistance = 0.0;
//     private double previousHeading = 0.0;
    

//     // Map reference points for LiDAR ICP (wall/field obstacle boundaries in meters)
//     private final List<double[]> mapReferencePoints = new ArrayList<>();

//     // ============================================================
//     // NETWORKTABLES & SHUFFLEBOARD TELEMETRY
//     // ============================================================
//     private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drive");
//     private final NetworkTable lidarTable = NetworkTableInstance.getDefault().getTable("Lidar");
//     private final NetworkTable controlTable = NetworkTableInstance.getDefault().getTable("DriveControls");


//     private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");

//     private NetworkTableEntry leftEncoderValue = tab.add("Left Encoder", 0).getEntry();
//     private NetworkTableEntry rightEncoderValue = tab.add("Right Encoder", 0).getEntry();
//     private NetworkTableEntry backEncoderValue = tab.add("Back Encoder", 0).getEntry();
//     private NetworkTableEntry gyroValue = tab.add("NavX Heading", 0).getEntry();

//     public DriveTrain()
//     {
//         // --------------------------------------------------------
//         // HARDWARE INIT WITH CRASH PROTECTION
//         // --------------------------------------------------------
//         cobra = new Cobra();

//         try {
//             lidar = new Lidar(Lidar.Port.kUSB2);
//             lidar.clusterConfig(50.0f, 5);
//             lidar.enableFilter(Lidar.Filter.kCLUSTER, false);
//             lidarConnected = true;
//         } catch (Throwable e) {
//             System.err.println("CRITICAL WARNING: Lidar failed to initialize! " + e.getMessage());
//             lidarConnected = false;
//         }

//         try {
//             navx = new AHRS(SPI.Port.kMXP);
//             navxConnected = navx.isConnected();
//         } catch (Throwable e) {
//             System.err.println("CRITICAL WARNING: NavX failed to initialize! " + e.getMessage());
//             navxConnected = false;
//         }

//         leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.M3);
//         rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
//         backMotor = new TitanQuad(Constants.TITAN_ID, Constants.M1);

//         leftEncoder = new TitanQuadEncoder(leftMotor, Constants.M3, Constants.WHEEL_DIST_PER_TICK);
//         rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
//         backEncoder = new TitanQuadEncoder(backMotor, Constants.M1, Constants.WHEEL_DIST_PER_TICK);

//         initMapReference();
//         resetOdometry();
//     }

//     /**
//      * Initializes reference map boundary points (Walls/Perimeter) for LiDAR ICP matching.
//      */
//     private void initMapReference() {
//         double mapSize = 10.0; // 10x10m square arena around center (0,0)
//         double step = 0.25;
//         for (double p = -mapSize / 2; p <= mapSize / 2; p += step) {
//             mapReferencePoints.add(new double[]{p, mapSize / 2});   // Top wall
//             mapReferencePoints.add(new double[]{p, -mapSize / 2});  // Bottom wall
//             mapReferencePoints.add(new double[]{mapSize / 2, p});   // Right wall
//             mapReferencePoints.add(new double[]{-mapSize / 2, p});  // Left wall
//         }
//     }

//     // ============================================================
//     // SENSOR & MOTOR METHODS
//     // ============================================================

//     public float getCobraVoltage(int channel) {
//         if (channel < 0 || channel >= 4) return 0.0f;
//         return cobra.getVoltage(channel);
//     }

//     public void calibrateCobraWhite() {
//         double[] sums = new double[4];
//         int samples = 10;
//         for (int s = 0; s < samples; s++) {
//             for (int ch = 0; ch < 4; ch++) {
//                 sums[ch] += cobra.getVoltage(ch);
//             }
//             try { Thread.sleep(10); } catch (InterruptedException ignored) {}
//         }

//         StringBuilder log = new StringBuilder("CALIBRATED WHITE BASELINE -> ");
//         for (int ch = 0; ch < 4; ch++) {
//             whiteBaseline[ch] = sums[ch] / samples;
//             log.append(String.format("Ch%d: %.2fV | ", ch, whiteBaseline[ch]));
//         }
//     }

//     public boolean isTapeDetected(int channel) {
//         if (channel < 0 || channel >= 4) return false;
//         double currentVoltage = cobra.getVoltage(channel);
//         return (whiteBaseline[channel] - currentVoltage) >= TAPE_DELTA_THRESHOLD_VOLTS;
//     }

//     public boolean isAnyTapeDetected() {
//         for (int ch = 0; ch < 4; ch++) {
//             if (isTapeDetected(ch)) return true;
//         }
//         return false;
//     }

//     public void startScan() { 
//         if (lidarConnected && lidar != null) lidar.start(); 
//         scanning = true; 
//     }

//     public void stopScan() { 
//         if (lidarConnected && lidar != null) lidar.stop(); 
//         scanning = false; 
//     }

//     public void setLeftMotorSpeed(double speed) { leftMotor.set(speed); }
//     public void setRightMotorSpeed(double speed) { rightMotor.set(speed); }
//     public void setBackMotorSpeed(double speed) { backMotor.set(speed); }

//     public void setDriveMotorSpeeds(double leftSpeed, double rightSpeed, double backSpeed) {
//         leftMotor.set(leftSpeed);
//         rightMotor.set(rightSpeed);
//         backMotor.set(backSpeed);
//     }
//     //     /**
//     //  * Reads keyboard velocity inputs sent from the Python application via NetworkTables.
//     //  */
//     public void processNetworkTableDrive() {
//         double forward = controlTable.getEntry("CmdForward").getDouble(0.0);
//         double strafe = controlTable.getEntry("CmdStrafe").getDouble(0.0);
//         double turn = controlTable.getEntry("CmdTurn").getDouble(0.0);

//         holonomicDrive(strafe, forward, turn);
//     }

//     public void holonomicDrive(double x, double y, double z) {
//         double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
//         double leftSpeed  = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
//         double backSpeed  = (-2 * x / 3) + z;

//         double max = Math.abs(rightSpeed);
//         if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
//         if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);

//         if (max > 1) {
//             rightSpeed /= max;
//             leftSpeed /= max;
//             backSpeed /= max;
//         }

//         leftMotor.set(leftSpeed);
//         rightMotor.set(rightSpeed);
//         backMotor.set(backSpeed);
//     }

//     public double getLeftEncoderDistance() { return leftEncoder.getEncoderDistance() * -1; }
//     public double getRightEncoderDistance() { return rightEncoder.getEncoderDistance() * -1; }
//     public double getBackEncoderDistance() { return backEncoder.getEncoderDistance(); }
//     public double getAverageForwardEncoderDistance() { return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0; }

//     public double getYaw() { 
//         return (navx != null && navxConnected) ? navx.getYaw() : 0.0; 
//     }

//     private double normalizeAngle(double angle) {
//         while (angle > 180.0) angle -= 360.0;
//         while (angle < -180.0) angle += 360.0;
//         return angle;
//     }

//     // ============================================================
//     // ODOMETRY & SENSOR FUSION ENGINE
//     // ============================================================

//     private void updateOdometry() {
//         double currentLeft = getLeftEncoderDistance();
//         double currentRight = getRightEncoderDistance();
//         double currentBack = getBackEncoderDistance();

//         double deltaLeft = currentLeft - previousLeftDistance;
//         double deltaRight = currentRight - previousRightDistance;
//         double deltaBack = currentBack - previousBackDistance;

//         double currentHeading = getYaw();
//         double deltaHeading = normalizeAngle(currentHeading - previousHeading);

//         // Robot relative motion (mm converted to meters by dividing by 1000)
//         double robotDeltaX = -((deltaLeft + deltaRight) / (2.0 * Math.sqrt(3.0))) - deltaBack;
//         double robotDeltaY = (deltaRight - deltaLeft) / 2.0;

//         double averageHeading = previousHeading + (deltaHeading / 2.0);
//         double headingRad = Math.toRadians(averageHeading);

//         double fieldDeltaX = (robotDeltaX * Math.cos(headingRad) + robotDeltaY * Math.sin(headingRad)) / 1000.0;
//         double fieldDeltaY = (-robotDeltaX * Math.sin(headingRad) + robotDeltaY * Math.cos(headingRad)) / 1000.0;

//         // 1. Update Pure Odometry Pose
//         odoX += fieldDeltaX;
//         odoY += fieldDeltaY;
//         odoHeading = currentHeading;

//         // 2. Propagate Fused Pose with Odometry increments
//         fusedX += fieldDeltaX;
//         fusedY += fieldDeltaY;
//         fusedHeading = normalizeAngle(fusedHeading + deltaHeading);

//         previousLeftDistance = currentLeft;
//         previousRightDistance = currentRight;
//         previousBackDistance = currentBack;
//         previousHeading = currentHeading;
//     }

//     public double getLidarAtZeroDegrees() {
//         if (scanData == null || scanData.distance == null || scanData.angle == null) return 999.0;
//         int length = Math.min(scanData.distance.length, scanData.angle.length);
//         if (length == 0) return 999.0;

//         double minDiff = Double.MAX_VALUE;
//         double distanceAt0 = 999.0;

//         for (int i = 0; i < length; i++) {
//             double angle = scanData.angle[i];
//             double diff = Math.min(Math.abs(angle - 0.0), Math.abs(angle - 360.0));
//             if (diff < minDiff) {
//                 minDiff = diff;
//                 distanceAt0 = scanData.distance[i] / 10.0;
//             }
//         }
//         return distanceAt0;
//     }

//     public double getLidarAt270Degrees() {
//         if (scanData == null || scanData.distance == null || scanData.angle == null) return 999.0;
//         int length = Math.min(scanData.distance.length, scanData.angle.length);
//         if (length == 0) return 999.0;

//         double minDiff = Double.MAX_VALUE;
//         double distanceAt270 = 999.0;

//         for (int i = 0; i < length; i++) {
//             double angle = scanData.angle[i];
//             double diff = Math.abs(angle - 270.0);
//             if (diff < minDiff) {
//                 minDiff = diff;
//                 distanceAt270 = scanData.distance[i] / 10.0;
//             }
//         }
//         return distanceAt270;
//     }

//     /**
//      * Performs lightweight ICP scan matching against map boundaries using safely snapshotted array data,
//      * then updates the Fused Pose using a complementary gain filter.
//      */
//     private void processLidarLocalizationSafe(float[] rawAngles, float[] rawDistances, int length) {
//         if (rawAngles == null || rawDistances == null || length < 10) return;

//         // Convert scan into local point cloud (meters)
//         List<double[]> localPoints = new ArrayList<>();
//         for (int i = 0; i < length; i++) {
//             double distM = rawDistances[i] / 1000.0;
//             if (distM < 0.05 || distM > 5.0) continue;
//             double rad = Math.toRadians(rawAngles[i]);
//             localPoints.add(new double[]{ distM * Math.sin(rad), distM * Math.cos(rad) });
//         }

//         if (localPoints.isEmpty()) return;

//         // Start ICP optimization with current fused estimate
//         double estX = fusedX;
//         double estY = fusedY;
//         double estHeading = fusedHeading;

//         int iterations = 5;
//         for (int iter = 0; iter < iterations; iter++) {
//             double headingRad = Math.toRadians(estHeading);
//             double cos = Math.cos(headingRad);
//             double sin = Math.sin(headingRad);

//             double errX = 0, errY = 0;
//             int count = 0;

//             for (double[] pt : localPoints) {
//                 // Transform local LiDAR point to global frame
//                 double gx = estX + pt[0] * cos + pt[1] * sin;
//                 double gy = estY - pt[0] * sin + pt[1] * cos;

//                 // Find closest map reference point
//                 double minDistSq = Double.MAX_VALUE;
//                 double[] nearest = null;
//                 for (double[] ref : mapReferencePoints) {
//                     double d2 = (ref[0] - gx) * (ref[0] - gx) + (ref[1] - gy) * (ref[1] - gy);
//                     if (d2 < minDistSq) {
//                         minDistSq = d2;
//                         nearest = ref;
//                     }
//                 }

//                 if (nearest != null && minDistSq < 0.5) { // Reject outliers > 0.7m
//                     errX += (nearest[0] - gx);
//                     errY += (nearest[1] - gy);
//                     count++;
//                 }
//             }

//             if (count > 0) {
//                 estX += (errX / count) * 0.5;
//                 estY += (errY / count) * 0.5;
//             }
//         }

//         // Store Pure LiDAR localized pose
//         lidarX = estX;
//         lidarY = estY;
//         lidarHeading = estHeading;

//         // Sensor Fusion Update: Correct Fused Pose using LiDAR update gain
//         fusedX = fusedX + FUSION_ALPHA_POS * (lidarX - fusedX);
//         fusedY = fusedY + FUSION_ALPHA_POS * (lidarY - fusedY);
//         fusedHeading = normalizeAngle(fusedHeading + FUSION_ALPHA_HEADING * normalizeAngle(lidarHeading - fusedHeading));
//     }

//     // ============================================================
//     // POSE GETTERS & RESETS
//     // ============================================================

//     public double getPoseX() { return fusedX; }
//     public double getPoseY() { return fusedY; }
//     public double getPoseHeading() { return fusedHeading; }

//     public double getOdoX() { return odoX; }
//     public double getOdoY() { return odoY; }
//     public double getOdoHeading() { return odoHeading; }

//     public double getLidarX() { return lidarX; }
//     public double getLidarY() { return lidarY; }
//     public double getLidarHeading() { return lidarHeading; }

//     public void resetEncoders() {
//         leftEncoder.reset();
//         rightEncoder.reset();
//         backEncoder.reset();
//     }

//     public void resetYaw() { 
//         if (navx != null && navxConnected) navx.zeroYaw(); 
//     }

//     public void resetOdometry() {
//         resetEncoders();
//         resetYaw();
//         odoX = 0.0; odoY = 0.0; odoHeading = 0.0;
//         lidarX = 0.0; lidarY = 0.0; lidarHeading = 0.0;
//         fusedX = 0.0; fusedY = 0.0; fusedHeading = 0.0;
//         previousLeftDistance = getLeftEncoderDistance();
//         previousRightDistance = getRightEncoderDistance();
//         previousBackDistance = getBackEncoderDistance();
//         previousHeading = getYaw();
//     }

//     // ============================================================
//     // PERIODIC LOOP & TELEMETRY
//     // ============================================================

//     @Override
//     public void periodic() {
//         processNetworkTableDrive();
//         updateOdometry();

//         if (scanning && lidarConnected && lidar != null) {
//             try {
//                 scanData = lidar.getData();
//                 if (scanData != null && scanData.distance != null && scanData.angle != null) {
//                     // Snapshot arrays as float[] to match Studica Lidar.ScanData types
//                     float[] rawAngles = scanData.angle.clone();
//                     float[] rawDistances = scanData.distance.clone();

//                     int length = Math.min(rawDistances.length, rawAngles.length);
//                     if (length > 0) {
//                         processLidarLocalizationSafe(rawAngles, rawDistances, length);

//                         int mid = length / 2;
//                         int len2 = length - mid;

//                         double[] angles1 = new double[mid], distances1 = new double[mid];
//                         for (int i = 0; i < mid; i++) {
//                             angles1[i] = rawAngles[i];
//                             distances1[i] = rawDistances[i];
//                         }

//                         double[] angles2 = new double[len2], distances2 = new double[len2];
//                         for (int i = 0; i < len2; i++) {
//                             angles2[i] = rawAngles[mid + i];
//                             distances2[i] = rawDistances[mid + i];
//                         }

//                         lidarTable.getEntry("ScanAngles_Part1").setDoubleArray(angles1);
//                         lidarTable.getEntry("ScanDistances_Part1").setDoubleArray(distances1);
//                         lidarTable.getEntry("ScanAngles_Part2").setDoubleArray(angles2);
//                         lidarTable.getEntry("ScanDistances_Part2").setDoubleArray(distances2);
//                     }
//                 }
//             } catch (Exception e) {
//                 System.err.println("LiDAR Read Error: " + e.getMessage());
//             }
//         }

//         // Telemetry & Debug Publishing
//         leftEncoderValue.setDouble(getLeftEncoderDistance());
//         rightEncoderValue.setDouble(getRightEncoderDistance());
//         backEncoderValue.setDouble(getBackEncoderDistance());
//         gyroValue.setDouble(getYaw());

//         // Fused Primary Pose (Used by default Python map)
//         driveTable.getEntry("PoseX").setDouble(fusedX);
//         driveTable.getEntry("PoseY").setDouble(fusedY);
//         driveTable.getEntry("PoseHeading").setDouble(fusedHeading);

//         // Independent Pose Streams for Debugging
//         driveTable.getEntry("OdoPoseX").setDouble(odoX);
//         driveTable.getEntry("OdoPoseY").setDouble(odoY);
//         driveTable.getEntry("OdoPoseHeading").setDouble(odoHeading);

//         driveTable.getEntry("LidarPoseX").setDouble(lidarX);
//         driveTable.getEntry("LidarPoseY").setDouble(lidarY);
//         driveTable.getEntry("LidarPoseHeading").setDouble(lidarHeading);

//         SmartDashboard.putNumber("Fused X", fusedX);
//         SmartDashboard.putNumber("Fused Y", fusedY);
//         SmartDashboard.putNumber("Odo X", odoX);
//         SmartDashboard.putNumber("Odo Y", odoY);
//         SmartDashboard.putNumber("Lidar X", lidarX);
//         SmartDashboard.putNumber("Lidar Y", lidarY);
//     }
// }