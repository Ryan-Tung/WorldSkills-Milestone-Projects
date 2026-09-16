package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import com.studica.frc.Lidar;
import com.studica.frc.Cobra;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain extends SubsystemBase 
{
    // ============================================================
    // HELPER CLASSES & CONSTANTS
    // ============================================================
    public static class Point2D {
        public double x;
        public double y;

        public Point2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Point2D copy() {
            return new Point2D(this.x, this.y);
        }
    }

    private static class Transform2D {
        public double dThetaRad;
        public double dx;
        public double dy;

        public Transform2D(double dThetaRad, double dx, double dy) {
            this.dThetaRad = dThetaRad;
            this.dx = dx;
            this.dy = dy;
        }
    }

    private static final double MIN_DISTANCE_METERS = 0.05;
    private static final double MAX_DISTANCE_METERS = 5.0;

    // ============================================================
    // COBRA IR SENSOR
    // ============================================================
    private Cobra cobra;
    private final double[] whiteBaseline = new double[4];
    private static final double TAPE_DELTA_THRESHOLD_VOLTS = 0.5;

    // ============================================================
    // LIDAR & LOCALIZATION
    // ============================================================
    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;
    private static final double LIDAR_OFFSET_DEGREES = 13.0; // Physical offset to the right

    private List<Point2D> prevScanPoints = null;
    private double lidarPoseX = 0.0;
    private double lidarPoseY = 0.0;
    private double lidarPoseHeading = 0.0; // In degrees

    // ============================================================
    // MOTORS & NAVX
    // ============================================================
    private TitanQuad leftMotor;
    private TitanQuad rightMotor;
    private TitanQuad backMotor;

    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;

    private AHRS navx;

    // NetworkTables & Shuffleboard
    private final NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drive");
    private final NetworkTable lidarTable = NetworkTableInstance.getDefault().getTable("Lidar");
    private final NetworkTable controlTable = NetworkTableInstance.getDefault().getTable("DriveControls");

    private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private NetworkTableEntry leftEncoderValue = tab.add("Left Encoder", 0).getEntry();
    private NetworkTableEntry rightEncoderValue = tab.add("Right Encoder", 0).getEntry();
    private NetworkTableEntry backEncoderValue = tab.add("Back Encoder", 0).getEntry();
    private NetworkTableEntry gyroValue = tab.add("NavX Heading", 0).getEntry();
    private NetworkTableEntry poseXValue = tab.add("Pose X", 0).getEntry();
    private NetworkTableEntry poseYValue = tab.add("Pose Y", 0).getEntry();
    private NetworkTableEntry poseHeadingValue = tab.add("Pose Heading", 0).getEntry();

    public DriveTrain() 
    {
        cobra = new Cobra();

        lidar = new Lidar(Lidar.Port.kUSB2);
        lidar.clusterConfig(50.0f, 5);
        lidar.enableFilter(Lidar.Filter.kCLUSTER, false);

        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.M3);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
        backMotor = new TitanQuad(Constants.TITAN_ID, Constants.M1);

        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.M3, Constants.WHEEL_DIST_PER_TICK);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
        backEncoder = new TitanQuadEncoder(backMotor, Constants.M1, Constants.WHEEL_DIST_PER_TICK);

        navx = new AHRS(SPI.Port.kMXP);

        resetOdometry();
    }

    // ============================================================
    // COBRA CONTROL & CALIBRATION
    // ============================================================

    public float getCobraVoltage(int channel) {
        if (channel < 0 || channel >= 4) return 0.0f;
        return cobra.getVoltage(channel);
    }

    public void calibrateCobraWhite() {
        double[] sums = new double[4];
        int samples = 10;
        for (int s = 0; s < samples; s++) {
            for (int ch = 0; ch < 4; ch++) {
                sums[ch] += cobra.getVoltage(ch);
            }
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        for (int ch = 0; ch < 4; ch++) {
            whiteBaseline[ch] = sums[ch] / samples;
        }
    }

    public boolean isTapeDetected(int channel) {
        if (channel < 0 || channel >= 4) return false;
        double currentVoltage = cobra.getVoltage(channel);
        return (whiteBaseline[channel] - currentVoltage) >= TAPE_DELTA_THRESHOLD_VOLTS;
    }

    public boolean isAnyTapeDetected() {
        for (int ch = 0; ch < 4; ch++) {
            if (isTapeDetected(ch)) return true;
        }
        return false;
    }

    // ============================================================
    // LIDAR CONTROL
    // ============================================================

    public void startScan() {
        lidar.start();
        scanning = true;
    }

    public void stopScan() {
        lidar.stop();
        scanning = false;
    }

    // ============================================================
    // LIDAR ODOMETRY & ICP LOCALIZATION
    // ============================================================

    /**
     * Converts raw scan arrays (degrees, mm) to Cartesian points (meters).
     */
    private List<Point2D> extractLocalScanPoints(Lidar.ScanData scan) {
        List<Point2D> points = new ArrayList<>();
        if (scan == null || scan.distance == null || scan.angle == null) return points;

        int len = Math.min(scan.distance.length, scan.angle.length);
        for (int i = 0; i < len; i++) {
            double distMeters = scan.distance[i] / 1000.0;
            // Subtract offset to rotate scan points back to robot centerline
            double angleDeg = scan.angle[i] - LIDAR_OFFSET_DEGREES;
            if (Double.isFinite(distMeters) && Double.isFinite(angleDeg)
                && distMeters >= MIN_DISTANCE_METERS && distMeters <= MAX_DISTANCE_METERS) {
                
                double angleRad = Math.toRadians(angleDeg);
                double lx = distMeters * Math.sin(angleRad);
                double ly = distMeters * Math.cos(angleRad);
                points.add(new Point2D(lx, ly));
            }
        }
        return points;
    }
    public double getAverageForwardEncoderDistance() { return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0; }

    /**
     * 2D Iterative Closest Point algorithm for scan matching.
     * Computes the relative transformation (R, t) between source and destination scans.
     */
    private Transform2D icp2D(List<Point2D> src, List<Point2D> dst, int maxIterations, double tolerance) {
        if (src.size() < 10 || dst.size() < 10) {
            return new Transform2D(0.0, 0.0, 0.0);
        }

        List<Point2D> currSrc = new ArrayList<>();
        for (Point2D p : src) currSrc.add(p.copy());

        double totalDTheta = 0.0;
        double totalDx = 0.0;
        double totalDy = 0.0;

        for (int iter = 0; iter < maxIterations; iter++) {
            List<Point2D> validSrc = new ArrayList<>();
            List<Point2D> matchedDst = new ArrayList<>();

            // 1. Find nearest neighbors
            for (Point2D s : currSrc) {
                double minSqDist = Double.MAX_VALUE;
                Point2D bestMatch = null;

                for (Point2D d : dst) {
                    double sqDist = Math.pow(s.x - d.x, 2) + Math.pow(s.y - d.y, 2);
                    if (sqDist < minSqDist) {
                        minSqDist = sqDist;
                        bestMatch = d;
                    }
                }

                // Reject outlier points further than 0.5 meters
                if (bestMatch != null && Math.sqrt(minSqDist) < 0.5) {
                    validSrc.add(s);
                    matchedDst.add(bestMatch);
                }
            }

            if (validSrc.size() < 5) break;

            // 2. Compute centroids
            double csX = 0, csY = 0, cdX = 0, cdY = 0;
            int n = validSrc.size();
            for (int i = 0; i < n; i++) {
                csX += validSrc.get(i).x;
                csY += validSrc.get(i).y;
                cdX += matchedDst.get(i).x;
                cdY += matchedDst.get(i).y;
            }
            csX /= n; csY /= n;
            cdX /= n; cdY /= n;

            // 3. 2D Cross-Covariance Alignment Matrix H
            double h00 = 0, h01 = 0, h10 = 0, h11 = 0;
            for (int i = 0; i < n; i++) {
                double sx = validSrc.get(i).x - csX;
                double sy = validSrc.get(i).y - csY;
                double dx = matchedDst.get(i).x - cdX;
                double dy = matchedDst.get(i).y - cdY;

                h00 += sx * dx;
                h01 += sx * dy;
                h10 += sy * dx;
                h11 += sy * dy;
            }

            // Extract optimal 2D rotation angle
            double dTheta = Math.atan2(h01 - h10, h00 + h11);
            double cosT = Math.cos(dTheta);
            double sinT = Math.sin(dTheta);

            // Compute translation step
            double stepDx = cdX - (cosT * csX - sinT * csY);
            double stepDy = cdY - (sinT * csX + cosT * csY);

            // Update running source points
            for (Point2D p : currSrc) {
                double rx = cosT * p.x - sinT * p.y + stepDx;
                double ry = sinT * p.x + cosT * p.y + stepDy;
                p.x = rx;
                p.y = ry;
            }

            totalDTheta += dTheta;
            totalDx += stepDx;
            totalDy += stepDy;

            if (Math.hypot(stepDx, stepDy) < tolerance) break;
        }

        return new Transform2D(totalDTheta, totalDx, totalDy);
    }

    /**
     * Updates global pose estimated from successive LiDAR scan frames.
     */
    private double lastEncoderDistance = 0.0;
    private static final double ENCODER_MOTION_THRESHOLD_METERS = 0.002; // 2 mm physical movement required

    private void updateLidarOdometry() {
        if (scanData == null) return;

        List<Point2D> currentPoints = extractLocalScanPoints(scanData);
        if (currentPoints.size() < 10) return;

        if (prevScanPoints != null && prevScanPoints.size() >= 10) {
            // 1. Calculate physical wheel movement since last frame
            double currentEncoderDistance = getAverageForwardEncoderDistance();
            double encoderDelta = Math.abs(currentEncoderDistance - lastEncoderDistance);

            // 2. Only run ICP pose integration if the physical wheels actually moved
            if (encoderDelta >= ENCODER_MOTION_THRESHOLD_METERS) {
                Transform2D step = icp2D(currentPoints, prevScanPoints, 20, 1e-4);

                lidarPoseHeading = getYaw();
                double headingRad = Math.toRadians(lidarPoseHeading);

                double dxGlobal = step.dx * Math.sin(headingRad) + step.dy * Math.cos(headingRad);
                double dyGlobal = step.dx * Math.cos(headingRad) + step.dy * Math.sin(headingRad);

                lidarPoseX += dxGlobal * 2;
                lidarPoseY += dyGlobal * 2;

                lastEncoderDistance = currentEncoderDistance;
            }
        }

        prevScanPoints = currentPoints;
    }

    // ============================================================
    // MOTOR & DRIVE METHODS
    // ============================================================

    public void setLeftMotorSpeed(double speed) { leftMotor.set(speed); }
    public void setRightMotorSpeed(double speed) { rightMotor.set(speed); }
    public void setBackMotorSpeed(double speed) { backMotor.set(speed); }

    public void setDriveMotorSpeeds(double leftSpeed, double rightSpeed, double backSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }

    public void holonomicDrive(double x, double y, double z) {
        double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed  = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed  = (-2 * x / 3) + z;

        double max = Math.abs(rightSpeed);
        if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
        if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);

        if (max > 1) {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }

        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }

    public void processNetworkTableDrive() {
        double forward = controlTable.getEntry("CmdForward").getDouble(0.0);
        double strafe = controlTable.getEntry("CmdStrafe").getDouble(0.0);
        double turn = controlTable.getEntry("CmdTurn").getDouble(0.0);

        holonomicDrive(strafe, forward, turn);
    }

    public double getLeftEncoderDistance() { return leftEncoder.getEncoderDistance() * -1; }
    public double getRightEncoderDistance() { return rightEncoder.getEncoderDistance() * -1; }
    public double getBackEncoderDistance() { return backEncoder.getEncoderDistance(); }
    public double getYaw() { return -navx.getYaw(); }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public double getPoseX() { return lidarPoseX; }
    public double getPoseY() { return lidarPoseY; }
    public double getPoseHeading() { return lidarPoseHeading; }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    public void resetYaw() { navx.zeroYaw(); }

    public void resetOdometry() {
        resetEncoders();
        resetYaw();
        lidarPoseX = 0.0;
        lidarPoseY = 0.0;
        lidarPoseHeading = 0.0;
        prevScanPoints = null;
    }

    // ============================================================
    // PERIODIC EXECUTION
    // ============================================================

    @Override
    public void periodic() {
        processNetworkTableDrive();

        if (scanning) {
            scanData = lidar.getData();
            updateLidarOdometry();
        }

        if (controlTable.getEntry("ResetNavX").getBoolean(false)) {
            resetYaw();
            lidarPoseHeading = 0.0;
            controlTable.getEntry("ResetNavX").setBoolean(false);
        }

        // Telemetry
        leftEncoderValue.setDouble(getLeftEncoderDistance());
        rightEncoderValue.setDouble(getRightEncoderDistance());
        backEncoderValue.setDouble(getBackEncoderDistance());
        gyroValue.setDouble(getYaw());
        poseXValue.setDouble(lidarPoseX);
        poseYValue.setDouble(lidarPoseY);
        poseHeadingValue.setDouble(lidarPoseHeading);

        SmartDashboard.putNumber("Pose X", lidarPoseX);
        SmartDashboard.putNumber("Pose Y", lidarPoseY);
        SmartDashboard.putNumber("Pose Heading", getYaw());

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Cobra Ch" + i, getCobraVoltage(i));
        }

        driveTable.getEntry("PoseX").setDouble(lidarPoseX);
        driveTable.getEntry("PoseY").setDouble(lidarPoseY);
        driveTable.getEntry("PoseHeading").setDouble(getYaw());

        // Stream raw scan parts for dashboard mapping
        if (scanning && scanData != null && scanData.distance != null && scanData.angle != null) {
            int length = Math.min(scanData.distance.length, scanData.angle.length);
            if (length > 0) {
                int mid = length / 2;
                int len2 = length - mid;

                double[] angles1 = new double[mid];
                double[] distances1 = new double[mid];
                for (int i = 0; i < mid; i++) {
                    angles1[i] = scanData.angle[i] - LIDAR_OFFSET_DEGREES;
                    distances1[i] = scanData.distance[i];
                }

                double[] angles2 = new double[len2];
                double[] distances2 = new double[len2];
                for (int i = 0; i < len2; i++) {
                    angles2[i] = scanData.angle[mid + i] - LIDAR_OFFSET_DEGREES;
                    distances2[i] = scanData.distance[mid + i];
                }

                lidarTable.getEntry("ScanAngles_Part1").setDoubleArray(angles1);
                lidarTable.getEntry("ScanDistances_Part1").setDoubleArray(distances1);
                lidarTable.getEntry("ScanAngles_Part2").setDoubleArray(angles2);
                lidarTable.getEntry("ScanDistances_Part2").setDoubleArray(distances2);
            }
        }
    }
}