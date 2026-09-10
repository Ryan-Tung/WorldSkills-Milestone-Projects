package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NetPrinter_v2;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.driveCommands.SimpleDrive; 
import frc.robot.commands.driveCommands.DriveWithPID; 

public class DriveAroundObstacleWithPID extends AutoCommand {

    // =========================================================================
    // TUNABLE CONSTANTS (Adjust these to match your real-world arena)
    // =========================================================================
    // Lidar Thresholds (Measured in CM)
    private static final double MIN_DETECTION_CM = 10.0;
    private static final double MAX_DETECTION_CM = 80.0;
    private static final double APPROACH_STOP_CM = 30.0;   // Distance to stop in front of the box
    private static final double SIDE_DETECTION_CM = 150.0;  // Threshold to consider box detected on side

    // Motor Speeds & Directions
    private static final double FORWARD_SPEED = 0.5;
    private static final double CRAB_Y_SPEED = 0.0;

    // Outward Crab Walk (Stepping away from centerline)
    private static final double CRAB_X_SPEED = 0.5;
    private static final double CRAB_ROT_CORRECTION = -0.05;

    // Return Crab Walk (Stepping back to centerline)
    private static final double RETURN_CRAB_X_SPEED = -0.5; 
    private static final double RETURN_ROT_CORRECTION = 0.05; 

    // Timeouts (Measured in Seconds)
    private static final double WAIT_TIME = 0.25;
    private static final double FIRST_CLEAR_TIMEOUT = 2.5;   // Time to clear robot width sideways
    private static final double FORWARD_DRIVE_TIMEOUT = 1.5; // Extra time to clear rear bumper past box
    private static final double RETURN_LINE_TIMEOUT = 3.0;   // Time to crab walk back to centerline
    private static final double FINAL_DRIVE_TIMEOUT = 5.0;   // Final straightaway duration
    // =========================================================================

    private final DriveTrain driveTrain;

    /**
     * Fully automated obstacle avoidance using 0° and side (270°/90°) LiDAR distances.
     * @param driveTrain The DriveTrain subsystem instance
     */
    public DriveAroundObstacleWithPID(DriveTrain driveTrain) {
        super(new SequentialCommandGroup());
        this.driveTrain = driveTrain;

        NetPrinter_v2.printf("LidarLog", "LIDAR INIT COMPLETE");

        addCommands(
            // Step 1: Run straight toward the box until 0° LiDAR detects it within approach range
            new DriveWithPID(1000, 1, 0, 1) {
                @Override
                public boolean isFinished() {
                    double distance = get0Distance();
                    return (distance <= APPROACH_STOP_CM && distance >= MIN_DETECTION_CM);
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 2: Crab walk sideways out of the way until 0° LiDAR no longer sees the object
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 3: Continue crab walking sideways so the entire chassis width clears the box
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(FIRST_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 4: Drive forward until side LiDAR detects the box alongside the robot
            new DriveWithPID(1000, 1, 0, 1) {
                @Override
                public boolean isFinished() {
                    double distance = get270Distance(); // Use get90Distance() if box is on the 90° side
                    NetPrinter_v2.printf("LidarLog", "Searching side box distance: " + distance);
                    return (distance > 0 && distance <= SIDE_DETECTION_CM);
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 5: Drive forward along the box until side LiDAR no longer detects it
            new DriveWithPID(1000, 1, 0, 1) {
                @Override
                public boolean isFinished() {
                    double distance = get270Distance(); // Use get90Distance() if box is on the 90° side
                    NetPrinter_v2.printf("LidarLog", "Passing side box distance: " + distance);
                    return (distance <= 0 || distance > SIDE_DETECTION_CM);
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 6: Drive forward extra time so the rear bumper fully clears past the box
            new DriveWithPID(1000, 1, 0, 1).withTimeout(FORWARD_DRIVE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 7: Crab walk sideways back onto the original centerline
            new SimpleDrive(RETURN_CRAB_X_SPEED, CRAB_Y_SPEED, RETURN_ROT_CORRECTION).withTimeout(RETURN_LINE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 8: Continue driving straight down the original path
            new DriveWithPID(1000, 1, 0, 1).withTimeout(FINAL_DRIVE_TIMEOUT)
        );
    }

    /**
     * @return distance directly ahead at 0 degrees in CM
     */
    public double get0Distance() {
        return driveTrain.getLidarAtZeroDegrees();
    }

    /**
     * @return distance to the side at 270 degrees in CM
     */
    public double get270Distance() {
        return driveTrain.getLidarAt270Degrees();
    }

    /**
     * @return distance to the side at 90 degrees in CM
     */
    // public double get90Distance() {
    //     return driveTrain.getLidarAt90Degrees();
    // }

    /**
     * Evaluates if an object is actively detected in front between 10cm and 80cm
     */
    public boolean isObjectDetected() {
        double distance = get0Distance();
        return (distance >= MIN_DETECTION_CM && distance <= MAX_DETECTION_CM);
    }
}