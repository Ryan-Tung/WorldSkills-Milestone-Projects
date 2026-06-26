package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.TurnWithPID;
import frc.robot.commands.driveCommands.SimpleDrive; 
import edu.wpi.first.wpilibj.AnalogInput;

public class DriveAroundObstacleWithPID extends AutoCommand {
    
    // =========================================================================
    // TUNABLE CONSTANTS (Adjust these to match your real-world arena)
    // =========================================================================
    private static final int IR_SENSOR_PORT = 0;
    
    // IR Sensor Thresholds (Measured in CM)
    private static final double MIN_DETECTION_CM = 10.0;
    private static final double MAX_DETECTION_CM = 80.0;
    private static final double APPROACH_STOP_CM = 15.0;  // Distance to stop in front of the box
    
    // Motor Speeds & Directions
    private static final double FORWARD_SPEED = 0.5;
    private static final double CRAB_Y_SPEED = 0.0;
    
    // Outward Crab Walk (Stepping away from centerline)
    private static final double CRAB_X_SPEED = 0.5;
    private static final double CRAB_ROT_CORRECTION = -0.05;
    
    // Return Crab Walk (Stepping back to centerline)
    // Flipped to negative since the robot faces forward again and must move left
    private static final double RETURN_CRAB_X_SPEED = -0.5; 
    private static final double RETURN_ROT_CORRECTION = 0.05; 
    
    // Timeouts (Measured in Seconds)
    private static final double WAIT_TIME = 0.25;
    private static final double FIRST_CLEAR_TIMEOUT = 2;   // Time to clear robot width
    private static final double FORWARD_DRIVE_TIMEOUT = 1.5; // Time to drive past box length
    private static final double SECOND_CLEAR_TIMEOUT = 1.5;  // Time to clear front corner of box
    private static final double RETURN_LINE_TIMEOUT = 2.0;   // Time to crab walk back to centerline
    private static final double FINAL_DRIVE_TIMEOUT = 5.0;    // Final straight away duration
    // =========================================================================

    // Hardware Sensor Initialization
    private final AnalogInput sharp = new AnalogInput(IR_SENSOR_PORT);

    /**
     * Fully automated obstacle avoidance.
     * Requires 0 parameters; relies entirely on the IR sensor and tuned constants.
     */
    public DriveAroundObstacleWithPID() {
        super(new SequentialCommandGroup());

        addCommands(
            // Step 1: Run straight toward the box until the sensor detects it within range
            new SimpleDrive(0.0, FORWARD_SPEED, 0.0) {
                @Override
                public boolean isFinished() {
                    double distance = getDistance();
                    return (distance <= APPROACH_STOP_CM && distance >= MIN_DETECTION_CM);
                }
            },
            new WaitCommand(WAIT_TIME),
            
            // Step 2: Crab walk sideways out of the way until the sensor no longer sees the object
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),
            
            // Step 3: Continue crab walking so the entire chassis clears the box width
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(FIRST_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 4: Drive forward to clear the side profile length of the box
            new SimpleDrive(0.0, FORWARD_SPEED, 0.0).withTimeout(FORWARD_DRIVE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 5: Turn 90 degrees Left to point the front IR sensor back toward the obstacle
            new TurnWithPID(0, 10, -90, 1),
            new WaitCommand(WAIT_TIME),

            // Step 6: Crab walk along the box profile until the sensor clears it again
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 7: Continue crab walking to safely clear the front corner of the box
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(SECOND_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 8: Turn 90 degrees Right to face back towards the original direction (front)
            new TurnWithPID(0, 10, 90, 1),
            new WaitCommand(WAIT_TIME),

            // Step 9: Crab walk sideways using return variables to get back onto the centerline
            new SimpleDrive(RETURN_CRAB_X_SPEED, CRAB_Y_SPEED, RETURN_ROT_CORRECTION).withTimeout(RETURN_LINE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 10: Continue driving straight down the original track layout
            new SimpleDrive(0.0, FORWARD_SPEED, 0.0).withTimeout(FINAL_DRIVE_TIMEOUT)
        );
    }

    /**
     * Calculates sensor distance.
     * @return distance in centimeters (CM)
     */
    public double getDistance() {
        return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }
    
    /**
     * Evaluates if an object is currently within the reliable tracking bounds of the sensor.
     * @return true if an object is actively detected between 10cm and 80cm
     */
    public boolean isObjectDetected() {
        double distance = getDistance();
        return (distance >= MIN_DETECTION_CM && distance <= MAX_DETECTION_CM);
    }
}