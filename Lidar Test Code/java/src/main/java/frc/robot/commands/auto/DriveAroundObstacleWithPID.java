package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NetPrinter_v2;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.driveCommands.SimpleDrive; 
import frc.robot.commands.driveCommands.DriveWithPID; 

public class DriveAroundObstacleWithPID extends AutoCommand {

    // =========================================================================
    // TUNABLE CONSTANTS
    // =========================================================================
    private static final double MIN_DETECTION_CM = 10.0;
    private static final double MAX_DETECTION_CM = 80.0;
    private static final double APPROACH_STOP_CM = 30.0;
    private static final double SIDE_DETECTION_CM = 100.0;

    private static final double FORWARD_SPEED = 0.5;
    private static final double CRAB_Y_SPEED = 0.0;

    private static final double CRAB_X_SPEED = 0.5;
    private static final double CRAB_ROT_CORRECTION = -0.05;

    private static final double RETURN_CRAB_X_SPEED = -0.5; 
    private static final double RETURN_ROT_CORRECTION = 0.05; 

    private static final double WAIT_TIME = 0.25;
    private static final double FIRST_CLEAR_TIMEOUT = 2.5;
    private static final double FORWARD_DRIVE_TIMEOUT = 1.5;
    private static final double RETURN_LINE_TIMEOUT = 3.0;
    private static final double FINAL_TAPE_TIMEOUT = 8.0; // Max safety timeout while driving to tape
    // =========================================================================

    private final DriveTrain driveTrain;

    public DriveAroundObstacleWithPID(DriveTrain driveTrain) {
        super(new SequentialCommandGroup());
        this.driveTrain = driveTrain;

        addCommands(
            // Step 0: Calibrate Cobra IR sensor on white floor baseline
            new InstantCommand(() -> {
                NetPrinter_v2.printf("IRLog", "EVENT: STARTING WHITE FLOOR IR CALIBRATION");
                driveTrain.calibrateCobraWhite();
                NetPrinter_v2.printf("LidarLog", "EVENT: AUTO OBSTACLE SEQUENCE INITIALIZED");
            }),

            // Step 1: Drive toward box until front LiDAR detects distance threshold
            new DriveWithPID(1000, 1, 0, 1) {
                private boolean logged = false;
                @Override
                public void initialize() {
                    super.initialize();
                    NetPrinter_v2.printf("LidarLog", "EVENT: STEP 1 - APPROACHING OBSTACLE");
                }
                @Override
                public boolean isFinished() {
                    double distance = get0Distance();
                    boolean detected = (distance <= APPROACH_STOP_CM && distance >= MIN_DETECTION_CM);
                    if (detected && !logged) {
                        NetPrinter_v2.printf("LidarLog", "EVENT: OBSTACLE DETECTED AT " + distance + " CM");
                        logged = true;
                    }
                    return detected;
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 2: Crab walk sideways until 0° LiDAR clears object
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public void initialize() {
                    super.initialize();
                    NetPrinter_v2.printf("LidarLog", "EVENT: STEP 2 - CRABBING SIDEWAYS");
                }
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 3: Continue crab walking to ensure chassis clearance
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(FIRST_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 4: Drive forward until side LiDAR (270°) sees the box
            new DriveWithPID(1000, 1, 0, 1) {
                private boolean found = false;
                @Override
                public void initialize() {
                    super.initialize();
                    NetPrinter_v2.printf("LidarLog", "EVENT: STEP 4 - SEARCHING SIDE BOX");
                }
                @Override
                public boolean isFinished() {
                    double distance = get270Distance();
                    boolean inRange = (distance > 0 && distance <= SIDE_DETECTION_CM);
                    if (inRange && !found) {
                        NetPrinter_v2.printf("LidarLog", "EVENT: SIDE BOX DETECTED AT " + distance + " CM");
                        found = true;
                    }
                    return inRange;
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 5: Drive forward until side LiDAR loses sight of box
            new DriveWithPID(1000, 1, 0, 1) {
                private boolean passed = false;
                @Override
                public void initialize() {
                    super.initialize();
                    NetPrinter_v2.printf("LidarLog", "EVENT: STEP 5 - PASSING ALONGSIDE BOX");
                }
                @Override
                public boolean isFinished() {
                    double distance = get270Distance();
                    boolean cleared = (distance <= 0 || distance > SIDE_DETECTION_CM);
                    if (cleared && !passed) {
                        NetPrinter_v2.printf("LidarLog", "EVENT: SIDE BOX CLEARED");
                        passed = true;
                    }
                    return cleared;
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 6: Clear rear bumper past box
            new DriveWithPID(1000, 1, 0, 1).withTimeout(FORWARD_DRIVE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 7: Crab walk sideways back to original centerline
            new SimpleDrive(RETURN_CRAB_X_SPEED, CRAB_Y_SPEED, RETURN_ROT_CORRECTION).withTimeout(RETURN_LINE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 8: Drive forward and STOP when black tape is detected by Cobra IR sensor
            new DriveWithPID(1000, 1, 0, 1) {
                @Override
                public void initialize() {
                    super.initialize();
                    NetPrinter_v2.printf("IRLog", "EVENT: STEP 8 - SEARCHING FOR BLACK TAPE");
                }
                @Override
                public boolean isFinished() {
                    boolean tapeDetected = driveTrain.isAnyTapeDetected();
                    if (tapeDetected) {
                        NetPrinter_v2.printf("IRLog", "EVENT: BLACK TAPE DETECTED - STOPPING");
                    }
                    return tapeDetected;
                }
            }.withTimeout(FINAL_TAPE_TIMEOUT)
        );
    }

    public double get0Distance() {
        return driveTrain.getLidarAtZeroDegrees();
    }

    public double get270Distance() {
        return driveTrain.getLidarAt270Degrees();
    }

    public boolean isObjectDetected() {
        double distance = get0Distance();
        return (distance >= MIN_DETECTION_CM && distance <= MAX_DETECTION_CM);
    }
}