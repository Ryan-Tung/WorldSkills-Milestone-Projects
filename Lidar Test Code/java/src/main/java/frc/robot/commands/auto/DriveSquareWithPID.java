package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.AlignToCornerWithLidar;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.TurnWithPID;
import frc.robot.commands.driveCommands.TurnToHeadingWithPID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class DriveSquareWithPID extends AutoCommand {

    private static final double WAIT_TIME = 0.25; // Pause duration in seconds between moves

    private final DriveTrain driveTrain;

    public DriveSquareWithPID(DriveTrain driveTrain) {
        super(new SequentialCommandGroup());
        this.driveTrain = driveTrain;

        // 1. Log initialization & calibrate initial pose using LiDAR corner walls
        addCommands(
            new InstantCommand(() -> {
                NetPrinter_v2.printf("LidarLog", "EVENT: AUTO SQUARE SEQUENCE INITIALIZED");
                driveTrain.calibrateCornerPosition();
            }, driveTrain),
            new WaitCommand(WAIT_TIME)
        );

        // 2. Turn 180 degrees to face outward away from the corner
        addCommands(
            new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 2 - TURNING 180 DEGREES OUTWARD")),
            new TurnWithPID(0, 10, 180, 1).withTimeout(20),
            new WaitCommand(WAIT_TIME)
        );

        // 3. Drive forward to clear the corner wall bounds
        addCommands(
            new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 3 - DRIVING FORWARD TO CLEAR CORNER")),
            new DriveWithPID(500, 1, 0, 10).withTimeout(20),
            new WaitCommand(WAIT_TIME)
        );

        // 4. Drive in a square (4 sides)
        for (int i = 0; i < 4; i++) {
            final int sideNumber = i + 1;
            addCommands(
                new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 4." + sideNumber + "a - DRIVING SIDE " + sideNumber + " OF SQUARE")),
                new DriveWithPID(500, 1, 0, 10).withTimeout(20),
                new WaitCommand(WAIT_TIME),

                new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 4." + sideNumber + "b - TURNING 90 DEGREES FOR CORNER " + sideNumber)),
                new TurnWithPID(0, 10, 90, 1).withTimeout(20),
                new WaitCommand(WAIT_TIME)
            );
        }

        // 5. Drive back into general corner region (reverse clear step)
        addCommands(
            new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 5 - REVERSING BACK NEAR CORNER REGION")),
            new DriveWithPID(-500, 10, 0, 10).withTimeout(20),
            new WaitCommand(WAIT_TIME)
        );

        // 6. Return to original rotation (0 degrees heading) so 0 deg and 270 deg LiDAR sensors face corner walls
        addCommands(
            new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 6 - ROTATING BACK TO ORIGINAL YAW (0 DEGREES)")),
            new TurnToHeadingWithPID(0, 1).withTimeout(20),
            new WaitCommand(WAIT_TIME)
        );

        // 7. Closed-loop LiDAR alignment back to baseline distance readings & odometry reset
        addCommands(
            new InstantCommand(() -> NetPrinter_v2.printf("LidarLog", "EVENT: STEP 7 - ALIGNING TO ORIGINAL CORNER POSITION VIA LIDAR READINGS")),
            new AlignToCornerWithLidar().withTimeout(10)
        );
    }
}