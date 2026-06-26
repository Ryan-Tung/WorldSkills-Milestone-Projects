package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.TurnWithPID;

public class DriveAroundObstacleWithPID extends AutoCommand {
    
    /**
     * Clear an obstacle and continue straight.
     * * @param distanceToBox         Distance from start (0.0) to the front of the box
     * @param boxSize               The width/length of the box obstacle
     * @param finalStraightDistance How far to keep going straight after clearing the box
     */
    public DriveAroundObstacleWithPID(double distanceToBox, double boxSize, double finalStraightDistance) {
        // 1. Create the main sequential group
        super(new SequentialCommandGroup());
        
        // Safety buffer so the robot doesn't scrape the side of the box
        double clearance = 400.0; 
        double detourDistance = boxSize + clearance;
        double forwardError = 1;

        addCommands(
            // Step 1: Drive straight to the obstacle
            // new DriveWithPID(distanceToBox, forwardError, 0, 1).withTimeout(3),
            new DriveWithPID(300, forwardError, 0, 1).withTimeout(2),
            
            // new WaitCommand(1.5),
            // Step 2: Turn 90 degrees Right to step out of the way
            new TurnWithPID(0, 10, 90, 1).withTimeout(2),
            // new WaitCommand(1.5),

            // // Step 3: Drive sideways to clear the width of the box
            new DriveWithPID(detourDistance, forwardError, 0, 1).withTimeout(2),
            // new WaitCommand(1.5),

            // // // Step 4: Turn 90 degrees Left (facing forward again)
            new TurnWithPID(0, 10, -90, 1).withTimeout(2),
            // new WaitCommand(1.5)

            // // Step 5: Drive forward past the length of the box
            new DriveWithPID(detourDistance, forwardError, 0, 1).withTimeout(2),
            // // new SimpleDrive(0.0, 0.5, 0.0).withTimeout(5),
            // new WaitCommand(1.5),


            // // Step 6: Turn 90 degrees Left to head back to the original centerline
            new TurnWithPID(0, 10, -90, 0.1).withTimeout(2),
            new WaitCommand(1.5),

            // // Step 7: Drive back to the original centerline path
            new DriveWithPID(detourDistance, forwardError, 0, 1).withTimeout(2),
            // new WaitCommand(1.5),

            // // Step 8: Turn 90 degrees Right to face forward again
            new TurnWithPID(0, 10, 90, 0.1).withTimeout(2),
            // new WaitCommand(1.5),

            // // Step 9: Keep going straight on the original path
            new DriveWithPID(finalStraightDistance, forwardError, 0, 1).withTimeout(4)
        );
    }
}