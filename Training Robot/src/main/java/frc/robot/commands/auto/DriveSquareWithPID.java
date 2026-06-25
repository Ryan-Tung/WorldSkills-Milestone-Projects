package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.TurnWithPID;

public class DriveSquareWithPID extends AutoCommand {
    
    public DriveSquareWithPID() {
        super (
            // 1. Drive the first leg of the square
            new DriveWithPID(10, 10, 0, 1).withTimeout(5),
            
            // 2. Wait for 1.5 seconds
            new WaitCommand(1.5), 
            
            // 3. Drive the second leg of the square
            new TurnWithPID(0, 10, 90, 1).withTimeout(5)
            
            // Add more legs and waits here to finish the square!
        );
    }
}