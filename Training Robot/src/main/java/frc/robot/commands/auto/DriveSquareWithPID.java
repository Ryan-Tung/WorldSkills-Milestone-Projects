package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.TurnWithPID;

public class DriveSquareWithPID extends AutoCommand {
    
    public DriveSquareWithPID() {
        super (
            new SequentialCommandGroup
            (
                // 1. Drive the first leg of the square
                new DriveWithPID(300, 1, 0, 1).withTimeout(2),
                
                // 3. Drive the second leg of the square
                new TurnWithPID(0, 10, 90, 1).withTimeout(2)
                
            )
        );
    }
}