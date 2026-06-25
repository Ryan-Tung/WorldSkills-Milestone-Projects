package frc.robot.commands.auto;

import frc.robot.commands.driveCommands.DriveWithPID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.TurnWithPID;


public class DriveForwardWithPID extends AutoCommand
{
    public DriveForwardWithPID ()
    {
        //Drive 1000mm, 10mm error, maintain an angle of 0, 1 degree error

        // super(new DriveWithPID(1000, 10, 0, 1).withTimeout(5));
        //Timeout used just incase the command does not finish
        super (
            new SequentialCommandGroup
            (
                // 1. Drive the first leg of the square
                new DriveWithPID(1000, 10, 0, 1).withTimeout(5),
                
                // 2. Wait for 1.5 seconds
                new WaitCommand(1.5),
                
                // 3. Drive the second leg of the square
                new TurnWithPID(0, 10, 90, 1).withTimeout(5)
                
                // Add more legs and waits here to finish the square!
            )
        );
    }

}