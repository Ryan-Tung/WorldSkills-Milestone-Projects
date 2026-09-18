package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class CalibrateCornerCommand extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;
    private boolean isCalibrated = false;

    public CalibrateCornerCommand() {
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetYaw();
        isCalibrated = false;
        drive.startScan(); // Turn on LiDAR hardware stream
    }

    @Override
    public void execute() {
        double baselineY = drive.getExactLidarReading(0.0);
        double baselineX = drive.getExactLidarReading(270.0);

        if (baselineY > 0.0 && baselineY < 800.0 && baselineX > 0.0 && baselineX < 800.0) {
            drive.setInitialCornerY(baselineY);
            drive.setInitialCornerX(baselineX);
            isCalibrated = true;
            NetPrinter_v2.printf("LidarLog", "CALIBRATION SUCCESS: Baseline X=%.2f CM, Baseline Y=%.2f CM", baselineX, baselineY);
        }
    }

    @Override
    public boolean isFinished() {
        return isCalibrated; // Hold step until valid readings arrive
    }
}