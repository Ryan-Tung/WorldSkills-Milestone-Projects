package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithPID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private double setpointDistance;
    private double setpointYaw; 
    private double targetYaw;

    PIDController pidYAxis;
    PIDController pidZAxis;

    public DriveWithPID(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw) {
        this.setpointDistance = setpointDistance;
        this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidYAxis = new PIDController(0.01, 0.0, 0);
        pidYAxis.setTolerance(epsilonDistance);

        pidZAxis = new PIDController(0.01, 0.0, 0);
        pidZAxis.setTolerance(epsilonYaw);
    }

    // Custom angle wrapping to keep values within [-180, 180] degrees
    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        pidYAxis.reset();
        pidZAxis.reset();

        // Lock target heading relative to current orientation when command starts
        targetYaw = normalizeAngle(drive.getYaw() + setpointYaw);
    }

    @Override
    public void execute() {
        // Calculate distance movement
        double yOutput = MathUtil.clamp(
            pidYAxis.calculate(drive.getAverageForwardEncoderDistance(), setpointDistance),
            -0.5,
            0.5
        );

        // Calculate heading correction across +/- 180 degree boundary
        double angleError = normalizeAngle(targetYaw - drive.getYaw());
        double zOutput = MathUtil.clamp(
            pidZAxis.calculate(0.0, angleError),
            -1.0,
            1.0
        );

        drive.holonomicDrive(0.0, yOutput, zOutput);
        SmartDashboard.putNumber("Yaw", drive.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drive.holonomicDrive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return pidYAxis.atSetpoint();
    }
}