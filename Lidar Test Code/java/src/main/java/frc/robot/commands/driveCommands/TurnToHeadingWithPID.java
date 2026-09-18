package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnToHeadingWithPID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private final double targetHeading;
    private final PIDController pidZAxis;
    private static final double MIN_POWER = 0.08; // 8% minimum power to overcome static friction

    public TurnToHeadingWithPID(double targetHeading, double yawTolerance) {
        this.targetHeading = normalizeAngle(targetHeading);
        addRequirements(drive);

        pidZAxis = new PIDController(0.012, 0.0, 0.001);
        pidZAxis.setTolerance(yawTolerance);
    }

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
        pidZAxis.reset();
    }

    @Override
    public void execute() {
        double angleError = normalizeAngle(targetHeading - drive.getYaw());
        double rawZ = pidZAxis.calculate(0.0, angleError);

        double zOutput = 0.0;
        if (!pidZAxis.atSetpoint()) {
            double magnitude = Math.max(Math.abs(rawZ), MIN_POWER);
            zOutput = Math.copySign(magnitude, rawZ);
        }

        zOutput = MathUtil.clamp(zOutput, -0.4, 0.4);
        drive.holonomicDrive(0.0, 0.0, zOutput);

        SmartDashboard.putNumber("Target Absolute Heading", targetHeading);
        SmartDashboard.putNumber("Current Yaw", drive.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drive.holonomicDrive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return pidZAxis.atSetpoint();
    }
}