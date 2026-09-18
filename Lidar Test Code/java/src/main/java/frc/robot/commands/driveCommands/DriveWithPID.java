package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithPID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    // Minimum motor output percentages required to overcome static friction
    private static final double kF_Y = 0.05; 
    private static final double kF_Z = 0.04; 

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

        targetYaw = normalizeAngle(drive.getYaw() + setpointYaw);
    }

    @Override
    public void execute() {
        // Calculate raw distance output
        double yOutput = pidYAxis.calculate(drive.getAverageForwardEncoderDistance(), setpointDistance);
        
        // Add friction compensation if outside distance tolerance
        if (!pidYAxis.atSetpoint() && Math.abs(yOutput) > 1e-4) {
            yOutput += Math.copySign(kF_Y, yOutput);
        }
        yOutput = MathUtil.clamp(yOutput, -0.5, 0.5);

        // Calculate raw yaw correction output
        double angleError = normalizeAngle(targetYaw - drive.getYaw());
        double zOutput = pidZAxis.calculate(0.0, angleError);

        // Add friction compensation if outside yaw tolerance
        if (!pidZAxis.atSetpoint() && Math.abs(zOutput) > 1e-4) {
            zOutput += Math.copySign(kF_Z, zOutput);
        }
        zOutput = MathUtil.clamp(zOutput, -1.0, 1.0);

        drive.holonomicDrive(0.0, yOutput, zOutput);
        SmartDashboard.putNumber("Yaw", drive.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drive.holonomicDrive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return pidYAxis.atSetpoint() && pidZAxis.atSetpoint();
    }
}