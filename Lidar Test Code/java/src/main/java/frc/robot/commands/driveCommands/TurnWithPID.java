package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnWithPID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    // Minimum motor output percentage to overcome rotational static friction
    private static final double kF_Z = 0.04; 

    private double setpointDistance;
    private double setpointYaw; 
    private double targetYaw;

    PIDController pidYAxis;
    PIDController pidZAxis;

    public TurnWithPID(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw) {
        this.setpointDistance = setpointDistance;
        this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidYAxis = new PIDController(0, 0, 0);
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
        double angleError = normalizeAngle(targetYaw - drive.getYaw());
        double zOutput = pidZAxis.calculate(0.0, angleError);

        // Add rotational friction compensation if outside target tolerance
        if (!pidZAxis.atSetpoint() && Math.abs(zOutput) > 1e-4) {
            zOutput += Math.copySign(kF_Z, zOutput);
        }
        zOutput = MathUtil.clamp(zOutput, -1.0, 1.0);

        drive.holonomicDrive(0.0, 0.0, zOutput);
        SmartDashboard.putNumber("Yaw", drive.getYaw());
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