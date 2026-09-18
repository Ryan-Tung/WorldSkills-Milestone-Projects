package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnWithPID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

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

    // Custom angle wrapping that works on any version of Java or WPILib
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

        // Calculate target heading relative to current gyro position
        targetYaw = normalizeAngle(drive.getYaw() + setpointYaw);
    }

    @Override
    public void execute() {
        // Calculate shortest angular path (-180 to 180 degrees)
        double angleError = normalizeAngle(targetYaw - drive.getYaw());

        // Feed remaining error into PID controller (0 measurement, angleError target)
        double output = pidZAxis.calculate(0.0, angleError);

        drive.holonomicDrive(0.0, 0.0, MathUtil.clamp(output, -1.0, 1.0));
        SmartDashboard.putNumber("Yaw", drive.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return pidZAxis.atSetpoint();
    }
}