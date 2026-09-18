package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class AlignToCornerWithLidar extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private double targetDistY;
    private double targetDistX;

    private final PIDController pidYAxis;
    private final PIDController pidXAxis;
    private final PIDController pidZAxis;

    private static final double MIN_OUTPUT = 0.06; // Minimum 6% power to overcome static friction

    public AlignToCornerWithLidar(double distanceTolerance, double yawTolerance) {
        addRequirements(drive);

        pidYAxis = new PIDController(0.015, 0.0, 0.001);
        pidYAxis.setTolerance(distanceTolerance);

        pidXAxis = new PIDController(0.015, 0.0, 0.001);
        pidXAxis.setTolerance(distanceTolerance);

        pidZAxis = new PIDController(0.01, 0.0, 0.0);
        pidZAxis.setTolerance(yawTolerance);
    }

    public AlignToCornerWithLidar() {
        this(1.0, 1.0);
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

    // Applies minimum power threshold to prevent friction stalling
    private double addFrictionCompensation(double pidOutput, boolean atSetpoint) {
        if (atSetpoint || Math.abs(pidOutput) < 0.001) {
            return 0.0;
        }
        double magnitude = Math.max(Math.abs(pidOutput), MIN_OUTPUT);
        return Math.copySign(magnitude, pidOutput);
    }

    @Override
    public void initialize() {
        targetDistY = drive.getInitialCornerY();
        targetDistX = drive.getInitialCornerX();

        pidYAxis.reset();
        pidXAxis.reset();
        pidZAxis.reset();

        NetPrinter_v2.printf("LidarLog", "EVENT: ALIGN INIT -> Target Y: %.2f cm | Target X: %.2f cm", targetDistY, targetDistX);
    }

    @Override
    public void execute() {
        double currentDist0 = drive.getLidarAtZeroDegrees();
        double currentDist270 = drive.getLidarAt270Degrees();

        double yOutput = 0.0;
        double xOutput = 0.0;

        // Process Y axis (0 deg LiDAR) if reading is valid
        if (currentDist0 > 0.0 && currentDist0 < 900.0) {
            double rawY = -pidYAxis.calculate(currentDist0, targetDistY);
            yOutput = MathUtil.clamp(addFrictionCompensation(rawY, pidYAxis.atSetpoint()), -0.25, 0.25);
        }

        // Process X axis (270 deg LiDAR) if reading is valid
        if (currentDist270 > 0.0 && currentDist270 < 900.0 && targetDistX > 0.0) {
            double rawX = -pidXAxis.calculate(currentDist270, targetDistX);
            xOutput = MathUtil.clamp(addFrictionCompensation(rawX, pidXAxis.atSetpoint()), -0.25, 0.25);
        }

        double angleError = normalizeAngle(0.0 - drive.getYaw());
        double rawZ = pidZAxis.calculate(0.0, angleError);
        double zOutput = MathUtil.clamp(rawZ, -0.2, 0.2);

        NetPrinter_v2.printf("LidarLog", "EXEC Align: 0Deg=%.2f (Tgt=%.2f, Y_Out=%.3f) | 270Deg=%.2f (Tgt=%.2f, X_Out=%.3f) | Yaw=%.2f",
            currentDist0, targetDistY, yOutput, currentDist270, targetDistX, xOutput, drive.getYaw());

        drive.holonomicDrive(xOutput, yOutput, zOutput);
    }

    @Override
    public void end(boolean interrupted) {
        drive.holonomicDrive(0.0, 0.0, 0.0);
        if (!interrupted) {
            drive.relocalizeFromCorner();
            NetPrinter_v2.printf("LidarLog", "EVENT: CORNER LIDAR ALIGNMENT COMPLETE -> Odometry Relocalized");
        }
    }

    @Override
    public boolean isFinished() {
        // If X reading is invalid (-0.10), finish based on Y axis and heading alignment alone
        boolean xFinished = (targetDistX <= 0.0) || pidXAxis.atSetpoint();
        return pidYAxis.atSetpoint() && xFinished && pidZAxis.atSetpoint();
    }
}