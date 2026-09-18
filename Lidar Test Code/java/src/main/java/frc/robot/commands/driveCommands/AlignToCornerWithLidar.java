package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class AlignToCornerWithLidar extends CommandBase {
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private double targetDistY;
    private double targetDistX;

    private double lastValidY = -1.0;
    private double lastValidX = -1.0;

    // Tracks raw readings to detect stale frame data
    private double prevRawY = -1.0;
    private double prevRawX = -1.0;

    private final PIDController pidYAxis;
    private final PIDController pidXAxis;
    private final PIDController pidZAxis;

    private static final double MIN_OUTPUT = 0.50; // Minimum power threshold below max clamp
    private static final double MAX_ALLOWED_JUMP = 15.0; // Filters corner reflection jumps

    private int atSetpointTicks = 0;
    private static final int REQUIRED_SETPOINT_TICKS = 5; // Requires 100ms (5 x 20ms) stable setpoint hold

    public AlignToCornerWithLidar(double distanceTolerance, double yawTolerance) {
        addRequirements(drive);

        pidYAxis = new PIDController(0.015, 0.0, 0.001);
        pidYAxis.setTolerance(distanceTolerance);

        pidXAxis = new PIDController(0.015, 0.0, 0.001);
        pidXAxis.setTolerance(distanceTolerance);

        pidZAxis = new PIDController(0.012, 0.0, 0.0);
        pidZAxis.setTolerance(yawTolerance);
        pidZAxis.enableContinuousInput(-180.0, 180.0);
    }

    public AlignToCornerWithLidar() {
        this(0.5, 0.5);
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

    private double addFrictionCompensation(double pidOutput, boolean atSetpoint) {
        if (atSetpoint || Math.abs(pidOutput) < 0.001) {
            return 0.0;
        }
        double magnitude = Math.max(Math.abs(pidOutput), MIN_OUTPUT);
        return Math.copySign(magnitude, pidOutput);
    }

    @Override
    public void initialize() {
        drive.startScan();

        targetDistY = drive.getInitialCornerY();
        targetDistX = drive.getInitialCornerX();

        if (targetDistY <= 0.0) targetDistY = drive.getExactLidarReading(0.0);
        if (targetDistX <= 0.0) targetDistX = drive.getExactLidarReading(270.0);

        lastValidY = -1.0;
        lastValidX = -1.0;
        prevRawY = -1.0;
        prevRawX = -1.0;
        atSetpointTicks = 0;

        pidYAxis.reset();
        pidXAxis.reset();
        pidZAxis.reset();

        NetPrinter_v2.printf("LidarLog", "EVENT: ALIGN INIT -> Target Y: %.2f cm | Target X: %.2f cm", targetDistY, targetDistX);
    }

    @Override
    public void execute() {
        double raw0 = drive.getExactLidarReading(0.0);
        double raw270 = drive.getExactLidarReading(270.0);

        boolean yUpdated = false;
        boolean xUpdated = false;

        // Verify Y-axis reading freshness
        if (raw0 > 0.0 && raw0 < 800.0) {
            if (Double.compare(raw0, prevRawY) != 0) { // New frame received
                if (lastValidY < 0.0 || Math.abs(raw0 - lastValidY) <= MAX_ALLOWED_JUMP) {
                    lastValidY = raw0;
                    yUpdated = true;
                }
                prevRawY = raw0;
            }
        }

        // Verify X-axis reading freshness
        if (raw270 > 0.0 && raw270 < 800.0) {
            if (Double.compare(raw270, prevRawX) != 0) { // New frame received
                if (lastValidX < 0.0 || Math.abs(raw270 - lastValidX) <= MAX_ALLOWED_JUMP) {
                    lastValidX = raw270;
                    xUpdated = true;
                }
                prevRawX = raw270;
            }
        }

        double yOutput = 0.0;
        double xOutput = 0.0;

        // Drive axis only when a new frame update is confirmed
        if (yUpdated && lastValidY > 0.0) {
            double rawY = -pidYAxis.calculate(lastValidY, targetDistY); // Direction sign fixed
            yOutput = MathUtil.clamp(addFrictionCompensation(rawY, pidYAxis.atSetpoint()), -0.20, 0.20);
        }

        if (xUpdated && lastValidX > 0.0) {
            double rawX = pidXAxis.calculate(lastValidX, targetDistX); // Direction sign fixed
            xOutput = MathUtil.clamp(addFrictionCompensation(rawX, pidXAxis.atSetpoint()), -0.90, 0.90);
        }

        double angleError = normalizeAngle(0.0 - drive.getYaw());
        double rawZ = pidZAxis.calculate(0.0, angleError);
        double zOutput = MathUtil.clamp(rawZ, -0.15, 0.15);

        NetPrinter_v2.printf("LidarLog", "EXEC Align: 0Deg=%.2f (Tgt=%.2f, Y_Out=%.3f, Fresh=%b) | 270Deg=%.2f (Tgt=%.2f, X_Out=%.3f, Fresh=%b) | Yaw=%.2f",
            lastValidY, targetDistY, yOutput, yUpdated, lastValidX, targetDistX, xOutput, xUpdated, drive.getYaw());

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
        boolean yFinished = (lastValidY > 0.0) && pidYAxis.atSetpoint();
        boolean xFinished = (targetDistX <= 0.0) || ((lastValidX > 0.0) && pidXAxis.atSetpoint());
        boolean zFinished = pidZAxis.atSetpoint();

        if (yFinished && xFinished && zFinished) {
            atSetpointTicks++;
        } else {
            atSetpointTicks = 0;
        }

        return atSetpointTicks >= REQUIRED_SETPOINT_TICKS;
    }
}