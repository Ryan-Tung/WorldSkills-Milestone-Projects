package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class DriveAroundObstacleThreaded extends AutoCommand implements Runnable {

    // =========================================================================
    // TUNABLE CONSTANTS
    // =========================================================================
    private static final double MIN_DETECTION_CM = 10.0;
    private static final double MAX_DETECTION_CM = 80.0;
    private static final double APPROACH_STOP_CM = 30.0;
    private static final double SIDE_DETECTION_CM = 50.0;

    private static final double FORWARD_SPEED = 0.4;
    private static final double CRAB_X_SPEED = 0.5;
    private static final double CRAB_ROT_CORRECTION = -0.05;

    private static final double RETURN_CRAB_X_SPEED = -0.5;
    private static final double RETURN_ROT_CORRECTION = 0.05;

    private static final long FIRST_CLEAR_MS = 2500;   // Time to clear chassis width
    private static final long FORWARD_DRIVE_MS = 1500; // Time to clear rear bumper
    private static final long RETURN_LINE_MS = 3000;   // Time to crab walk back to centerline
    // =========================================================================

    private final DriveTrain driveTrain;
    private Thread avoidanceThread;
    private volatile boolean isRunning = false;

    private enum State {
        DRIVING_FORWARD,
        STRAFE_OUT,
        STRAFE_EXTRA_CLEAR,
        DRIVE_FIND_SIDE,
        DRIVE_PASS_SIDE,
        DRIVE_CLEAR_REAR,
        STRAFE_RETURN
    }

    private State currentState = State.DRIVING_FORWARD;

    public DriveAroundObstacleThreaded(DriveTrain driveTrain) {
        super(new InstantCommand()); // Satisfies AutoCommand inheritance
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        isRunning = true;
        currentState = State.DRIVING_FORWARD;
        avoidanceThread = new Thread(this, "ObstacleAvoidanceThread");
        avoidanceThread.start();
        NetPrinter_v2.printf("LidarLog", "THREAD STARTED: Repeatable Obstacle Avoidance");
    }

    @Override
    public void run() {
        long timerStartTime = 0;

        while (isRunning && !Thread.currentThread().isInterrupted()) {
            double dist0 = driveTrain.getLidarAtZeroDegrees();
            double dist270 = driveTrain.getLidarAt270Degrees();

            switch (currentState) {

                case DRIVING_FORWARD:
                    driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                    if (dist0 <= APPROACH_STOP_CM && dist0 >= MIN_DETECTION_CM) {
                        NetPrinter_v2.printf("LidarLog", "Obstacle Detected at 0°! Distance: " + dist0);
                        currentState = State.STRAFE_OUT;
                    }
                    break;

                case STRAFE_OUT:
                    driveTrain.holonomicDrive(CRAB_X_SPEED, 0, CRAB_ROT_CORRECTION);
                    if (dist0 < MIN_DETECTION_CM || dist0 > MAX_DETECTION_CM) {
                        timerStartTime = System.currentTimeMillis();
                        currentState = State.STRAFE_EXTRA_CLEAR;
                    }
                    break;

                case STRAFE_EXTRA_CLEAR:
                    driveTrain.holonomicDrive(CRAB_X_SPEED, 0, CRAB_ROT_CORRECTION);
                    if (System.currentTimeMillis() - timerStartTime >= FIRST_CLEAR_MS) {
                        currentState = State.DRIVE_FIND_SIDE;
                    }
                    break;

                case DRIVE_FIND_SIDE:
                    driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                    if (dist270 > 0 && dist270 <= SIDE_DETECTION_CM) {
                        NetPrinter_v2.printf("LidarLog", "Box acquired on 270° side: " + dist270);
                        currentState = State.DRIVE_PASS_SIDE;
                    }
                    break;

                case DRIVE_PASS_SIDE:
                    driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                    if (dist270 <= 0 || dist270 > SIDE_DETECTION_CM) {
                        NetPrinter_v2.printf("LidarLog", "Box passed on side. Clearing rear bumper...");
                        timerStartTime = System.currentTimeMillis();
                        currentState = State.DRIVE_CLEAR_REAR;
                    }
                    break;

                case DRIVE_CLEAR_REAR:
                    driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                    if (System.currentTimeMillis() - timerStartTime >= FORWARD_DRIVE_MS) {
                        timerStartTime = System.currentTimeMillis();
                        currentState = State.STRAFE_RETURN;
                    }
                    break;

                case STRAFE_RETURN:
                    driveTrain.holonomicDrive(RETURN_CRAB_X_SPEED, 0, RETURN_ROT_CORRECTION);
                    if (System.currentTimeMillis() - timerStartTime >= RETURN_LINE_MS) {
                        NetPrinter_v2.printf("LidarLog", "Returned to line. Ready for next obstacle!");
                        currentState = State.DRIVING_FORWARD;
                    }
                    break;
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        if (avoidanceThread != null) {
            avoidanceThread.interrupt();
        }
        driveTrain.holonomicDrive(0, 0, 0);
        NetPrinter_v2.printf("LidarLog", "THREAD STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}