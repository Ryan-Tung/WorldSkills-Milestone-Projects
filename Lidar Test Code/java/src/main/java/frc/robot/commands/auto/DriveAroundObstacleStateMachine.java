package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NetPrinter_v2;

public class DriveAroundObstacleStateMachine extends AutoCommand {

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

    private static final double FIRST_CLEAR_SEC = 2.5;   // Chassis width clearance time
    private static final double FORWARD_DRIVE_SEC = 1.5; // Rear bumper clearance time
    private static final double RETURN_LINE_SEC = 3.0;   // Centerline return time
    // =========================================================================

    private final DriveTrain driveTrain;
    private final Timer stateTimer = new Timer();

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

    public DriveAroundObstacleStateMachine(DriveTrain driveTrain) {
        super(new InstantCommand());
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        currentState = State.DRIVING_FORWARD;
        stateTimer.reset();
        stateTimer.start();
        NetPrinter_v2.printf("LidarLog", "STATE MACHINE STARTED: Repeatable Obstacle Avoidance");
    }

    @Override
    public void execute() {
        double dist0 = driveTrain.getLidarAtZeroDegrees();
        double dist270 = driveTrain.getLidarAt270Degrees();

        switch (currentState) {

            case DRIVING_FORWARD:
                driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                if (dist0 <= APPROACH_STOP_CM && dist0 >= MIN_DETECTION_CM) {
                    NetPrinter_v2.printf("LidarLog", "Obstacle Ahead! Distance: " + dist0);
                    currentState = State.STRAFE_OUT;
                }
                break;

            case STRAFE_OUT:
                driveTrain.holonomicDrive(CRAB_X_SPEED, 0, CRAB_ROT_CORRECTION);
                if (dist0 < MIN_DETECTION_CM || dist0 > MAX_DETECTION_CM) {
                    stateTimer.reset();
                    currentState = State.STRAFE_EXTRA_CLEAR;
                }
                break;

            case STRAFE_EXTRA_CLEAR:
                driveTrain.holonomicDrive(CRAB_X_SPEED, 0, CRAB_ROT_CORRECTION);
                if (stateTimer.hasPeriodPassed(FIRST_CLEAR_SEC)) {
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
                    stateTimer.reset();
                    currentState = State.DRIVE_CLEAR_REAR;
                }
                break;

            case DRIVE_CLEAR_REAR:
                driveTrain.holonomicDrive(0, FORWARD_SPEED, 0);
                if (stateTimer.hasPeriodPassed(FORWARD_DRIVE_SEC)) {
                    stateTimer.reset();
                    currentState = State.STRAFE_RETURN;
                }
                break;

            case STRAFE_RETURN:
                driveTrain.holonomicDrive(RETURN_CRAB_X_SPEED, 0, RETURN_ROT_CORRECTION);
                if (stateTimer.hasPeriodPassed(RETURN_LINE_SEC)) {
                    NetPrinter_v2.printf("LidarLog", "Returned to line. Resetting for next obstacle!");
                    
                    // REPEATABILITY RESET: Automatically loops back to looking for obstacles
                    currentState = State.DRIVING_FORWARD;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.holonomicDrive(0, 0, 0);
        stateTimer.stop();
        NetPrinter_v2.printf("LidarLog", "COMMAND STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous loop
    }
}