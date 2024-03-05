package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.IndependentMotorGroup;

import java.util.Locale;
import java.util.stream.Collectors;

@Config
public class EndgameSubsystem extends SubsystemBase {

    public static double kP = 0.5;
    private final IndependentMotorGroup elevator;
    private final ServoEx launcher;

    private ElevatorState elevatorState = ElevatorState.DOWN;

    public EndgameSubsystem(final HardwareMap hardwareMap) {
        this(
                new Motor(hardwareMap, "pullup_left", 384.5, 435),
                new Motor(hardwareMap, "pullup_right", 384.5, 435),
                new SimpleServo(hardwareMap, "drone", -900, 900)
        );
    }

    private EndgameSubsystem(Motor leftArm, Motor rightArm, ServoEx launcher) {
        elevator = new IndependentMotorGroup(leftArm, rightArm);
        this.launcher = launcher;
        elevator.setInverted(false);

        elevator.stopAndResetEncoder();
        elevator.setDistancePerPulse(getDegreesPerTick());
        elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        elevator.setPositionTolerance(1.0);
        elevator.setPositionCoefficient(kP);
        elevator.setRunMode(Motor.RunMode.PositionControl);

        this.launcher.setInverted(true);
        this.launcher.turnToAngle(50);
    }

    @Override
    public void periodic() {
        if (elevator.atTargetPosition()) {
            elevator.stopMotors();
            return;
        }

        elevator.set(1.0);
    }

    private double getDegreesPerTick() {
        assert Double.isFinite(elevator.getCPR()) : "The CPR of the elevator is undefined";

        final double GEAR_RATIO = 28.0;
        return 360.0 / (elevator.getCPR() * GEAR_RATIO);
    }

    public void toggleElevator() {
        switch (elevatorState) {
            case DOWN:
                setElevatorState(ElevatorState.DRONE);
                break;
            case DRONE:
                setElevatorState(ElevatorState.HOOKING);
                break;
            case HOOKING:
                setElevatorState(ElevatorState.HANGING);
                break;
            case HANGING:
                setElevatorState(ElevatorState.DOWN);
                break;
        }
    }

    /**
     * Toggles the servomotor of the drone launcher. Only works if in position.
     */
    public void launchPlane() {
        if (elevatorState != ElevatorState.DRONE)
            return;

        if (Math.round(launcher.getAngle()) == 50)
            launcher.turnToAngle(20);
        else launcher.turnToAngle(50);
    }

    public String getElevatorState() {
        return elevatorState.toString();
    }

    public void setElevatorState(ElevatorState state) {
        elevatorState = state;
        setElevatorAngle(elevatorState.getAngle());
    }

    /**
     * Gets the current angle of the elevator.
     *
     * @return The angle of the system
     */
    public String getElevatorAngle() {
        return elevator.getDistances().stream()
                .map(distance -> String.format(Locale.ROOT, "%.2f", distance))
                .collect(Collectors.joining(", "));
    }

    /**
     * Sets the elevator mechanism to the given angle.
     *
     * @param angle The angle to move to
     */
    public void setElevatorAngle(double angle) {
        elevator.setTargetDistance(angle);
    }

    public enum ElevatorState {
        HOOKING(105), DRONE(60), HANGING(45), DOWN(0);

        private final double angle;

        ElevatorState(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }
}
