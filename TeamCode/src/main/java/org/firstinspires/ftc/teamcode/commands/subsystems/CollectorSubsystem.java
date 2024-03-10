package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedServo;

import java.util.concurrent.TimeUnit;

@Config
public class CollectorSubsystem extends SubsystemBase {
    public static Double LOWER_LIFT = 180.0, RAISE_LIFT = 21.0, STACK_LIFT = 161.0;
    private final ServoEx claw;
    private final InterpolatedServo left, right;

    /*
     Timer responsible for periodic check:
     - Starts/resets once the claw is closed (something is collected)
     - Runs until it's done
     - Claw gets raised
     - Timer is paused until the cycle repeats
    */
    private final Timing.Timer clampTimer = new Timing.Timer(275, TimeUnit.MILLISECONDS);

    public enum ClampState {
        CLOSED,
        OPENED
    }

    public ClampState clamping = ClampState.CLOSED;
    public LiftState location = LiftState.STACK;

    public CollectorSubsystem(final HardwareMap hardwareMap) {
        this(
                new SimpleServo(hardwareMap, "v4b_left", 0, 220),
                new SimpleServo(hardwareMap, "v4b_right", 0, 220),
                new SimpleServo(hardwareMap, "claw", 0, 220)
        );
    }

    private CollectorSubsystem(ServoEx liftL, ServoEx liftR, ServoEx clamp) {
        left = new InterpolatedServo(liftL);
        right = new InterpolatedServo(liftR);

        left.setInverted(false);
        right.setInverted(true);

        left.generatePositions(new Pair<>(0.0, 15.0), new Pair<>(90.0, 112.5), new Pair<>(180.0, 210.0));
        right.generatePositions(new Pair<>(0.0, 6.5), new Pair<>(90.0, 101.0), new Pair<>(180.0, 199.0));

        claw = clamp;
        claw.setInverted(true);
        claw.turnToAngle(90);
        setLiftLocation(LiftState.RAISED);
    }

    public void setLiftLocation(LiftState target) {
        if (target == location)
            return;

        // Claw needs to be closed to avoid hitting the belts
        if (clamping == ClampState.OPENED && (target == LiftState.RAISED || location == LiftState.RAISED))
            toggleClamp();

        switch (target) {
            case RAISED:
                left.setToPosition(RAISE_LIFT);
                right.setToPosition(RAISE_LIFT);
                break;

            case STACK:
                left.setToPosition(STACK_LIFT);
                right.setToPosition(STACK_LIFT);
                break;

            case LOWERED:
                left.setToPosition(LOWER_LIFT);
                right.setToPosition(LOWER_LIFT);
                break;
        }
        location = target;
    }

    public void toggleLiftLocation() {
        switch (location) {
            case LOWERED:
                setLiftLocation(LiftState.RAISED);
                break;
            case RAISED:
                setLiftLocation(LiftState.STACK);
                break;
            case STACK:
                // Open the claw when it's fully lowered
                if (clamping == ClampState.CLOSED)
                    toggleClamp();

                setLiftLocation(LiftState.LOWERED);
                break;
        }
    }

    public void adjustLiftPosition(Double adjustment) {
        double new_pos = left.getCurrentPosition() + adjustment;

        left.setToPosition(new_pos);
        right.setToPosition(new_pos);
    }

    public void toggleClamp() {
        switch (clamping) {
            case OPENED:
                // The code of the timer resets it every time .start() is called (equivalent to a .reset())
                if (location != LiftState.RAISED) // Can't collect when raised
                    clampTimer.start();

                claw.turnToAngle(90);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                // Don't open the claw fully when the lift is raised to avoid the belts
                claw.turnToAngle(location != LiftState.RAISED ? 50 : 70);
                clamping = ClampState.OPENED;
                break;
        }
    }

    public void setClampPosition(double angle) {
        claw.turnToAngle(angle);
        clamping = (angle <= 70) ? ClampState.OPENED : ClampState.CLOSED;
    }

    // Periodic check: Automatically raise the claw 275ms after collection
    @Override
    public void periodic() {
        // Timer keeps going even after it's done, so pause it to avoid repetition
        if (clampTimer.done() && clampTimer.isTimerOn()) {
            setLiftLocation(LiftState.RAISED);
            clampTimer.pause();
        }
    }

    public enum LiftState {
        LOWERED,
        RAISED,
        STACK
    }
}
