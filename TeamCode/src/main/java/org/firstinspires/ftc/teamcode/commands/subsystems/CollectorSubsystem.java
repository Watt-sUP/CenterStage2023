package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

@Config
public class CollectorSubsystem extends SubsystemBase {
    public static Double LOWER_LIFT = 0.81, RAISE_LIFT = 0.04, STACK_LIFT = 0.75;
    private final ServoEx liftLeft, liftRight;
    private final ServoEx claw;
    private final InterpLUT rightConverter = new InterpLUT();
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

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, ServoEx clamp) {
        liftLeft = liftL;
        liftRight = liftR;
        claw = clamp;

        // Servos have slight factory mismatch, create position table for synchronization
        rightConverter.add(-1e-6, 0.05);
        rightConverter.add(0.93 + 1e-6, 0.96);
        rightConverter.createLUT();

        liftR.setInverted(true);
        claw.setPosition(0);
        this.setLiftLocation(LiftState.RAISED);
    }

    public void setLiftLocation(LiftState target) {

        if (target == location)
            return;

        // Claw needs to be closed to avoid hitting the belts
        if (clamping == ClampState.OPENED && (target == LiftState.RAISED || location == LiftState.RAISED))
            this.toggleClamp();

        switch (target) {
            case RAISED:
                liftLeft.setPosition(RAISE_LIFT);
                liftRight.setPosition(rightConverter.get(RAISE_LIFT));
                break;

            case STACK:
                liftLeft.setPosition(STACK_LIFT);
                liftRight.setPosition(rightConverter.get(STACK_LIFT));
                break;

            case LOWERED:
                liftLeft.setPosition(LOWER_LIFT);
                liftRight.setPosition(rightConverter.get(LOWER_LIFT));
                break;
        }
        location = target;
    }

    public void toggleLiftLocation() {
        switch (location) {
            case LOWERED:
                this.setLiftLocation(LiftState.RAISED);
                break;
            case RAISED:
                this.setLiftLocation(LiftState.STACK);
                break;
            case STACK:
                // Open the claw when it's fully lowered
                if (clamping == ClampState.CLOSED)
                    this.toggleClamp();

                this.setLiftLocation(LiftState.LOWERED);
                break;
        }
    }

    public void toggleClamp() {
        switch (clamping) {
            case OPENED:
                // The code of the timer resets it every time .start() is called (equivalent to a .reset())
                if (location != LiftState.RAISED) // Can't collect when raised
                    clampTimer.start();
                claw.setPosition(0);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                // Don't open the claw fully when the lift is raised to avoid the belts
                claw.setPosition(location != LiftState.RAISED ? .55 : .22);
                clamping = ClampState.OPENED;
                break;
        }
    }

    // Periodic check: Automatically raise the claw 275ms after collection
    @Override
    public void periodic() {
        // Timer keeps going even after it's done, so pause it to avoid repetition
        if (clampTimer.done() && clampTimer.isTimerOn()) {
            this.setLiftLocation(LiftState.RAISED);
            clampTimer.pause();
        }
    }

    public enum LiftState {
        LOWERED,
        RAISED,
        STACK
    }
}
