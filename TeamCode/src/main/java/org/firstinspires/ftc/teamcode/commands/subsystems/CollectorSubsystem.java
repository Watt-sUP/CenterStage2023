package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

@Config
public class CollectorSubsystem extends SubsystemBase {
    private final ServoEx liftLeft, liftRight;
    public static Double LOWER_LIFT = 0.83, RAISE_LIFT = 0.06, STACK_LIFT = 0.76;
    private final InterpLUT rightConverter = new InterpLUT();
    private final Double CLOSED_POS = 0., OPENED_POS = .55;
    private final ServoEx claw;
    public LiftState location = LiftState.STACK;

    public enum ClampState {
        CLOSED,
        OPENED
    }

    public ClampState clamping = ClampState.CLOSED;
    private Timing.Timer clampTimer = new Timing.Timer(275, TimeUnit.MILLISECONDS);

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, ServoEx clamp) {
        liftLeft = liftL;
        liftRight = liftR;
        claw = clamp;

        rightConverter.add(-1e-6, 0.05);
        rightConverter.add(0.93 + 1e-6, 0.96);
        rightConverter.createLUT();

        liftR.setInverted(true);
        claw.setPosition(CLOSED_POS);
        this.setLiftLocation(LiftState.RAISED);
    }

    public void setLiftLocation(LiftState target) {

        if (target == location)
            return;

        if (clamping == ClampState.OPENED)
            this.toggleClamp();

        switch (target) {
            case RAISED:
                liftLeft.setPosition(RAISE_LIFT);
                liftRight.setPosition(rightConverter.get(RAISE_LIFT));

                location = LiftState.RAISED;
                break;

            case STACK:
                liftLeft.setPosition(STACK_LIFT);
                liftRight.setPosition(rightConverter.get(STACK_LIFT));

                location = LiftState.STACK;
                break;

            case LOWERED:
                liftLeft.setPosition(LOWER_LIFT);
                liftRight.setPosition(rightConverter.get(LOWER_LIFT));

                location = LiftState.LOWERED;
                break;
        }
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
                if (clamping == ClampState.CLOSED)
                    this.toggleClamp();

                this.setLiftLocation(LiftState.LOWERED);
                break;
        }
    }

    public void toggleClamp() {
        switch (clamping) {
            case OPENED:
                claw.setPosition(CLOSED_POS);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                if (location != LiftState.RAISED)
                    claw.setPosition(OPENED_POS);
                else {
                    claw.setPosition(0.22);
                    clampTimer.start();
                }
                clamping = ClampState.OPENED;
                break;
        }
    }

    @Override
    public void periodic() {
        if (clampTimer.done() && clampTimer.isTimerOn() && location != LiftState.RAISED) {
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
