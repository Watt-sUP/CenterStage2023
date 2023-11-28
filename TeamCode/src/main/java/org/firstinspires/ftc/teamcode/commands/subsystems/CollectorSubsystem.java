package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;

import javax.annotation.Nullable;

@Config
public class CollectorSubsystem extends SubsystemBase {
    ServoEx liftLeft, liftRight;
    public static Double LOWER_LIFT = 0.87, RAISE_LIFT = 0.12, STACK_LIFT = 0.8;
    private final InterpLUT rightConverter = new InterpLUT();
    private final Double CLOSED_POS = 0., OPENED_POS = .55;

    public LiftState location = LiftState.STACK;

    public enum ClampState {
        CLOSED,
        OPENED
    }

    public ClampState clamping;
    ServoEx claw;

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clamp) {
        liftLeft = liftL;
        liftRight = liftR;
        claw = clamp;

        rightConverter.add(-1e-6, 0.05);
        rightConverter.add(0.93 + 1e-6, 0.96);
        rightConverter.createLUT();

        liftR.setInverted(true);
        claw.setPosition(CLOSED_POS);
        clamping = ClampState.CLOSED;
        this.setLiftLocation(LiftState.RAISED);
    }

    public void setLiftLocation(LiftState target) {

        if (target == location)
            return;

        if (claw.getPosition() > 0.2)
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
                this.setLiftLocation(LiftState.LOWERED);
                if (claw.getPosition() < .5) {
                    if (clamping == ClampState.CLOSED)
                        this.toggleClamp();
                    else claw.setPosition(OPENED_POS);
                }
                break;
        }
    }

    public void toggleClamp() {
        if (claw == null)
            return;

        switch (clamping) {
            case OPENED:
                claw.setPosition(CLOSED_POS);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                claw.setPosition(location != LiftState.RAISED ? OPENED_POS : 0.2);
                clamping = ClampState.OPENED;
                break;
        }
    }

    public enum LiftState {
        LOWERED,
        RAISED,
        STACK
    }
}
