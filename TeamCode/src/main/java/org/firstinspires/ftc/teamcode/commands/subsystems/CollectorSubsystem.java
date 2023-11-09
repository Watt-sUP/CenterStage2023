package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;

import javax.annotation.Nullable;

@Config
public class CollectorSubsystem extends SubsystemBase {
    ServoEx liftLeft, liftRight;
    ServoEx clawSpin, claw;

    // TODO: Raise the lift exclusively if the claw is closed (Done, must be tested)
    public static Double LOWER_LIFT = 0.91, RAISE_LIFT = 0.12;
    private final Double TRANSFER_POS = 0.01, COLLECT_POS = 1.0 / 9.0;

    public static Double CLOSED_POS = 0., OPENED_POS = .55;

    private enum RotationState {
        COLLECT,
        TRANSFER
    }

    public ClampState clamping;
    private LiftState location = LiftState.LOWERED;

    private final InterpLUT rightLiftPositions = new InterpLUT();

    private RotationState rotation = RotationState.COLLECT;

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clamp, @Nullable ServoEx clawR) {
        liftLeft = liftL;
        liftRight = liftR;
        clawSpin = clawR;
        claw = clamp;

        rightLiftPositions.add(-1e-6, 0.05);
        rightLiftPositions.add(0.91 + 1e-6, 0.94);
        rightLiftPositions.createLUT();

        liftR.setInverted(true);
        clamping = (this.claw.getPosition() < 0.2 ? ClampState.CLOSED : ClampState.OPENED);
        setLiftLocation(LiftState.IDLE);
    }

    public void lowerLift() {
        liftLeft.setPosition(LOWER_LIFT);
        liftRight.setPosition(rightLiftPositions.get(LOWER_LIFT));
        location = LiftState.LOWERED;
    }

    public void raiseLift() {
        liftLeft.setPosition(RAISE_LIFT);
        liftRight.setPosition(rightLiftPositions.get(RAISE_LIFT));
        location = LiftState.RAISED;
    }

    public void setLiftLocation(LiftState target) {

        if (target == location)
            return;

        if (claw.getPosition() > 0.2)
            toggle();

        switch (target) {
            case RAISED:
                if (rotation != RotationState.TRANSFER)
                    rotate();

                liftLeft.setPosition(RAISE_LIFT);
                liftRight.setPosition(rightLiftPositions.get(RAISE_LIFT));
                location = LiftState.RAISED;
                break;

            case IDLE:
                // TODO: Replace placeholder positions
                liftLeft.setPosition(0.5);
                liftRight.setPosition(rightLiftPositions.get(0.5));

                if (rotation != RotationState.COLLECT)
                    rotate();

                location = LiftState.IDLE;
                break;

            case LOWERED:
                liftLeft.setPosition(LOWER_LIFT);
                liftRight.setPosition(rightLiftPositions.get(LOWER_LIFT));

                if (rotation != RotationState.COLLECT)
                    rotate();

                location = LiftState.LOWERED;
                break;
        }
    }

    public void toggle() {
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

    public void rotate() {
        if (clawSpin == null)
            return;

        switch (rotation) {
            case COLLECT:
                clawSpin.setPosition(TRANSFER_POS);
                rotation = RotationState.TRANSFER;
                break;
            case TRANSFER:
                clawSpin.setPosition(COLLECT_POS);
                rotation = RotationState.COLLECT;
                break;
        }
    }

    public enum LiftState {
        LOWERED,
        RAISED,
        IDLE
    }

    public enum ClampState {
        CLOSED,
        OPENED
    }
}
