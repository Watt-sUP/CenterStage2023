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

    // TODO: Raise the lift exclusively if the claw is closed
    public static Double LOWER_LIFT = 0.91, UP_LIFT = 0.12;
    private final Double TRANSFER_POS = 0.01, COLLECT_POS = 1.0 / 9.0;

    // TODO: When the lift is raised, make the opening position of the claw 0.2
    public static Double CLOSED_POS = 0., OPENED_POS = .55;

    private enum RotationState {
        COLLECT,
        TRANSFER
    }

    private enum ClampState {
        CLOSED,
        OPENED
    }

    private final InterpLUT rightLiftPositions = new InterpLUT();

    private RotationState rotation = RotationState.COLLECT;
    private ClampState clamping = ClampState.OPENED;

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clamp, @Nullable ServoEx clawR) {
        liftLeft = liftL;
        liftRight = liftR;
        clawSpin = clawR;
        claw = clamp;

        rightLiftPositions.add(-1e-6, 0.05);
        rightLiftPositions.add(0.91 + 1e-6, 0.94);
        rightLiftPositions.createLUT();

        liftR.setInverted(true);
        lowerLift();

        if (clawSpin != null)
            clawSpin.setPosition(COLLECT_POS);

        if (claw != null)
            claw.setPosition(OPENED_POS);
    }

    public void lowerLift() {
        liftLeft.setPosition(LOWER_LIFT);
        liftRight.setPosition(rightLiftPositions.get(LOWER_LIFT));
    }

    public void raiseLift() {
        liftLeft.setPosition(UP_LIFT);
        liftRight.setPosition(rightLiftPositions.get(UP_LIFT));
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

    public void toggle() {
        if (claw == null)
            return;

        switch (clamping) {
            case OPENED:
                claw.setPosition(CLOSED_POS);
                clamping = ClampState.CLOSED;
                break;
            case CLOSED:
                claw.setPosition(OPENED_POS);
                clamping = ClampState.OPENED;
                break;
        }
    }
}
