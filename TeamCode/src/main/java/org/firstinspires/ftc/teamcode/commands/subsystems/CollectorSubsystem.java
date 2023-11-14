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

    public static Double LOWER_LIFT = 0.92, RAISE_LIFT = 0.09;
    private final Double TRANSFER_POS = 1.0 / 180.0, COLLECT_POS = 1.0 / 9.0;

    private final Double CLOSED_POS = 0., OPENED_POS = .55;

    private enum RotationState {
        COLLECT,
        TRANSFER
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

    public ClampState clamping;
    public LiftState location = LiftState.IDLE;

    private final InterpLUT rightLiftPositions = new InterpLUT();

    private RotationState rotation;

    public CollectorSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clamp, @Nullable ServoEx clawR) {
        liftLeft = liftL;
        liftRight = liftR;
        clawSpin = clawR;
        claw = clamp;

        rightLiftPositions.add(-1e-6, 0.05);
        rightLiftPositions.add(0.93 + 1e-6, 0.96);
        rightLiftPositions.createLUT();

        liftR.setInverted(true);
        clamping = (this.claw.getPosition() < 0.2 ? ClampState.CLOSED : ClampState.OPENED);
        rotation = (this.clawSpin.getPosition() < 0.01 ? RotationState.TRANSFER : RotationState.COLLECT);
        this.setLiftLocation(LiftState.LOWERED);
    }

    public void setLiftLocation(LiftState target) {

        if (target == location)
            return;

        if (claw.getPosition() > 0.2)
            this.toggleClamp();

        switch (target) {
            case RAISED:
                if (rotation != RotationState.TRANSFER)
                    this.rotateClaw();

                liftLeft.setPosition(RAISE_LIFT);
                liftRight.setPosition(rightLiftPositions.get(RAISE_LIFT));

                location = LiftState.RAISED;
                break;

            case IDLE:
                // TODO: Replace placeholder positions
                liftLeft.setPosition(0.5);
                liftRight.setPosition(rightLiftPositions.get(0.5));

                if (rotation != RotationState.COLLECT)
                    this.rotateClaw();

                location = LiftState.IDLE;
                break;

            case LOWERED:
                liftLeft.setPosition(LOWER_LIFT);
                liftRight.setPosition(rightLiftPositions.get(LOWER_LIFT));

                if (rotation != RotationState.COLLECT)
                    this.rotateClaw();

                location = LiftState.LOWERED;
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

    public void rotateClaw() {
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
}
