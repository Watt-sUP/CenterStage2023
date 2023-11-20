package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.Objects;

import javax.annotation.Nullable;

@Config
public class CollectorSubsystem extends SubsystemBase {
    ServoEx liftLeft, liftRight;
    ServoEx clawSpin, claw;

    public static Double LOWER_LIFT = 0.92, RAISE_LIFT = 0.09, STACK_LIFT = 0.8;
    private final Double TRANSFER_POS = 0., COLLECT_POS = 180.;

    private final Double CLOSED_POS = 0., OPENED_POS = .55;

    private enum RotationState {
        COLLECT,
        TRANSFER
    }

    public LiftState location = LiftState.STACK;

    public enum ClampState {
        CLOSED,
        OPENED
    }

    public ClampState clamping;

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

            case STACK:
                liftLeft.setPosition(STACK_LIFT);
                liftRight.setPosition(rightLiftPositions.get(STACK_LIFT));

                if (rotation != RotationState.COLLECT)
                    this.rotateClaw();

                location = LiftState.STACK;
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
        claw.setPosition(CLOSED_POS);
        Objects.requireNonNull(clawR).setInverted(true);
        clamping = ClampState.CLOSED;
        rotation = (this.clawSpin.getPosition() < 0.01 ? RotationState.TRANSFER : RotationState.COLLECT);
        this.setLiftLocation(LiftState.RAISED);
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

    public void rotateClaw() {
        if (clawSpin == null)
            return;

        switch (rotation) {
            case COLLECT:
                clawSpin.turnToAngle(TRANSFER_POS);
                rotation = RotationState.TRANSFER;
                break;
            case TRANSFER:
                clawSpin.turnToAngle(COLLECT_POS);
                rotation = RotationState.COLLECT;
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
