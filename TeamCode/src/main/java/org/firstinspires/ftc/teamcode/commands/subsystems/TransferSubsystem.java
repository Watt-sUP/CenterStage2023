package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import javax.annotation.Nullable;

public class TransferSubsystem extends SubsystemBase {
    ServoEx liftLeft, liftRight, clawSpin;
    public Double LOWER_POS = 0.75;

    public TransferSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clawR) {
        liftLeft = liftL;
        liftRight = liftR;
        clawSpin = clawR;

        liftR.setInverted(true);
    }

    public void lowerLift() {
        liftLeft.setPosition(LOWER_POS);
        liftRight.setPosition(LOWER_POS);
    }

    public void raiseLift() {
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
    }
}
