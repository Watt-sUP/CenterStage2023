package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import javax.annotation.Nullable;

@Config
public class TransferSubsystem extends SubsystemBase {
    ServoEx liftLeft, liftRight, clawSpin;
    public static Double LOWER_LEFT = 0.91, LOWER_RIGHT = 0.94;
    public static Double UP_LEFT = 0.0, UP_RIGHT = 0.05;

    public TransferSubsystem(ServoEx liftL, ServoEx liftR, @Nullable ServoEx clawR) {
        liftLeft = liftL;
        liftRight = liftR;
        clawSpin = clawR;

        liftR.setInverted(true);
        lowerLift();
    }

    public void lowerLift() {
        liftLeft.setPosition(LOWER_LEFT);
        liftRight.setPosition(LOWER_RIGHT);
    }

    public void raiseLift() {
        liftLeft.setPosition(UP_LEFT);
        liftRight.setPosition(UP_RIGHT);
    }
}
