package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import javax.annotation.Nullable;

// TODO: Merge with SlidesSubsystem.java
@Config
public class DepositSubsystem extends SubsystemBase {
    ServoEx left, right;
    public static Double LOW_LEFT = 0.04, LOW_RIGHT = 0.06;
    public static Double HIGH_LEFT = 0.56, HIGH_RIGHT = 0.55;

    public DepositSubsystem(ServoEx left, ServoEx right, @Nullable ServoEx stopper) {
        this.left = left;
        this.right = right;

        this.left.setInverted(true);
        lower();
    }

    public void raise() {
        left.setPosition(HIGH_LEFT);
        right.setPosition(HIGH_RIGHT);
    }

    public void lower() {
        left.setPosition(LOW_LEFT);
        right.setPosition(LOW_RIGHT);
    }
}
