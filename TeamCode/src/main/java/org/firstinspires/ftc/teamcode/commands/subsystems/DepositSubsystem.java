package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import javax.annotation.Nullable;

public class DepositSubsystem extends SubsystemBase {
    ServoEx left, right;
    // Right: down 0.02, up 0.55
    // Left: down 0, up 0.56

    public DepositSubsystem(ServoEx left, ServoEx right, @Nullable ServoEx stopper) {
        this.left = left;
        this.right = right;

        this.left.setInverted(true);
        lower();
    }

    public void raise() {
        left.setPosition(0.56);
        right.setPosition(0.55);
    }

    public void lower() {
        left.setPosition(0);
        right.setPosition(0.02);
    }
}
