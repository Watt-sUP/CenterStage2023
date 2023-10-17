package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class CollectorSubsystem extends SubsystemBase {
    ServoEx claw;

    public CollectorSubsystem(ServoEx claw) {
        this.claw = claw;

    }
}
