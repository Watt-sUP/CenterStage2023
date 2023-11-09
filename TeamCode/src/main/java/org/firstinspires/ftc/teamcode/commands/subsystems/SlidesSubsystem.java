package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

// TODO: Merge with DepositSubsystem.java
public class SlidesSubsystem extends SubsystemBase {

    DcMotor jos, sus;
    public SlidesSubsystem(@Nullable DcMotor jos, DcMotor sus) {
        this.jos = jos;
        this.sus = sus;

        if (jos != null) {
            this.jos.setDirection(DcMotorSimple.Direction.REVERSE);
            this.jos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        this.sus.setDirection(DcMotorSimple.Direction.FORWARD);
        this.sus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setToTicks(int ticks) {
        if (jos != null) {
            jos.setTargetPosition(ticks);
            jos.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            jos.setPower(1);
        }

        sus.setTargetPosition(ticks);
        sus.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sus.setPower(1);
    }

    public int getTicks() {
        return sus.getCurrentPosition();
    }

    public boolean isBusy() {
        return (jos != null ? jos.isBusy() || sus.isBusy() : sus.isBusy());
    }
}