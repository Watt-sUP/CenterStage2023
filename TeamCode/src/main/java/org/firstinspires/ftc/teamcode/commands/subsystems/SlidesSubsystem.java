package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SlidesSubsystem extends SubsystemBase {

    DcMotor jos, sus;
    public SlidesSubsystem(DcMotor jos, DcMotor sus) {
        this.jos = jos;
        this.sus = sus;

        this.jos.setDirection(DcMotorSimple.Direction.REVERSE);
        this.sus.setDirection(DcMotorSimple.Direction.FORWARD);

        this.jos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.sus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setToTicks(int ticks) {
        jos.setTargetPosition(ticks);
        jos.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jos.setPower(1);

        sus.setTargetPosition(ticks);
        sus.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sus.setPower(1);
    }

    public int getTicks() {
        return jos.getCurrentPosition();
    }

    public boolean isBusy() {
        return jos.isBusy() && sus.isBusy();
    }
}