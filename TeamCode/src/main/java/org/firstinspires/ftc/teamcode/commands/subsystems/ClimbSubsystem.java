package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

public class ClimbSubsystem extends SubsystemBase {

    DcMotor left, right;

    private State state = State.LOWERED;

    public void toggle() {
        switch (state) {
            case LOWERED:
                setToTicks(3750);
                state = State.RAISED;
                break;
            case RAISED:
                setToTicks(0);
                state = State.LOWERED;
                break;
        }
    }

    public ClimbSubsystem(DcMotor left, @Nullable DcMotor right) {
        this.left = left;
        this.right = right;

        this.left.setDirection(DcMotorSimple.Direction.REVERSE);
        this.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (right != null) {
            this.right.setDirection(DcMotorSimple.Direction.FORWARD);
            this.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setToTicks(int ticks) {
        left.setTargetPosition(ticks);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(1);

        if (right != null) {
            right.setTargetPosition(ticks);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(1);
        }
    }

    private enum State {
        RAISED,
        LOWERED
    }

    public Integer getTicks() {
        return left.getCurrentPosition();
    }

    public boolean isBusy() {
        return (right != null ? left.isBusy() || right.isBusy() : left.isBusy());
    }
}
