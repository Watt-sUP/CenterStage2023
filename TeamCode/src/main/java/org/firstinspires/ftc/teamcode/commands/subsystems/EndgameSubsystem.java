package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

public class EndgameSubsystem extends SubsystemBase {

    DcMotor leftPull, rightPull;
    ServoEx launcher;

    private ClimbState climbState = ClimbState.LOWERED;

    public EndgameSubsystem(DcMotor leftPull, @Nullable DcMotor rightPull, @Nullable ServoEx launcher) {
        this.leftPull = leftPull;
        this.rightPull = rightPull;
        this.launcher = launcher;

        this.leftPull.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (rightPull != null) {
            this.rightPull.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rightPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (launcher != null)
            this.launcher.turnToAngle(0);
    }

    public void toggleClimb() {
        switch (climbState) {
            case LOWERED:
                this.setClimbToTicks(3750);
                climbState = ClimbState.HOOKING;
                break;
            case HOOKING:
                this.setClimbToTicks(1500);
                climbState = ClimbState.LIFT;
                break;
            case LIFT:
                this.setClimbToTicks(0);
                climbState = ClimbState.LOWERED;
                break;
        }
    }

    public void setClimbToTicks(int ticks) {
        leftPull.setTargetPosition(ticks);
        leftPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPull.setPower(1);

        if (rightPull != null) {
            rightPull.setTargetPosition(ticks);
            rightPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightPull.setPower(1);
        }
    }

    public void launchPlane() {
        if (launcher == null || climbState == ClimbState.LOWERED)
            return;

        if ((int) launcher.getAngle() == 0)
            launcher.turnToAngle(55);
        else launcher.turnToAngle(0);
    }

    public Integer getClimbTicks() {
        return rightPull.getCurrentPosition();
    }

    public boolean isBusy() {
        return rightPull != null ? leftPull.isBusy() || rightPull.isBusy() : leftPull.isBusy();
    }

    private enum ClimbState {
        HOOKING,
        LIFT,
        LOWERED
    }
}
