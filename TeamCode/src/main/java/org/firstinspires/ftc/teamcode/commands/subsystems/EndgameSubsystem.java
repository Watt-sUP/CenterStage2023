package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

@Config
public class EndgameSubsystem extends SubsystemBase {

    public static int HOOKING_POS = 4000, DRONE_POS = 2100;
    private final DcMotor leftPull, rightPull;
    private final ServoEx launcher;

    private ClimbState climbState = ClimbState.LOWERED;

    public EndgameSubsystem(DcMotor leftPull, DcMotor rightPull, @Nullable ServoEx launcher) {
        this.leftPull = leftPull;
        this.rightPull = rightPull;
        this.launcher = launcher;

        this.leftPull.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.rightPull.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (launcher != null) {
            this.launcher.setInverted(true);
            this.launcher.turnToAngle(35);
        }
    }

    public void toggleClimb() {
        switch (climbState) {
            case LOWERED:
                this.setClimbToTicks(DRONE_POS);
                climbState = ClimbState.DRONE;
                break;
            case DRONE:
                this.setClimbToTicks(HOOKING_POS);
                climbState = ClimbState.HOOKING;
                break;
            case HOOKING:
                this.setClimbToTicks(0);
                climbState = ClimbState.LOWERED;
                break;
        }
    }

    public void setClimbToTicks(int ticks) {
        leftPull.setTargetPosition(ticks);
        leftPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPull.setPower(1);

        rightPull.setTargetPosition(ticks);
        rightPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPull.setPower(1);
    }

    public void launchPlane() {
        if (launcher == null || climbState != ClimbState.DRONE)
            return;

        if (Math.round(launcher.getAngle()) == 35)
            launcher.turnToAngle(120);
        else launcher.turnToAngle(35);
    }

    public Integer getClimbTicks() {
        return rightPull.getCurrentPosition();
    }

    public boolean isBusy() {
        return leftPull.isBusy() || rightPull.isBusy();
    }

    private enum ClimbState {
        HOOKING,
        DRONE,
        LOWERED
    }
}
