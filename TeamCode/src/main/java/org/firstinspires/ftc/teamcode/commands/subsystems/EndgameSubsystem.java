package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import javax.annotation.Nullable;

@Config
public class EndgameSubsystem extends SubsystemBase {

    public static double HOOK_ANGLE = 105, DRONE_ANGLE = 60, HANGING_ANGLE = 45;
    private final double TICKS_PER_REV = 537.7, GEAR_RATIO = 28.0;
    private final DcMotor leftPull, rightPull;
    private final ServoEx launcher;

    private ClimbState climbState = ClimbState.DOWN;

    public EndgameSubsystem(DcMotor leftPull, DcMotor rightPull, @Nullable ServoEx launcher) {
        this.leftPull = leftPull;
        this.rightPull = rightPull;
        this.launcher = launcher;

        this.leftPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightPull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (launcher != null) {
            this.launcher.setInverted(true);
            this.launcher.turnToAngle(50);
        }
    }

    private int angleToTicks(double angle, AngleUnit unit) {
        return (int) (angle * TICKS_PER_REV * GEAR_RATIO / (unit.equals(AngleUnit.DEGREES) ? 360.0 : 2 * Math.PI));
    }

    private double ticksToAngle(int ticks, AngleUnit unit) {
        return ticks / (TICKS_PER_REV * GEAR_RATIO) * (unit.equals(AngleUnit.DEGREES) ? 360.0 : 2 * Math.PI);
    }

    public void toggleClimb() {
        switch (climbState) {
            case DOWN:
                this.setClimbToAngle(DRONE_ANGLE);
                climbState = ClimbState.DRONE;
                break;
            case DRONE:
                this.setClimbToAngle(HOOK_ANGLE);
                climbState = ClimbState.HOOKING;
                break;
            case HOOKING:
                this.setClimbToAngle(HANGING_ANGLE);
                climbState = ClimbState.HANGING;
                break;
            case HANGING:
                this.setClimbToAngle(0);
                climbState = ClimbState.DOWN;
                break;
        }
    }

    private void setClimbToTicks(int ticks) {
        leftPull.setTargetPosition(ticks);
        leftPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPull.setPower(1);

        rightPull.setTargetPosition(ticks);
        rightPull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPull.setPower(1);
    }

    /**
     * Sets the ascension mechanism to the given angle.
     *
     * @param angle The angle to move to
     * @param unit  The unit the angle is in
     */
    public void setClimbToAngle(double angle, AngleUnit unit) {
        this.setClimbToTicks(this.angleToTicks(angle, unit));
    }

    /**
     * Sets the ascension mechanism to the given angle. Defaults to degrees.
     *
     * @param angle The angle to move to
     */
    public void setClimbToAngle(double angle) {
        this.setClimbToAngle(angle, AngleUnit.DEGREES);
    }

    /**
     * Toggles the servomotor of the drone launcher. Only works if in position.
     */
    public void launchPlane() {
        if (launcher == null || climbState != ClimbState.DRONE)
            return;

        if (Math.round(launcher.getAngle()) == 50)
            launcher.turnToAngle(20);
        else launcher.turnToAngle(50);
    }

    public boolean isBusy() {
        return leftPull.isBusy() || rightPull.isBusy();
    }

    public String getClimbState() {
        return climbState.toString();
    }

    /**
     * Gets the current angle of the ascension system.
     *
     * @param unit The unit the output should be in
     * @return The angle of the system in the desired unit
     */
    public double getClimbAngle(AngleUnit unit) {
        int avg = (leftPull.getCurrentPosition() + rightPull.getCurrentPosition()) / 2;
        return ticksToAngle(avg, unit);
    }

    private enum ClimbState {
        HOOKING,
        DRONE,
        HANGING,
        DOWN
    }
}
