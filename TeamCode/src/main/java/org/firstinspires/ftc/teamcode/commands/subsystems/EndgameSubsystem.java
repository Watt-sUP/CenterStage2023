package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import javax.annotation.Nullable;

public class EndgameSubsystem extends SubsystemBase {

    private final double TICKS_PER_REV = 384.5, GEAR_RATIO = 28.0;
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
                setClimbState(ClimbState.DRONE);
                break;
            case DRONE:
                setClimbState(ClimbState.HOOKING);
                break;
            case HOOKING:
                setClimbState(ClimbState.HANGING);
                break;
            case HANGING:
                setClimbState(ClimbState.DOWN);
                break;
        }
    }

    public void setClimbState(ClimbState state) {
        climbState = state;
        setClimbToAngle(climbState.getAngle());
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
        setClimbToTicks(angleToTicks(angle, unit));
    }

    /**
     * Sets the ascension mechanism to the given angle. Defaults to degrees.
     *
     * @param angle The angle to move to
     */
    public void setClimbToAngle(double angle) {
        setClimbToAngle(angle, AngleUnit.DEGREES);
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

    public enum ClimbState {
        HOOKING(105), DRONE(60), HANGING(45), DOWN(0);

        private final double angle;

        ClimbState(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }
}
