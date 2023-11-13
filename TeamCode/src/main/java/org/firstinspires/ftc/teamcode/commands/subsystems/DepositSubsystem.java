package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final ServoEx leftLift, rightLift, stopper;
    private final DcMotor slides;
    public static Double LOW_LEFT = 0.04, LOW_RIGHT = 0.06;
    public static Double HIGH_LEFT = 0.58, HIGH_RIGHT = 0.57;

    private enum Blocker {
        TWO_PIXELS,
        ONE_PIXEL,
        FREE
    }
    private Blocker blockerState = Blocker.FREE;

    public DepositSubsystem(ServoEx leftLift, ServoEx rightLift, ServoEx stopper, DcMotor slides) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.stopper = stopper;
        this.slides = slides;

        this.leftLift.setInverted(true);
        this.stopper.turnToAngle(0);
        lowerSpike();

        this.slides.setDirection(DcMotorSimple.Direction.FORWARD);
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Integer getSlidesTicks() {
        return slides.getCurrentPosition();
    }

    public void setSlidesTicks(int ticks) {
        slides.setTargetPosition(ticks);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }

    public boolean slidesBusy() {
        return slides.isBusy();
    }

    public void raiseSpike() {
        leftLift.setPosition(HIGH_LEFT);
        rightLift.setPosition(HIGH_RIGHT);
    }

    public void toggleBlocker() {
        switch (blockerState) {
            case FREE:
                stopper.turnToAngle(50);
                blockerState = Blocker.TWO_PIXELS;
                break;
            case ONE_PIXEL:
                stopper.turnToAngle(0);
                blockerState = Blocker.FREE;
                break;
            case TWO_PIXELS:
                stopper.turnToAngle(30);
                blockerState = Blocker.ONE_PIXEL;
                break;
        }
    }

    public void lowerSpike() {
        leftLift.setPosition(LOW_LEFT);
        rightLift.setPosition(LOW_RIGHT);
    }
}
