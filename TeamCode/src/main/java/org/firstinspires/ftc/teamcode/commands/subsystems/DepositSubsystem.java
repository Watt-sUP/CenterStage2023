package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final ServoEx leftLift, rightLift, stopper;
    private final DcMotor slides;
    public static Double LOW_LEFT = 0.04, LOW_RIGHT = 0.06;
    public static Double HIGH_LEFT = 0.59, HIGH_RIGHT = 0.58;

    private enum Blocker {
        TWO_PIXELS,
        ONE_PIXEL,
        FREE
    }
    private enum Spike {
        RAISED,
        LOWERED
    }
    private Spike spikeState = Spike.RAISED;

    private Blocker blockerState = Blocker.FREE;
    private boolean raisingSlides = false;

    public DepositSubsystem(ServoEx leftLift, ServoEx rightLift, ServoEx stopper, DcMotor slides) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.stopper = stopper;
        this.slides = slides;

        this.leftLift.setInverted(true);
        this.stopper.turnToAngle(140);
        this.toggleSpike();

        this.slides.setDirection(DcMotorSimple.Direction.FORWARD);
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void periodic() {
        if (slides.getCurrentPosition() > 100 && raisingSlides)
        {
            while (blockerState != Blocker.TWO_PIXELS)
                this.toggleBlocker();

            if (spikeState == Spike.LOWERED)
                this.toggleSpike();

            raisingSlides = false;
        }
    }

    public Integer getSlidesTicks() {
        return slides.getCurrentPosition();
    }

    public void setSlidesTicks(int ticks) {
        raisingSlides = slides.getTargetPosition() < 100 && ticks > 100;

        slides.setTargetPosition(ticks);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }

    public void setSlidesPosition(int position) {

        position = MathUtils.clamp(position, 0, 4);
        List<Integer> positions = Arrays.asList(0, 400, 700, 1000, 1300);

        this.setSlidesTicks(positions.get(position));
    }

    public void raiseSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(new int[]{0, 400, 700, 1000, 1300}, ticks);

        this.setSlidesPosition(current_pos < 0 ? MathUtils.clamp(current_pos * -1 - 1, 0, 4) : (current_pos + 1));
    }

    public void lowerSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(new int[]{0, 400, 700, 1000, 1300}, ticks);

        this.setSlidesPosition(current_pos < 0 ? MathUtils.clamp(current_pos * -1 - 2, 0, 4) : (current_pos - 1));
    }

    public void adjustSlidesTicks(int ticks) {
        this.setSlidesTicks(this.getSlidesTicks() + ticks);
    }

    public boolean slidesBusy() {
        return slides.isBusy();
    }

    public void toggleSpike() {
        switch (spikeState) {
            case RAISED:
                leftLift.setPosition(LOW_LEFT);
                rightLift.setPosition(LOW_RIGHT);
                spikeState = Spike.LOWERED;
                break;
            case LOWERED:
                leftLift.setPosition(HIGH_LEFT);
                rightLift.setPosition(HIGH_RIGHT);
                spikeState = Spike.RAISED;
                break;
        }
    }

    public void toggleBlocker() {
        switch (blockerState) {
            case FREE:
                stopper.turnToAngle(165);
                blockerState = Blocker.TWO_PIXELS;
                break;
            case ONE_PIXEL:
                stopper.turnToAngle(140);
                blockerState = Blocker.FREE;
                break;
            case TWO_PIXELS:
                stopper.turnToAngle(155);
                blockerState = Blocker.ONE_PIXEL;
                break;
        }
    }
}
