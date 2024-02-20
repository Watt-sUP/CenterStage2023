package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final DcMotor slides;
    public static Double LOW_LEFT = 0.05, LOW_RIGHT = 0.1;
    private final int[] slidesPositions = {0, 400, 700, 1000, 1250};
    private final ServoEx leftLift, rightLift;
    public static Double HIGH_LEFT = .825, HIGH_RIGHT = .875;
    private final ServoEx stopperTop, stopperBottom;
    private BooleanSupplier safeToMove = () -> true;

    private enum Blocker {
        TWO_PIXELS,
        ONE_PIXEL,
        FREE
    }

    public Spike spikeState = Spike.RAISED;

    public void setSpikePosition(double position) {
        leftLift.setPosition(MathUtils.clamp(position - 0.05, 0, 1));
        rightLift.setPosition(MathUtils.clamp(position, 0, 1));
    }

    private Blocker blockerState = Blocker.FREE;
    private boolean raisingSlides = false;

    public DepositSubsystem(ServoEx leftLift, ServoEx rightLift, ServoEx stopperTop, ServoEx stopperBottom, DcMotor slides) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.stopperTop = stopperTop;
        this.stopperBottom = stopperBottom;
        this.slides = slides;

        this.stopperBottom.setInverted(true);

        this.stopperTop.turnToAngle(90);
        this.stopperBottom.turnToAngle(60);
        this.toggleSpike();

        this.slides.setDirection(DcMotorSimple.Direction.FORWARD);
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSafeguard(BooleanSupplier safeToMove) {
        this.safeToMove = safeToMove;
    }

    @Override
    public void periodic() {
        if (slides.getCurrentPosition() > 100 && raisingSlides) {
            while (blockerState != Blocker.TWO_PIXELS)
                this.toggleBlockers();

            if (spikeState == Spike.LOWERED)
                this.toggleSpike();

            raisingSlides = false;
        }
    }

    public Integer getSlidesTicks() {
        return slides.getCurrentPosition();
    }

    public void setSlidesTicks(int ticks) {
        if (!safeToMove.getAsBoolean())
            return;

        raisingSlides = slides.getTargetPosition() < 100 && ticks > 100;

        slides.setTargetPosition(ticks);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }

    public void setSlidesPosition(int position) {
        position = MathUtils.clamp(position, 0, 4);
        this.setSlidesTicks(slidesPositions[position]);
    }

    public void raiseSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        this.setSlidesPosition(current_pos < 0 ? -(current_pos + 1) : (current_pos + 1));
    }

    public void lowerSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        this.setSlidesPosition(current_pos < 0 ? -(current_pos + 1) - 1 : (current_pos - 1));
    }

    public void adjustSlidesTicks(int ticks) {
        this.setSlidesTicks(this.slides.getTargetPosition() + ticks);
    }

    public enum Spike {
        RAISED,
        LOWERED
    }

    public boolean slidesBusy() {
        return slides.isBusy();
    }

    public void toggleSpike() {
        if (!safeToMove.getAsBoolean())
            return;

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

    public void toggleBlockers() {
        switch (blockerState) {
            case FREE:
                stopperBottom.turnToAngle(108);
                stopperTop.turnToAngle(135);
                blockerState = Blocker.TWO_PIXELS;
                break;
            case ONE_PIXEL:
                stopperBottom.turnToAngle(60);
                blockerState = Blocker.FREE;
                break;
            case TWO_PIXELS:
                stopperTop.turnToAngle(90);
                blockerState = Blocker.ONE_PIXEL;
                break;
        }
    }

    public String getBlockerState() {
        return blockerState.toString();
    }
}
