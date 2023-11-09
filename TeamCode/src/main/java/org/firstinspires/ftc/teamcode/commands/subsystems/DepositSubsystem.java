package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.annotation.Nullable;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final ServoEx leftLift, rightLift;
    private final DcMotor slides;
    public static Double LOW_LEFT = 0.04, LOW_RIGHT = 0.06;
    public static Double HIGH_LEFT = 0.56, HIGH_RIGHT = 0.55;

    public DepositSubsystem(ServoEx leftLift, ServoEx rightLift, @Nullable ServoEx stopper, DcMotor slides) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.slides = slides;

        this.leftLift.setInverted(true);
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

    public void lowerSpike() {
        leftLift.setPosition(LOW_LEFT);
        rightLift.setPosition(LOW_RIGHT);
    }
}
