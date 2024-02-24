package org.firstinspires.ftc.teamcode.commands.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotSubsystem;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends RobotSubsystem {

    private double powerLimit = 1.0;
    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;

    @NonNull
    public static DriveSubsystem createWithDefaults(final HardwareMap hardwareMap) {
        return new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");
    }

    @Override
    public void periodic() {
        if (Arrays.asList(forward, strafe, rotation).contains(null))
            return;

        updateSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble());
    }

    public DriveSubsystem(HardwareMap hardwareMap, String leftFront, String rightFront, String leftBack, String rightBack) {
        drive = new MecanumDrive(
                new Motor(hardwareMap, leftFront),
                new Motor(hardwareMap, rightFront),
                new Motor(hardwareMap, leftBack),
                new Motor(hardwareMap, rightBack)
        );
    }

    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    public void updateSpeeds(double fwd, double st, double rot) {
        drive.driveRobotCentric(st, fwd, rot);
    }

    public void setPowerLimit(double limit) {
        powerLimit = MathUtils.clamp(Math.abs(limit), 0, 1);
        drive.setMaxSpeed(powerLimit);
    }
}