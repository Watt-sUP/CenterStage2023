package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    private double powerLimit = 1.0;
    private final MecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap, String leftFront, String rightFront, String leftBack, String rightBack) {
        drive = new MecanumDrive(
                new Motor(hardwareMap, leftFront),
                new Motor(hardwareMap, rightFront),
                new Motor(hardwareMap, leftBack),
                new Motor(hardwareMap, rightBack)
        );
    }

    public void updateSpeeds(double fwd, double st, double rot, double heading) {
        drive.driveFieldCentric(st, fwd, rot, heading);
    }

    public double getPowerLimit() {
        return powerLimit;
    }

    public void setPowerLimit(double limit) {
        if (MathUtils.clamp(Math.abs(limit), 0, 1) == powerLimit)
            return;

        powerLimit = MathUtils.clamp(Math.abs(limit), 0, 1);
        drive.setMaxSpeed(powerLimit);
    }
}