package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private DoubleSupplier heading = () -> 0;
    private final DoubleSupplier forward, strafe, rotation;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        addRequirements(drive);
    }

    public DriveCommand(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe,
                        DoubleSupplier rotation, DoubleSupplier heading) {
        this(drive, forward, strafe, rotation);
        this.heading = heading;
    }

    @Override
    public void execute() {
        drive.updateSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble(), heading.getAsDouble());
    }
}