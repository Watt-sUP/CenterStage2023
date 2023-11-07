package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TransferSubsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    public static Integer SLIDES_TICKS = 250;

    public void initialize() {
        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                null
        );
        TransferSubsystem transferSystem = new TransferSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                null
        );
        DepositSubsystem depositSystem = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 300),
                new SimpleServo(hardwareMap, "depo_right", 0, 300),
                null
        );
        SlidesSubsystem slidesSystem = new SlidesSubsystem(
                hardwareMap.dcMotor.get("gli_jos"),
                hardwareMap.dcMotor.get("gli_sus")
        );
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");
        register(driveSystem, transferSystem, odometrySystem, depositSystem, slidesSystem);

        GamepadEx gamepad = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, gamepad::getLeftY, gamepad::getLeftX, gamepad::getRightX);

        odometrySystem.raise();
        driveSystem.setDefaultCommand(driveCommand);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> driveSystem.setPowerLimit(0.5))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(transferSystem::raiseLift, transferSystem::lowerLift);
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(depositSystem::raise, depositSystem::lower);
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> slidesSystem.setToTicks(SLIDES_TICKS));
    }
}
