package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    public static Integer SLIDES_TICKS = 1300, PULL_TICKS = 3750;

    /* TODO:
     * To avoid a complicated sequence, when grabbing a pixel automatize the following steps:
     *   1. Close the claw
     *   2. Raise the lift and rotate the claw simultaneously
     *
     * When coming back:
     *   1. Make sure the clamping is smaller than 0.2, otherwise close the claw
     *   2. Rotate the claw back and lower the lift
     *   3. Once that's done, open the claw
     */
    public void initialize() {
        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                null
        );
        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300),
                new SimpleServo(hardwareMap, "claw_r", 0, 1800)
        );
        DepositSubsystem depositSystem = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 300),
                new SimpleServo(hardwareMap, "depo_right", 0, 300),
                null
        );
        SlidesSubsystem slidesSystem = new SlidesSubsystem(
                null,
                hardwareMap.dcMotor.get("gli_sus")
        );
        ClimbSubsystem climbSystem = new ClimbSubsystem(
                hardwareMap.dcMotor.get("pull_up"),
                null
        );
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");
        register(driveSystem, collectorSystem, odometrySystem, depositSystem, slidesSystem, climbSystem);

        GamepadEx gamepad = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, gamepad::getLeftY, gamepad::getLeftX, gamepad::getRightX);

        odometrySystem.raise();
        driveSystem.setDefaultCommand(driveCommand);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> driveSystem.setPowerLimit(0.5))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));

        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(collectorSystem::raiseLift, collectorSystem::lowerLift);
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(depositSystem::raise, depositSystem::lower);
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        () -> slidesSystem.setToTicks(SLIDES_TICKS),
                        () -> slidesSystem.setToTicks(0)
                );
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> climbSystem.setToTicks(PULL_TICKS));
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(collectorSystem::rotate);
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(collectorSystem::toggle);

        schedule(new RunCommand(() -> {
            telemetry.addData("Slides Ticks", slidesSystem.getTicks());
            telemetry.addData("Climber Ticks", climbSystem.getTicks());
            telemetry.update();
        }));
    }
}
