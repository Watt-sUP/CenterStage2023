package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    public static Integer SLIDES_TICKS = 1300;

    /* TODO:
     * To avoid a complicated sequence, when grabbing a pixel automatize the following steps:
     *   1. Close the claw
     *   2. Raise the lift and rotate the claw simultaneously (Done, must test)
     *
     * When coming back:
     *   1. Make sure the clamping is smaller than 0.2, otherwise close the claw
     *   2. Rotate the claw back and move it to an idle location
     *   3. Once that's done, open the claw
     *   (Made an alternative, first 2 steps are automatic, last one must be done manually)
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
                null, hardwareMap.dcMotor.get("gli_sus")
        );
        ClimbSubsystem climbSystem = new ClimbSubsystem(
                hardwareMap.dcMotor.get("pull_up"),
                null
        );
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");
        register(driveSystem, collectorSystem, odometrySystem, depositSystem, climbSystem);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        odometrySystem.raise();
        driveSystem.setDefaultCommand(driveCommand);
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> driveSystem.setPowerLimit(0.5))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(climbSystem::toggle);

        // Either raise the lift, or lower it
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(
                        () -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.RAISED),
                        () -> collectorSystem.setLiftLocation(CollectorSubsystem.LiftState.LOWERED)
                );

        // Slides commands
        // TODO: Find slides positions for each line on the backdrop
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> depositSystem.setSlidesTicks(SLIDES_TICKS));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> depositSystem.setSlidesTicks(0));
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(depositSystem::raiseSpike, depositSystem::lowerSpike);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(collectorSystem::toggle);

        schedule(new RunCommand(() -> {
            telemetry.addData("Slides Ticks", depositSystem.getSlidesTicks());
            telemetry.addData("Climber Ticks", climbSystem.getTicks());
            telemetry.update();
        }));
    }
}
