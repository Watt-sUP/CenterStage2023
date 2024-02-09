package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.EndgameSubsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Tomoiu + Vulpoiu)")
public class TeleOp extends CommandOpMode {

    public void initialize() {

        Mugurel robot = new Mugurel(hardwareMap, Mugurel.OpModeType.TELEOP);

        DriveSubsystem driveControl = robot.getSubsystem(DriveSubsystem.class);
        CollectorSubsystem intake = robot.getSubsystem(CollectorSubsystem.class);
        DepositSubsystem outtake = robot.getSubsystem(DepositSubsystem.class);
        EndgameSubsystem endgame = robot.getSubsystem(EndgameSubsystem.class);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        driveControl.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        // Brakes
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> driveControl.setPowerLimit(0.5))
                .whenReleased(() -> driveControl.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> driveControl.setPowerLimit(0.33))
                .whenReleased(() -> driveControl.setPowerLimit(1.0));

        // Endgame specific controls
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgame::toggleClimb);
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(endgame::launchPlane);

        // Slides commands
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(outtake::raiseSlidesPosition);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(outtake::lowerSlidesPosition);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> outtake.adjustSlidesTicks(-75));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> outtake.adjustSlidesTicks(75));

        // Intake commands
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> intake.adjustLiftPosition(0.0175));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> intake.adjustLiftPosition(-0.0175));
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(intake::toggleLiftLocation);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intake::toggleClamp);

        // Spike commands
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtake::toggleSpike);
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(outtake::toggleBlockers);

        schedule(new RunCommand(() -> {
            telemetry.addData("Blocker State", outtake.getBlockerState());
            telemetry.addData("Climb Position", endgame.getClimbState());
            telemetry.update();
        }));
    }
}
