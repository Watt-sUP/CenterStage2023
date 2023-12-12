package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;

//@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {

    public void initialize() {
        OdometrySubsystem odometrySystem = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 300),
                new SimpleServo(hardwareMap, "odo_right", 0, 300),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        CollectorSubsystem collectorSystem = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300)
        );
        DepositSubsystem depositSystem = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 180),
                new SimpleServo(hardwareMap, "depo_right", 0, 180),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 1800),
                hardwareMap.dcMotor.get("gli_sus")
        );
        EndgameSubsystem endgameSystem = new EndgameSubsystem(
                hardwareMap.dcMotor.get("pullup_left"),
                hardwareMap.dcMotor.get("pullup_right"),
                new SimpleServo(hardwareMap, "drone", -900, 900)
        );
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");
        register(driveSystem, depositSystem, collectorSystem);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        // Raise odometry to avoid sliding
        odometrySystem.raise();
        depositSystem.setSafeguard(() -> collectorSystem.location != CollectorSubsystem.LiftState.RAISED);
        driveSystem.setDefaultCommand(driveCommand);

        // Brakes
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> driveSystem.setPowerLimit(0.5))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> driveSystem.setPowerLimit(0.25))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));

        // Endgame specific controls
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgameSystem::toggleClimb);
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(endgameSystem::launchPlane);

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(collectorSystem::toggleLiftLocation);

        // Slides commands
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(depositSystem::raiseSlidesPosition);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(depositSystem::lowerSlidesPosition);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> depositSystem.setSlidesPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> depositSystem.setSlidesPosition(0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> depositSystem.adjustSlidesTicks(-75));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> depositSystem.adjustSlidesTicks(75));


        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(depositSystem::toggleSpike);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(collectorSystem::toggleClamp);
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(depositSystem::toggleBlockers);

        schedule(new RunCommand(() -> {
            telemetry.addData("Power Limit", driveSystem.getPowerLimit());
            telemetry.addData("Slides Ticks", depositSystem.getSlidesTicks());
            telemetry.addData("Blocker State", depositSystem.getBlockerState());
            telemetry.update();
        }));
    }
}
