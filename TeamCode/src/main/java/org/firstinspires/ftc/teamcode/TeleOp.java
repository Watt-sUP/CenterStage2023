package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Tomoiu + Vulpoiu)")
public class TeleOp extends CommandOpMode {
    private List<LynxModule> hubs;

    public void initialize() {

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);
        EndgameSubsystem endgame = new EndgameSubsystem(hardwareMap);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);
        outtake.setSafeguard(() -> intake.location != CollectorSubsystem.LiftState.RAISED);
        Trigger rightTrigger = new Trigger(() -> gamepad2.right_trigger > .3 && outtake.spikeState == DepositSubsystem.Spike.RAISED);

        register(chassis, intake, outtake, endgame);

        // Brakes
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.5))
                .whenReleased(() -> chassis.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.33))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        // Endgame specific controls
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgame::toggleElevator);
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
                .whenPressed(() -> intake.adjustLiftPosition(5.0));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> intake.adjustLiftPosition(-5.0));
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(intake::toggleLiftLocation);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intake::toggleClamp);

        // Spike commands
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtake::toggleSpike);
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(outtake::toggleBlockers);
        rightTrigger.toggleWhenActive(
                () -> outtake.setSpikePosition(.925),
                () -> outtake.setSpikePosition(DepositSubsystem.HIGH_RIGHT)
        );

        schedule(new RunCommand(() -> {
            telemetry.addData("Blocker State", outtake.getBlockerState());
            telemetry.addData("Elevator Position", endgame.getElevatorState());
            telemetry.addData("Elevator Angle", endgame.getElevatorAngle());
            telemetry.update();
        }));
    }

    @Override
    public void run() {
        super.run();
        hubs.forEach(LynxModule::clearBulkCache);
    }
}
