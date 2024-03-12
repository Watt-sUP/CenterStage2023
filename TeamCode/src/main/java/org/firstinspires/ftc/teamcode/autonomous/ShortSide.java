package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceLocation;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.commands.ActionToCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.TensorflowDetectCommand;
import org.firstinspires.ftc.teamcode.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DepositSubsystem;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

@Config
@Autonomous(name = "Short Auto (Side)", group = "short")
public class ShortSide extends CommandOpMode {
    public static AllianceLocation location = AllianceLocation.RED_SHORT;

    @Override
    public void initialize() {

        assert location.name().contains("SHORT") : "Unable to run autonomous: " +
                "The robot must be at the backdrop position on either side";

        TensorflowDetectCommand propDetection = new TensorflowDetectCommand(hardwareMap, location);
        propDetection.setEndCondition(this::isStarted);

        MecanumDrive drive = new MecanumDrive(hardwareMap, location.getStartingPosition());
        PathGenerator pathGenerator = new PathGenerator(drive, location);

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        Map<PropLocations, Action> purpleCases = pathGenerator.generatePurplePaths();
        Map<PropLocations, Action> yellowCases = pathGenerator.generateYellowPaths();

        Map<PropLocations, Action> firstStack = Arrays.stream(PropLocations.values())
                .collect(Collectors.toMap(
                        location -> location,
                        location -> pathGenerator.generateStackPath(
                                new Pose2d(pathGenerator.yellowLocations.get(location), Math.PI),
                                PathGenerator.Stack.CLOSE
                        )
                ));

        schedule(propDetection.andThen(
                new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                new ActionToCommand(purpleCases.get(propDetection.detectedLocation)),
                new InstantCommand(intake::toggleLiftLocation),
                new ParallelCommandGroup(
                        new ActionToCommand(yellowCases.get(propDetection.detectedLocation)),
                        new InstantCommand(() -> {
                            intake.setClampPosition(25);
                            intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                            outtake.toggleBlockers();
                            outtake.toggleSpike();
                        })
                ),
                new InstantCommand(outtake::toggleBlockers).andThen(
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleBlockers),
                        new WaitCommand(500),
                        new InstantCommand(outtake::toggleSpike)
                ),
                new WaitCommand(500),
                new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.RAISED))
        ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (!isStopRequested() && !Thread.currentThread().isInterrupted())
            this.run();

        this.reset();
    }
}
