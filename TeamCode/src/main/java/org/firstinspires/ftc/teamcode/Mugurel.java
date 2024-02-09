package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;

import java.lang.reflect.Field;
import java.util.Objects;

public class Mugurel extends Robot {

    private final CollectorSubsystem intake;
    private final OdometrySubsystem odometry;
    private final OpModeType opModeType;
    private TensorflowSubsystem tensorflow;
    private DepositSubsystem outtake;
    private EndgameSubsystem endgame;
    private DriveSubsystem driverControl;

    public Mugurel(HardwareMap hardwareMap, OpModeType opModeType) throws IllegalStateException {
        hardwareMap.getAll(LynxModule.class).forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        odometry = new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 180),
                new SimpleServo(hardwareMap, "odo_right", 0, 180),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
        intake = new CollectorSubsystem(
                new SimpleServo(hardwareMap, "v4b_left", 0, 180),
                new SimpleServo(hardwareMap, "v4b_right", 0, 180),
                new SimpleServo(hardwareMap, "claw", 0, 300)
        );

        this.opModeType = opModeType;
        switch (opModeType) {
            case TELEOP:
                odometry.raise();
                initTeleOp(hardwareMap);
                outtake.setSafeguard(() -> intake.location != CollectorSubsystem.LiftState.RAISED);
                break;
            case AUTO:
                odometry.lower();
                initAuto(hardwareMap);
            case TUNING:
                odometry.lower();
            default:
                throw new IllegalStateException("Couldn't start robot. Unknown OpMode type running.");
        }
    }

    private void initTeleOp(HardwareMap hardwareMap) {
        outtake = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 220),
                new SimpleServo(hardwareMap, "depo_right", 0, 220),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 300),
                hardwareMap.dcMotor.get("gli_sus")
        );
        endgame = new EndgameSubsystem(
                hardwareMap.dcMotor.get("pullup_left"),
                hardwareMap.dcMotor.get("pullup_right"),
                new SimpleServo(hardwareMap, "drone", -900, 900)
        );
        driverControl = new DriveSubsystem(hardwareMap, "leftFront", "rightFront",
                "leftBack", "rightBack");

        register(driverControl, outtake, intake);
    }

    private void initAuto(HardwareMap hardwareMap) {
        outtake = new DepositSubsystem(
                new SimpleServo(hardwareMap, "depo_left", 0, 220),
                new SimpleServo(hardwareMap, "depo_right", 0, 220),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 300),
                hardwareMap.dcMotor.get("gli_sus")
        );
    }

    public <T> T getSubsystem(Class<T> subsystemType) {
        Field[] fields = getClass().getDeclaredFields();

        for (Field field : fields)
            if (field.getType().equals(subsystemType)) {
                try {
                    field.setAccessible(true);
                    T subsystem = subsystemType.cast(field.get(this));
                    Objects.requireNonNull(subsystem, subsystemType.getSimpleName() + " is unavailable." +
                            "Current configuration: " + opModeType.toString());

                    return subsystem;
                } catch (IllegalAccessException e) {
                    throw new IllegalStateException("Error accessing field: " + field.getName(), e);
                }
            }

        throw new IllegalArgumentException("No " + subsystemType.getSimpleName() + " could be found.");
    }

    public enum OpModeType {
        AUTO, TELEOP, TUNING;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case AUTO:
                    return "Autonomous";
                case TELEOP:
                    return "TeleOp";
                case TUNING:
                    return "Autonomous Tuning OpMode";
                default:
                    return "Unknown";
            }
        }
    }
}
