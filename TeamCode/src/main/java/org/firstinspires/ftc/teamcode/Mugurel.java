package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.RobotSubsystem;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Objects;

public class Mugurel extends Robot {

    private CollectorSubsystem intake;
    private OdometrySubsystem odometry;
    private final OpModeType opModeType;
    private DepositSubsystem outtake;
    private EndgameSubsystem endgame;
    private DriveSubsystem driverControl;

    public Mugurel(HardwareMap hardwareMap, OpModeType opModeType) throws IllegalStateException {
        CommandScheduler.getInstance().reset();
        hardwareMap.getAll(LynxModule.class).forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        loadSubsystems(hardwareMap, CollectorSubsystem.class, RobotSubsystem.class);

        this.opModeType = opModeType;
        switch (opModeType) {
            case TELEOP:
                loadSubsystems(hardwareMap, DepositSubsystem.class, EndgameSubsystem.class, DriveSubsystem.class);

                odometry.raise();
                outtake.setSafeguard(() -> intake.location != CollectorSubsystem.LiftState.RAISED);
                register(driverControl, outtake, intake);
                break;
            case AUTO:
                odometry.lower();
                loadSubsystems(hardwareMap, DepositSubsystem.class);
                break;
            case TUNING:
                odometry.lower();
                break;
            default:
                throw new IllegalStateException("Couldn't start robot. Unknown OpMode type running.");
        }
    }

    public static Mugurel getInstance() {
        // TODO: Finish the stuff before this
        return null;
    }

    @SafeVarargs
    private final void loadSubsystems(final HardwareMap hardwareMap, Class<? extends RobotSubsystem>... subsystems) {
        Field[] fields = this.getClass().getDeclaredFields();

        for (Class<? extends RobotSubsystem> subsystem : subsystems)
            for (Field field : fields)
                if (field.getType().equals(subsystem))
                    try {
                        // Get the value of the field
                        field.setAccessible(true);
                        Object value = subsystem.cast(field.get(this));

                        // If it's not initialized, do so with default settings
                        if (value == null) {
                            Method subsystemCreator = subsystem.getMethod("createWithDefaults", HardwareMap.class);
                            field.set(this, subsystemCreator.invoke(null, hardwareMap));
                        }
                    } catch (IllegalAccessException | InvocationTargetException e) {
                        throw new IllegalStateException("Error accessing field: " + field.getName(), e);
                    } catch (NoSuchMethodException e) {
                        throw new IllegalStateException("Unable to initialize subsystem. " +
                                subsystem.getSimpleName() + " is missing the 'createWithDefaults' method.", e);
                    }
    }

    @NonNull
    public final <T extends RobotSubsystem> T getSubsystem(Class<T> subsystemType) {
        Field[] fields = this.getClass().getDeclaredFields();

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
        AUTO,
        TELEOP,
        TUNING;

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
