package org.firstinspires.ftc.teamcode.commands.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotSubsystem;

@Config
public class OdometrySubsystem extends RobotSubsystem {

    public static double PARALLEL_UP = 0, PARALLEL_DOWN = 1;
    public static double PERPENDICULAR_UP = 0, PERPENDICULAR_DOWN = 1;

    private final ServoEx left, right, back;
    private final InterpLUT leftTable = new InterpLUT();
    private final InterpLUT backTable = new InterpLUT();
    private final InterpLUT rightTable = new InterpLUT();

    @NonNull
    public static OdometrySubsystem createWithDefaults(final HardwareMap hardwareMap) {
        return new OdometrySubsystem(
                new SimpleServo(hardwareMap, "odo_left", 0, 180),
                new SimpleServo(hardwareMap, "odo_right", 0, 180),
                new SimpleServo(hardwareMap, "odo_back", 0, 1800)
        );
    }

    public OdometrySubsystem(ServoEx left, ServoEx right, ServoEx back) {
        this.left = left;
        this.right = right;
        this.back = back;

        this.left.setInverted(true);

        leftTable.add(-1e-9, 93);
        leftTable.add(1.0 + 1e-9, 125);

        rightTable.add(-1e-9, 87);
        rightTable.add(1.0 + 1e-9, 115);

        backTable.add(-1e-9, 400);
        backTable.add(0.5, 270);
        backTable.add(1.0 + 1e-9, 180);

        leftTable.createLUT();
        rightTable.createLUT();
        backTable.createLUT();
    }

    public void raise() {
        left.turnToAngle(leftTable.get(PARALLEL_UP));
        right.turnToAngle(rightTable.get(PARALLEL_UP));
        back.turnToAngle(backTable.get(PERPENDICULAR_UP));
    }

    public void lower() {
        left.turnToAngle(leftTable.get(PARALLEL_DOWN));
        right.turnToAngle(rightTable.get(PARALLEL_DOWN));
        back.turnToAngle(backTable.get(PERPENDICULAR_DOWN));
    }
}
