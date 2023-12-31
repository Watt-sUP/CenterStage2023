package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class OdometrySubsystem extends SubsystemBase {

    public static double PARALLEL_UP = 0, PARALLEL_DOWN = 0.5;
    public static double PERPENDICULAR_UP = 0, PERPENDICULAR_DOWN = 1;

    private final ServoEx left, right, back;
    private final InterpLUT leftTable = new InterpLUT();
    private final InterpLUT backTable = new InterpLUT();
    private final InterpLUT rightTable = new InterpLUT();

    public OdometrySubsystem(ServoEx left, ServoEx right, ServoEx back) {
        this.left = left;
        this.right = right;
        this.back = back;

        this.back.setInverted(true);

        leftTable.add(-1e-9, 210);
        leftTable.add(0.5, 110);
        leftTable.add(1.0 + 1e-9, 0);

        rightTable.add(-1e-9, 205);
        rightTable.add(0.5, 115);
        rightTable.add(1.0 + 1e-9, 15);

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
