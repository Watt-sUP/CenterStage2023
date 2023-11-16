package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import javax.annotation.Nullable;

public class OdometrySubsystem extends SubsystemBase {

    Map<String, ServoEx> odometry = new HashMap<>();
    Map<String, Pair<Double, Double>> positions = new HashMap<String, Pair<Double, Double>>() {{
        put("left", new Pair<>(0.35, 0.8));
        put("right", new Pair<>(0.3, 0.75));
        put("front", new Pair<>(160.0 / 1800.0, 0.));
    }};

    public OdometrySubsystem(@Nullable ServoEx left, @Nullable ServoEx right, @Nullable ServoEx front) {
        odometry.put("left", left);
        odometry.put("right", right);
        odometry.put("front", front);
    }

    public void raise() {
        for (String name : odometry.keySet()) {
            ServoEx servo = odometry.get(name);
            if (servo != null)
                servo.setPosition(Objects.requireNonNull(positions.get(name)).second);
        }
    }

    public void lower() {
        for (String name : odometry.keySet()) {
            ServoEx servo = odometry.get(name);
            if (servo != null)
                servo.setPosition(Objects.requireNonNull(positions.get(name)).first);
        }
    }
}
