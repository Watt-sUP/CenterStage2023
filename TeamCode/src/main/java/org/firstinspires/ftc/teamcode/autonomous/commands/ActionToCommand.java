package org.firstinspires.ftc.teamcode.autonomous.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class ActionToCommand extends CommandBase {
    private final Action action;
    private boolean finished = false;

    public ActionToCommand(Action action) {
        this.action = action;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket(true);
        assert packet.fieldOverlay() != null : "Packet is null";
        action.preview(packet.field());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
