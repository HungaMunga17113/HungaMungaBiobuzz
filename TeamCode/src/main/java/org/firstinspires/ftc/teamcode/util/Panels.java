package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public final class Panels {
    public static final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private Panels() {}
}
