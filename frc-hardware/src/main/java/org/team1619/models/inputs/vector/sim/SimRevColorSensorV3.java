package org.team1619.models.inputs.vector.sim;

import org.team1619.models.inputs.vector.ColorSensor;
import org.uacr.shared.abstractions.EventBus;
import org.uacr.utilities.Config;

import java.util.Map;

public class SimRevColorSensorV3 extends ColorSensor {
    private final SimInputVectorListener listener;

    public SimRevColorSensorV3(EventBus eventbus, Object name, Config config) {
        super(name, config);
        listener = new SimInputVectorListener(eventbus, name, Map.of("red", 0.0, "green", 0.0, "blue",0.0, "IR",0.0, "proximity", 0.0));
    }

    @Override
    public Map<String, Double> getValue() {
        return listener.get();
    }
}
