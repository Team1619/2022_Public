package org.team1619.models.inputs.vector.sim;

import org.team1619.models.inputs.vector.PowerDistributionPanel;
import org.uacr.shared.abstractions.EventBus;
import org.uacr.utilities.Config;

import java.util.HashMap;
import java.util.Map;

public class SimPowerDistributionPanel extends PowerDistributionPanel {

    private final SimInputVectorListener listener;

    public SimPowerDistributionPanel(EventBus eventBus, Object name, Config config) {
        super(name, config);
        Map<String, Double> powerDistributionMap = new HashMap<>();
        switch (moduleType) {
            case kCTRE:
                for (Integer i = 0; i < 16; i++) {
                    powerDistributionMap.put(i.toString(), 0.0);
                }
                break;
            case kRev:
                for (Integer i = 0; i < 24; i++) {
                    powerDistributionMap.put(i.toString(), 0.0);
                }
                break;
        }
        listener = new SimInputVectorListener(eventBus, name, powerDistributionMap);
    }

    @Override
    public void initialize() {

    }

    @Override
    protected Map<String, Double> readHardware() {
        return listener.get();
    }
}
