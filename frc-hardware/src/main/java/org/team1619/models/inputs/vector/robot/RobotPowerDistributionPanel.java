package org.team1619.models.inputs.vector.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.team1619.models.inputs.vector.PowerDistributionPanel;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;

import java.util.HashMap;
import java.util.Map;

public class RobotPowerDistributionPanel extends PowerDistributionPanel {

    private final PowerDistribution powerDistribution;

    public RobotPowerDistributionPanel (Object name, Config config, HardwareFactory sharedHardwareFactory){
        super(name, config);
        powerDistribution = sharedHardwareFactory.get(PowerDistribution.class, deviceNumber, moduleType);
    }

    @Override
    protected Map<String, Double> readHardware() {
        // Make local map to overwrite port ID and current pairs
        Map<String, Double> powerDistributionReadings = new HashMap<>();

        // iterate through the number of ports on the given power distribution device
        for (Integer i = 0; i < powerDistribution.getNumChannels(); i++) {
            // populate the map
            powerDistributionReadings.put(i.toString(), powerDistribution.getCurrent(i));
        }

        return powerDistributionReadings;
    }
}
