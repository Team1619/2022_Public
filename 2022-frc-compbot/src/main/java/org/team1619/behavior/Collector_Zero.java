package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Zeroes the collector subsystem
 */

public class Collector_Zero implements Behavior {

	private static final Logger logger = LogManager.getLogger(Collector_Zero.class);
	private static final Set<String> subsystems = Set.of("ss_collector");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	public Collector_Zero(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		sharedOutputValues.setBoolean("ipb_collector", false);
		sharedOutputValues.setNumeric("opn_collector_rollers", "percent",0.0);
	}

	@Override
	public void update() {
		if (!sharedInputValues.getBoolean("ipb_collector_has_been_zeroed")) {
			sharedOutputValues.setBoolean("ipb_collector", false);
			sharedOutputValues.setNumeric("opn_collector_rollers", "percent", 0.0);
			sharedInputValues.setBoolean("ipb_collector_has_been_zeroed", true);
			logger.debug("Collector Zero -> Zeroed");
		}
	}

	@Override
	public void dispose() {
		sharedOutputValues.setBoolean("ipb_collector", false);
		sharedOutputValues.setNumeric("opn_collector_rollers", "percent",0.0);
	}

	@Override
	public boolean isDone() {
		return sharedInputValues.getBoolean("ipb_collector_has_been_zeroed");
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
