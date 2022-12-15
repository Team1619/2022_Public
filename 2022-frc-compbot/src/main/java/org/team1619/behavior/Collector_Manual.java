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
 * Manual control for the collector; sets roller speed, extends/retracts collector
 */

public class Collector_Manual implements Behavior {

	private static final Logger logger = LogManager.getLogger(Collector_Manual.class);
	private static final Set<String> subsystems = Set.of("ss_collector");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final String extendButton;
	private final String rollerOnButton;

	private double rollerSpeed;
	private double speed;

	private boolean position;

	public Collector_Manual(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		extendButton = robotConfiguration.getString("global_manual", "manual_collector_extend_button");
		rollerOnButton = robotConfiguration.getString("global_manual", "manual_collector_roller_on_button");

		rollerSpeed = 0.0;
		speed = 0.0;

		position = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		rollerSpeed = config.getDouble("roller_speed", 0.0);
		speed = 0.0;

		position = config.getBoolean("position", false);
	}

	@Override
	public void update() {
		if (sharedInputValues.getBooleanRisingEdge(extendButton)) {
			position = !position;
		}

		if (sharedInputValues.getBoolean(rollerOnButton)) {
			speed = rollerSpeed;
		} else {
			speed = 0.0;
		}

		sharedOutputValues.setBoolean("opb_collector", position);
		sharedOutputValues.setNumeric("opn_collector_rollers", "percent", speed);
	}

	@Override
	public void dispose() {
		sharedOutputValues.setBoolean("opb_collector", false);
		sharedOutputValues.setNumeric("opn_collector_rollers", "percent", 0.0);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
