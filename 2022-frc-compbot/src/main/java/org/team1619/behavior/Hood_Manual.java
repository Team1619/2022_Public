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
 * Manual control for Hood; sets the hood position
 */

public class Hood_Manual implements Behavior {

	private static final Logger logger = LogManager.getLogger(Hood_Manual.class);
	private static final Set<String> subsystems = Set.of("ss_hood");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final String hoodUpButton;
	private final String hoodDownButton;

	private double baseSpeed;

	public Hood_Manual(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		hoodUpButton = robotConfiguration.getString("global_manual", "manual_hood_up_button");
		hoodDownButton = robotConfiguration.getString("global_manual", "manual_hood_down_button");
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		baseSpeed = config.getDouble("speed", 0.0);
	}

	@Override
	public void update() {
		if ((sharedInputValues.getNumeric("ipn_hood_position") < 30.0) && (sharedInputValues.getNumeric("ipn_hood_position") > 0.0)) {
			double speed = 0.0;

			if (sharedInputValues.getBoolean(hoodUpButton)) {
				speed = baseSpeed;
			} else if (sharedInputValues.getBoolean(hoodDownButton)) {
				speed = -baseSpeed;
			}

			sharedOutputValues.setNumeric("opn_hood", "percent", speed);
		}
	}

	@Override
	public void dispose() {
		sharedOutputValues.setNumeric("opn_hood", "percent", 0.0);
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
