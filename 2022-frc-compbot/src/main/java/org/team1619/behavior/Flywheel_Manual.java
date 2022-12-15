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
 * Manual control for the flywheels; sets flywheel speed
 */

public class Flywheel_Manual implements Behavior {

	private static final Logger logger = LogManager.getLogger(Flywheel_Manual.class);
	private static final Set<String> subsystems = Set.of("ss_flywheel");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final String onButton;
	private final String increaseButton;
	private final String decreaseButton;

	private double startSpeed;
	private double flywheelSpeed;
	private double incrementSpeed;
	private double offsetSpeed;

	public Flywheel_Manual(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		increaseButton = robotConfiguration.getString("global_manual", "manual_flywheel_increase_velocity");
		decreaseButton = robotConfiguration.getString("global_manual", "manual_flywheel_decrease_velocity");
		onButton = robotConfiguration.getString("global_manual", "manual_flywheel_on_button");

		startSpeed = 0.0;
		flywheelSpeed = 0.0;
		incrementSpeed = 0.0;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		startSpeed = config.getDouble("start_speed", 0.0);
		incrementSpeed = config.getDouble("increment_speed", 0.0);
	}

	@Override
	public void update() {
		if (sharedInputValues.getBoolean(onButton)) {
			if (sharedInputValues.getBoolean(increaseButton)) {
				offsetSpeed += incrementSpeed;
			}
			if (sharedInputValues.getBoolean(decreaseButton)) {
				offsetSpeed -= incrementSpeed;
			}
			if (offsetSpeed < -startSpeed) {
				offsetSpeed = -startSpeed;
			}
			if (offsetSpeed > (1.0 - startSpeed)) {
				offsetSpeed = (1.0 - startSpeed);
			}

			flywheelSpeed = startSpeed + offsetSpeed;
		}
		else {
			flywheelSpeed = 0.0;
		}

		sharedInputValues.setNumeric("ipn_flywheel_velocity", flywheelSpeed);
		sharedOutputValues.setNumeric("opn_flywheel", "percent", flywheelSpeed);
	}

	@Override
	public void dispose() {
		sharedOutputValues.setNumeric("opn_flywheel", "percent", 0.0);
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
