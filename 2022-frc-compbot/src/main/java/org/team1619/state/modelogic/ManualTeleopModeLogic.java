package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for manual teleop mode on competition bot
 */

public class ManualTeleopModeLogic extends AbstractModeLogic {

	private static final Logger logger = LogManager.getLogger(ManualTeleopModeLogic.class);

	public ManualTeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		logger.info("***** MANUAL_TELEOP *****");
	}

	@Override
	public void update() {

	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
			case "st_climber_manual":
				return true;
			case "st_collector_manual":
				return true;
			case "st_elevator_manual":
				return true;
			case "st_flywheel_manual":
				return true;
			case "st_hood_manual":
				return true;
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			default:
				return false;
		}
	}
}
