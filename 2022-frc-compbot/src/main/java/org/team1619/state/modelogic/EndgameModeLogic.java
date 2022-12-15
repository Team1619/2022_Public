package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for endgame mode on competition bot
 */

public class EndgameModeLogic extends AbstractModeLogic {

	private static final Logger logger = LogManager.getLogger(EndgameModeLogic.class);

	private boolean isDeployed;
	private String climbAlignZeroButton;

	//private String climberStartPositionButton;
	private String climberSequenceButton;

	public EndgameModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);

		//climberStartPositionButton = robotConfiguration.getString("global_climber", "climber_start_position_button");
		climberSequenceButton = robotConfiguration.getString("global_climber", "climber_sequence_button");
		climbAlignZeroButton = robotConfiguration.getString("global_climber", "climb_align_zero_button");
	}

	@Override
	public void initialize() {
		logger.info("***** ENDGAME *****");

		isDeployed = false;
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
			case "st_drivetrain_climb_align_zero":
				return sharedInputValues.getBoolean(climbAlignZeroButton);
			case "st_climber_deploy":
				return (!sharedInputValues.getBoolean("ipb_climber_has_been_deployed"));
			case "sq_climber_climb":
				return sharedInputValues.getBooleanRisingEdge(climberSequenceButton) && sharedInputValues.getBoolean("ipb_climber_has_been_deployed");
			case "st_elevator_stop":
				return true;
			case "st_flywheel_stop":
				return true;
			case "st_drivetrain_climb":
				return sharedInputValues.getBoolean("ipb_climber_turn_wheels");
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_climb_align_zero":
				return !sharedInputValues.getBoolean(climbAlignZeroButton);
			case "st_drivetrain_climb":
				return false;
			case "st_elevator_stop":
				return false;
			default:
				return state.isDone();
		}
	}
}
