package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for autonomous mode on competition bot
 */

public class AutonomousModeLogic extends AbstractModeLogic {

	private static final Logger logger = LogManager.getLogger(AutonomousModeLogic.class);

	private String autoOrigin;
	private String autoDestination;
	private String autoAction;
	private String combinedAuto;

	public AutonomousModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);

		autoOrigin = "none";
		autoDestination = "none";
		combinedAuto = "none";
	}

	@Override
	public void initialize() {
		logger.info("***** AUTONOMOUS *****");

		//Reads the values selected on the webdashboard and compiles them into the name of an auto.
		autoOrigin = (sharedInputValues.getString("ips_auto_origin").toLowerCase().replaceAll("\\s", ""));
		combinedAuto = "sq_auto_" + autoOrigin;
		if (autoOrigin.equals("doesnotexist")) {
			combinedAuto = "";
		}

		sharedInputValues.setString("ips_combined_auto", combinedAuto);
		logger.debug(combinedAuto);

		sharedInputValues.setBoolean("ipb_auto_complete", false);
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
			case "st_drivetrain_zero":
				return !sharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_climber_zero":
				return !sharedInputValues.getBoolean("ipb_climber_has_been_zeroed");
			case "st_collector_zero":
				return !sharedInputValues.getBoolean("ipb_collector_has_been_zeroed");
			case "st_elevator_zero":
				return !sharedInputValues.getBoolean("ipb_elevator_has_been_zeroed");
			case "st_flywheel_zero":
				return !sharedInputValues.getBoolean("ipb_flywheel_has_been_zeroed");
			case "st_hood_zero":
				return !sharedInputValues.getBoolean("ipb_hood_has_been_zeroed");
		}

		// Check isReady on auto states
		// This reads the string assembled from the webdashboard and checks it against all possible autos until it finds a match
		// If it doesn't find a match it does nothing
		if (!sharedInputValues.getBoolean("ipb_auto_complete") && sharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			return name.equals(combinedAuto);
		}

		return false;
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {

		}

		//Checks the isDone on zero states and determines when autonomous is done
		if (state.isDone()) {
			if (name.contains("auto")) {
				sharedInputValues.setBoolean("ipb_auto_complete", true);
			}
			return true;
		}
		return false;
	}
}
