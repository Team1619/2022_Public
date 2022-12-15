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
 * Controls the climber subsystem, using the state interrupt system
 */

public class Climber_States implements Behavior {

	private static final Logger logger = LogManager.getLogger(Climber_States.class);
	private static final Set<String> subsystems = Set.of("ss_climber");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final String speedJoystick;
	private final String manualAdjustButton;
	private final String climberStallButton;

	private final double errorThreshold;

	private double desiredArmPosition;
	private double clawAtPosition;

	private boolean desiredArmReached;
	private boolean clawTop;
	private boolean clawBottom;
	private boolean adjustClimber;
	private boolean turnWheels;

	public Climber_States(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		speedJoystick = robotConfiguration.getString("global_climber", "climber_rotate_joystick");
		manualAdjustButton = robotConfiguration.getString("global_climber", "sequence_adjust_button");
		climberStallButton = robotConfiguration.getString("global_climber", "climber_stall_button");

		errorThreshold = robotConfiguration.getDouble("global_climber", "error_threshold");

		desiredArmPosition = 0.0;
		clawAtPosition = 0.0;

		desiredArmReached = false;
		clawTop = false;
		clawBottom = false;
		adjustClimber = false;
		turnWheels = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		desiredArmPosition = config.getDouble("arm_position", 0.0);
		clawAtPosition = config.getDouble("activate_claw_values_at_position", 0.0);

		clawTop = config.getBoolean("claw_top", false);
		clawBottom = config.getBoolean("claw_bottom", false);
		adjustClimber = config.getBoolean("adjust_climber", false);
		turnWheels = config.getBoolean("turn_wheels", false);
		sharedInputValues.setBoolean("ipb_climber_turn_wheels", turnWheels);

		sharedInputValues.setNumeric("ipn_climber_desired_arm_position", desiredArmPosition);
		desiredArmReached = false;

		boolean coastMode = config.getBoolean("coast_mode", false);
		String motorMode = coastMode ? "coast" : "brake";
		sharedOutputValues.setOutputFlag("opn_climber", motorMode);
	}

	@Override
	public void update() {
		if (sharedInputValues.getBoolean(climberStallButton)) {
			if (adjustClimber && sharedInputValues.getBoolean(manualAdjustButton)) {
				double joystick = sharedInputValues.getNumeric((speedJoystick));
				sharedOutputValues.setNumeric("opn_climber", "percent", joystick);
			} else {
				sharedOutputValues.setNumeric("opn_climber", "percent", 0.0);
			}
		} else {
			double currentArmPosition = sharedInputValues.getNumeric("ipn_climber_position");

			sharedOutputValues.setNumeric("opn_climber", "motion_magic", desiredArmPosition, "pr_climb");

			if (clawAtPosition == 0.0 || (clawAtPosition - currentArmPosition) <= errorThreshold) {
				sharedOutputValues.setBoolean("opb_claw_top", clawTop);
			}

			sharedOutputValues.setBoolean("opb_claw_bottom", clawBottom);

			if (Math.abs(desiredArmPosition - currentArmPosition) <= errorThreshold) {
				desiredArmReached = true;
			}

			sharedInputValues.setBoolean("ipb_climber_desired_arm_position_reached", desiredArmReached);
		}
	}

	@Override
	public void dispose() {
	}

	@Override
	public boolean isDone() {
		return desiredArmReached;
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
