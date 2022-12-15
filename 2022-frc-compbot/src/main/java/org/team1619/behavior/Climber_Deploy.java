package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Controls the climber, using the state interrupt system
 */

public class Climber_Deploy implements Behavior {

	private static final Logger logger = LogManager.getLogger(Climber_Deploy.class);
	private static final Set<String> subsystems = Set.of("ss_climber");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final Timer climberDeployTimer;

	private final double errorThreshold;

	private double finalAngle;

	private int deployWaitTime;

	private boolean clawTop;
	private boolean clawBottom;
	private boolean climberExtend;
	private boolean isStep1Done;
	private boolean isStep2Done;

	public Climber_Deploy(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		climberDeployTimer = new Timer();

		errorThreshold = robotConfiguration.getDouble("global_climber", "error_threshold");

		finalAngle = 0.0;

		deployWaitTime = 0;

		clawTop = false;
		clawBottom = false;
		climberExtend = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		sharedInputValues.setBoolean("ipb_climber_has_been_deployed", false);

		climberDeployTimer.reset();

		finalAngle = config.getDouble("final_angle", 0.0);

		deployWaitTime = config.getInt("deploy_wait_time", 0);

		clawTop = config.getBoolean("claw_top", false);
		clawBottom = config.getBoolean("claw_bottom", false);
		climberExtend = config.getBoolean("climber_extend", false);
		isStep1Done = false;
		isStep2Done = false;
	}

	@Override
	public void update() {
		double currentAngle = sharedInputValues.getNumeric("ipn_climber_position");

		//Step 1 - deploy arm
		if (!climberDeployTimer.isStarted() && !isStep1Done) {
			climberExtend = true;
			climberDeployTimer.start(deployWaitTime);
		}

		if (climberDeployTimer.isDone() && !isStep1Done) {
			isStep1Done = true;
		}

		//Step 2 - move arm to final angle
		if (!sharedInputValues.getBoolean("ipb_climber_has_been_deployed") && isStep1Done) {
			sharedOutputValues.setNumeric("opn_climber", "motion_magic", finalAngle, "pr_deploy");
			if ((Math.abs(finalAngle - currentAngle) <= errorThreshold) && !isStep2Done) {
				isStep2Done = true;
				sharedInputValues.setBoolean("ipb_climber_has_been_deployed", true);
			}
		}

		sharedOutputValues.setBoolean("opb_claw_top", clawTop);
		sharedOutputValues.setBoolean("opb_claw_bottom", clawBottom);
		sharedOutputValues.setBoolean("opb_climber_deploy", climberExtend);
	}

	@Override
	public void dispose() {
	}

	@Override
	public boolean isDone() {
		 return sharedInputValues.getBoolean("ipb_climber_has_been_deployed");
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
