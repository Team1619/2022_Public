package org.team1619.modelfactory;

import org.team1619.behavior.*;
import org.uacr.models.behavior.Behavior;
import org.uacr.models.exceptions.ConfigurationException;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.ObjectsDirectory;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

public class ModelFactory_Behaviors extends AbstractModelFactory {

	private static final Logger logger = LogManager.getLogger(ModelFactory_Behaviors.class);

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;
	private final RobotConfiguration robotConfiguration;

	public ModelFactory_Behaviors(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration, ObjectsDirectory objectsDirectory) {
		super(inputValues, outputValues, robotConfiguration, objectsDirectory);
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;
		this.robotConfiguration = robotConfiguration;
	}

	public Behavior createBehavior(String name, Config config) {
		logger.trace("Creating behavior '{}' of type '{}' with config '{}'", name, config.getType(), config.getData());

		switch (name) {
			// ------ Drivetrain ------ //
			case "bh_drivetrain_swerve_zero":
				return new Drivetrain_Swerve_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_drivetrain_swerve":
				return new Drivetrain_Swerve(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_drivetrain_swerve_pure_pursuit":
				return new Drivetrain_Swerve_Pure_Pursuit(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Climber ------ //
			case "bh_climber_zero":
				return new Climber_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_climber_states":
				return new Climber_States(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_climber_deploy":
				return new Climber_Deploy(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_manual_climber":
				return new Climber_Manual(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Collector ------ //
			case "bh_collector_zero":
				return new Collector_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_collector_manual":
				return new Collector_Manual(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_collector_states":
				return new Collector_States(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Elevator ------ //
			case "bh_elevator_zero":
				return new Elevator_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_elevator_states":
				return new Elevator_States(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_elevator_manual":
				return new Elevator_Manual(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Hood ---------- //
			case "bh_hood_zero":
				return new Hood_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Flywheel ------ //
			case "bh_flywheel_zero":
				return new Flywheel_Zero(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_flywheel_states":
				return new Flywheel_States(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_flywheel_manual":
				return new Flywheel_Manual(sharedInputValues, sharedOutputValues, robotConfiguration);

			// ------ Hood ------ //
			case "bh_hood_states":
				return new Hood_States(sharedInputValues, sharedOutputValues, robotConfiguration);
			case "bh_hood_manual":
				return new Hood_Manual(sharedInputValues,sharedOutputValues, robotConfiguration);

			// State not found
			default:
				throw new ConfigurationException("Behavior " + name + " does not exist.");
		}
	}

}
