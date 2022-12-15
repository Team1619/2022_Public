package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.Timer;

import java.util.Set;

/**
 * Manual control for the climber; sets speed for arm, closes/opens claws
 */

public class Climber_Manual implements Behavior {

    private static final Logger logger = LogManager.getLogger(Climber_Manual.class);
    private static final Set<String> subsystems = Set.of("ss_climber");

    private final InputValues sharedInputValues;
    private final OutputValues sharedOutputValues;

    private final Timer climberDeployTimer;

    private final String climberDeployButton;
    private final String onButton;
    private final String speedJoystick;
    private final String clawTopButton;
    private final String clawBottomButton;

    private int climberDeployTime;

    private boolean clawTop;
    private boolean clawBottom;
    private boolean climberDeploy;

    public Climber_Manual(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
        sharedInputValues = inputValues;
        sharedOutputValues = outputValues;

        climberDeployTimer = new Timer();

        climberDeployButton = robotConfiguration.getString("global_manual", "manual_climber_deploy_button");
        speedJoystick = robotConfiguration.getString("global_manual", "manual_climber_rotate_joystick");
        onButton = robotConfiguration.getString("global_manual", "manual_climber_rotate_on_button");
        clawTopButton = robotConfiguration.getString("global_manual", "manual_claw_top_button");
        clawBottomButton = robotConfiguration.getString("global_manual", "manual_claw_bottom_button");

        climberDeployTime = 0;

        clawTop = false;
        clawBottom = false;
        climberDeploy = false;
    }

    @Override
    public void initialize(String stateName, Config config) {
        logger.debug("Entering state {}", stateName);

        sharedOutputValues.setOutputFlag("opn_climber", "coast");

        climberDeployTimer.reset();

        climberDeployTime = config.getInt("climber_deploy_time", 0);

        clawTop = config.getBoolean("claw_top", false);
        clawBottom = config.getBoolean("claw_bottom", false);
        climberDeploy = config.getBoolean("climber_deploy", false);
    }

    @Override
    public void update() {
        double climbSpeed = 0.0;

        if (sharedInputValues.getBoolean(climberDeployButton)) {
            if (!climberDeployTimer.isStarted()) {
                climberDeployTimer.start(climberDeployTime);
            }
            if (climberDeployTimer.isDone()) {
                climberDeploy = true;
            }
        } else {
            climberDeployTimer.reset();
        }

        if (sharedInputValues.getBoolean(onButton)) {
            double joystick = sharedInputValues.getNumeric((speedJoystick));
            climbSpeed = Math.pow(joystick, 3);
        }

        sharedOutputValues.setNumeric("opn_climber", "percent", climbSpeed);

        if (sharedInputValues.getBooleanRisingEdge(clawTopButton)) {
            clawTop = !clawTop;
        }
        if (sharedInputValues.getBooleanRisingEdge(clawBottomButton)) {
            clawBottom = !clawBottom;
        }

        sharedOutputValues.setBoolean("opb_claw_top", clawTop);
        sharedOutputValues.setBoolean("opb_claw_bottom", clawBottom);
        sharedOutputValues.setBoolean("opb_climber_deploy", climberDeploy);
    }

    @Override
    public void dispose() {
        sharedOutputValues.setNumeric("opn_climber", "percent", 0.0);
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
