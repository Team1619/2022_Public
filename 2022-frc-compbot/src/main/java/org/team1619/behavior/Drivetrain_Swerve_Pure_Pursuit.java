package org.team1619.behavior;

import org.uacr.models.exceptions.ConfigurationException;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.WebDashboardGraphDataset;
import org.uacr.utilities.YamlConfigParser;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.*;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static java.lang.String.format;

/**
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Drivetrain_Swerve_Pure_Pursuit extends BaseSwerve {

    private static final Logger logger = LogManager.getLogger(Drivetrain_Swerve_Pure_Pursuit.class);

    private final Map<String, Path> paths;
    private final Map<String, Pose2d> startingPositions;
    private final Map<String, ValueInterpolator> targetHeadingInterpolators;

    @Nullable
    private Path currentPath;
    @Nullable
    private ValueInterpolator targetHeadingInterpolator;
    @Nullable
    private Pose2d startingPosition;

    private String headingMode;
    private String pathName;

    private boolean isFollowing;

    public Drivetrain_Swerve_Pure_Pursuit(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, robotConfiguration, false);

        paths = new HashMap<>();
        startingPositions = new HashMap<>();
        targetHeadingInterpolators = new HashMap<>();

        YamlConfigParser yamlConfigParser = new YamlConfigParser();
        yamlConfigParser.loadWithFolderName("paths.yaml");

        Map<String, Map<String, Object>> paths = ((Map<String, Map<String, Object>>) yamlConfigParser.getData().get("path"));

        if (paths != null) {
            for (String pathName : paths.keySet()) {
                Config pathConfig = yamlConfigParser.getConfig(pathName);

                ArrayList<Pose2d> waypoints = waypointsFromConfig(pathConfig);
                Path path = new SplinePath(waypoints);
                setupPathUsingConfig(path, pathConfig, yamlConfigParser);
                path.build();

                this.paths.put(pathName, path);

                double heading = 0.0;

                if (pathConfig.contains("start")) {
                    Config startConfig = pathConfig.getSubConfig("start", "position");

                    startingPositions.put(pathName, new Pose2d(startConfig.getDouble("x"), startConfig.getDouble("y"), startConfig.getDouble("heading", 0.0)));

                    if (startConfig.contains("heading")) {
                        heading = startConfig.getDouble("heading");
                    }
                }

                List<ValueInterpolator.ValueDeviation> headingDeviations = new ArrayList<>();

                if (pathConfig.contains("deviations")) {
                    Config deviationsConfig = pathConfig.getSubConfig("deviations", "deviations");

                    for (String deviationKey : deviationsConfig.getData().keySet()) {
                        try {
                            Config deviationConfig = deviationsConfig.getSubConfig(deviationKey, "deviation");

                            double startRamp = 0.0;
                            double endRamp = 0.0;

                            if (deviationConfig.contains("start_ramp")) {
                                startRamp = deviationConfig.getDouble("start_ramp");
                            } else if (deviationConfig.contains("ramp")) {
                                startRamp = deviationConfig.getDouble("ramp");
                            }

                            if (deviationConfig.contains("end_ramp")) {
                                endRamp = deviationConfig.getDouble("end_ramp");
                            } else if (deviationConfig.contains("ramp")) {
                                endRamp = deviationConfig.getDouble("ramp");
                            }

                            PathDeviationConfig deviation = new PathDeviationConfig(deviationConfig.getDouble("start"), deviationConfig.getDouble("end"), startRamp, endRamp);

                            setupDeviation(deviation, deviationConfig, yamlConfigParser);

                            path.addDeviation(deviation);

                            if (deviationConfig.contains("heading")) {
                                headingDeviations.add(new ValueInterpolator.ValueDeviation(deviationConfig.getDouble("heading"), deviationConfig.getDouble("start"), deviationConfig.getDouble("end"), startRamp, endRamp));
                            }
                        } catch (Exception e) {
                            ConfigurationException configurationException = new ConfigurationException("Error configuring \"" + deviationKey + "\" deviation");
                            configurationException.addSuppressed(e);

                            throw configurationException;
                        }
                    }
                }

                targetHeadingInterpolators.put(pathName, new ValueInterpolator(heading, headingDeviations));
            }
        }

        pathName = "Unknown";
        headingMode = "none";

        isFollowing = true;
    }

    @Override
    public void initialize(String stateName, Config config) {
        logger.debug("Entering state {}", stateName);

        stopModules();

        pathName = config.getString("path_name");
        headingMode = config.getString("heading_mode", "imu");

        if (!paths.containsKey(pathName)) {
            throw new RuntimeException("Path \"" + pathName + "\" doesn't exist");
        } else {
            currentPath = paths.get(pathName);
            targetHeadingInterpolator = targetHeadingInterpolators.get(pathName);
        }

        if (startingPositions.containsKey(pathName)) {
            startingPosition = startingPositions.get(pathName);
            sharedInputValues.setVector(odometry + "_new_position", Map.of("x", startingPosition.getX(), "y", startingPosition.getY(), "heading", startingPosition.getHeading()));
        } else {
            startingPosition = null;
        }

        if (null != currentPath) {
            graphPath(stateName, currentPath);
            graphPathWayPoints(stateName, currentPath);
            // graphVelocityProfile(stateName, currentPath);

            currentPath.reset();
        }

        isFollowing = true;
    }

    @Override
    public void update() {
        if (!isFollowing || null == currentPath || null == targetHeadingInterpolator) {
            stopModules();

            return;
        }

        Map<String, Double> odometryValues = sharedInputValues.getVector(odometry);

        // Turns the odometry values into a Pose 2d to pass to path methods
        Pose2d currentPosition = new Pose2d(odometryValues.get("x"), odometryValues.get("y"), odometryValues.get("heading_acc"));

        if (null != startingPosition && startingPosition.distance(currentPosition) > 1) {
            stopModules();

            return;
        }

        startingPosition = null;

        int closest = currentPath.getClosestPointIndex(currentPosition);
        int lookahead = currentPath.getLookAheadPointIndex(currentPosition, closest);

        double targetHeading = currentPath.getPoint(closest).getHeading();
        double headingError = Math.abs(targetHeading - currentPosition.getHeading());

        if (lookahead == -1 && headingError < 5) {
            // The bot HAS arrived at the end of the path, and HAS achieved its target heading
            stopModules();
            isFollowing = false;
            return;
        }

        Vector translationVector;

        if (lookahead == -1) {
            // The bot HAS arrived at the end of the path, but HAS NOT achieved its target heading
            translationVector = new Vector(0, currentPosition.getHeading());
        } else {
            // The bot HAS NOT arrived at the end of the path, and HAS NOT achieved its target heading
            double translationVelocity = currentPath.getPathPointVelocity(closest, currentPosition);
            translationVector = new Vector(currentPath.getPoint(lookahead).subtract(currentPosition)).normalize().scale(translationVelocity).rotate(-currentPosition.getHeading());
        }

        setModulePowers(translationVector, headingMode, targetHeading, "pr_pure_pursuit");
    }

    @Override
    public void dispose() {
        stopModules();
    }

    @Override
    public boolean isDone() {
        return !isFollowing;
    }

    // A path is a list of waypoints, and each waypoint is a list of three numbers: [x, y, heading].
    // The raw data comes from the Yaml config parser as List<List<Object>>.
    // We need to convert this into an ArrayList<Pose2d>.

    private ArrayList<Pose2d> waypointsFromConfig(Config config) {
        List<List<Object>> pathPosesFromConfig = config.getList("path");
        ArrayList<Pose2d> points = new ArrayList<>();

        for (List<Object> cfgPose : pathPosesFromConfig) {
            if (cfgPose.size() < 3)
                throw new ConfigurationException(format("Error parsing path. Found a pose defined with %d values. Each pose must be defined with three values for [x, y, heading]", cfgPose.size()));
            double x = toDouble(cfgPose.get(0));
            double y = toDouble(cfgPose.get(1));
            double heading = toDouble(cfgPose.get(2));
            Pose2d pose2d = new Pose2d(x, y, heading);
            points.add(pose2d);
        }

        return points;
    }

    private double toDouble(Object o) {
        if (o instanceof Double) return (Double) o;
        if (o instanceof Integer) return ((Integer) o).doubleValue();
        throw new NumberFormatException(format("Invalid value found in path pose: %s cannot be converted to a double.", o));
    }

    private void setupPathUsingConfig(Path path, Config config, YamlConfigParser yamlConfigParser) {
        String modelName = config.getString("model", "none");

        if (!modelName.equals("none")) {
            Config model = yamlConfigParser.getConfig(modelName);

            setupPathUsingConfig(path, model, yamlConfigParser);
        }

        path.setPointSpacing(config.getDouble("spacing", path.getPointSpacing()));
        path.setPathSmoothing(config.getDouble("smoothing", path.getPathSmoothing()));
        path.setTurnSpeed(config.getDouble("turn_speed", path.getTurnSpeed()));
        path.setTrackingErrorSpeed(config.getDouble("tracking_error_speed", path.getTrackingErrorSpeed()));
        path.setMaxAcceleration(config.getDouble("max_acceleration", path.getMaxAcceleration()));
        path.setMaxDeceleration(config.getDouble("max_deceleration", path.getMaxDeceleration()));
        path.setMinSpeed(config.getDouble("min_speed", path.getMinSpeed()));
        path.setMaxSpeed(config.getDouble("max_speed", path.getMaxSpeed()));
        path.setLookAheadDistance(config.getDouble("look_ahead_distance", path.getLookAheadDistance()));
        path.setVelocityLookAheadPoints(config.getInt("velocity_look_ahead_points", path.getVelocityLookAheadPoints()));
    }

    private void setupDeviation(PathDeviationConfig deviation, Config config, YamlConfigParser yamlConfigParser) {
        String modelName = config.getString("model", "none");

        if (!modelName.equals("none")) {
            Config model = yamlConfigParser.getConfig(modelName);

            setupDeviation(deviation, model, yamlConfigParser);
        }

        if (config.contains("turn_speed")) {
            deviation.setTurnSpeed(config.getDouble("turn_speed"));
        }
        if (config.contains("max_acceleration")) {
            deviation.setTurnSpeed(config.getDouble("max_acceleration"));
        }
        if (config.contains("max_deceleration")) {
            deviation.setTurnSpeed(config.getDouble("max_deceleration"));
        }
        if (config.contains("min_speed")) {
            deviation.setTurnSpeed(config.getDouble("min_speed"));
        }
        if (config.contains("max_speed")) {
            deviation.setTurnSpeed(config.getDouble("max_speed"));
        }
        if (config.contains("look_ahead_distance")) {
            deviation.setTurnSpeed(config.getDouble("look_ahead_distance"));
        }
        if (config.contains("turn_speed")) {
            deviation.setTurnSpeed(config.getDouble("turn_speed"));
        }
    }

    private void graphPath(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

        for (PathPoint point : path.getPathPoints()) {
            pathGraphDataset.addPoint(point.getX(), point.getY(), point.getHeading());
        }

        sharedInputValues.setVector("gr_" + name, pathGraphDataset);
    }

    private void graphPathWayPoints(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

        for (Pose2d point : path.getWayPoints()) {
            pathGraphDataset.addPoint(point.getX(), point.getY(), point.getHeading());
        }

        sharedInputValues.setVector("gr_" + name + "_waypoints", pathGraphDataset);
    }

    private void graphVelocityProfile(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

        for (PathPoint point : path.getPathPoints()) {
            pathGraphDataset.addPoint(point.getDistance(), point.getVelocity(), point.getHeading());
        }

        sharedInputValues.setVector("gr_" + name + "_velocity", pathGraphDataset);
    }
}
