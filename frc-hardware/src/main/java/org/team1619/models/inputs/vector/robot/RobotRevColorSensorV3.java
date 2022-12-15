package org.team1619.models.inputs.vector.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import org.team1619.models.inputs.vector.ColorSensor;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.utilities.Config;
import com.revrobotics.ColorSensorV3;

import java.util.HashMap;
import java.util.Map;

public class RobotRevColorSensorV3 extends ColorSensor {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private ColorSensorV3 colorSensorV3;

    public RobotRevColorSensorV3(Object name, Config config, HardwareFactory hardwareFactory) {
        super(name, config);
        colorSensorV3 = hardwareFactory.get(ColorSensorV3.class, i2cPort);
    }

    @Override
    public Map<String, Double> getValue() {
        Map<String, Double> colorValues = new HashMap<>();
        if(colorSensorV3.isConnected() ) {
            Color colorSensorColor = colorSensorV3.getColor();

            colorValues.put("red", colorSensorColor.red);
            colorValues.put("green", colorSensorColor.green);
            colorValues.put("blue", colorSensorColor.blue);
            colorValues.put("IR", (double)colorSensorV3.getIR());
            colorValues.put("proximity", (double) colorSensorV3.getProximity());
        } else {
            colorValues.put("red", -1.0);
            colorValues.put("green", -1.0);
            colorValues.put("blue", -1.0);
            colorValues.put("IR", -1.0);
            colorValues.put("proximity", -1.0);
        }

        return colorValues;
    }

}
