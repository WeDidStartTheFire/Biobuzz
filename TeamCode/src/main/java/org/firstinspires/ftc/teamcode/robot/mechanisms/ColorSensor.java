package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.enums.Hardware.COLOR_SENSOR_A;
import static org.firstinspires.ftc.teamcode.enums.Hardware.COLOR_SENSOR_B;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;
import org.opencv.core.Scalar;

public class ColorSensor {

    private final @Nullable RevColorSensorV3 colorSensorA, colorSensorB;
    private @Nullable Scalar colorA, colorB;
    private boolean aLast;

    public ColorSensor(HardwareMap hardwareMap, TelemetryUtils tm) {
        colorSensorA = HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_A);
        colorSensorB = HardwareInitializer.init(hardwareMap, tm, COLOR_SENSOR_B);
        setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    /**
     * Sets the bus speed for the color sensors
     *
     * @param busSpeed Bus speed to set the color sensors to
     */
    public void setBusSpeed(LynxI2cDeviceSynch.BusSpeed busSpeed) {
        if (colorSensorA != null)
            ((LynxI2cDeviceSynch) colorSensorA.getDeviceClient()).setBusSpeed(busSpeed);
        if (colorSensorB != null)
            ((LynxI2cDeviceSynch) colorSensorB.getDeviceClient()).setBusSpeed(busSpeed);
    }

    /**
     * Gets the averaged normalized RGB values from the color sensors
     *
     * @return The normalized RGB values, Scalar(R, G, B)
     */
    public Scalar getRGB(boolean bothSensors) {
        colorA = bothSensors || !aLast ? getRGB_A() : colorA;
        colorB = bothSensors || aLast ? getRGB_B() : colorB;
        aLast = !aLast;
        if (colorA != null && colorB != null)
            return new Scalar((colorA.val[0] + colorB.val[0]) / 2, (colorA.val[1] + colorB.val[1]) / 2, (colorA.val[2] + colorB.val[2]) / 2);
        return colorA != null ? colorA : colorB;
    }

    /**
     * Gets the normalized RGB values from color sensor A
     *
     * @return The normalized RGB values, Scalar(R, G, B)
     */
    @NonNull
    public Scalar getRGB_A() {
        if (colorSensorA == null) return new Scalar(0, 0, 0);
        NormalizedRGBA color = colorSensorA.getNormalizedColors();
        if (color.alpha == 0) return new Scalar(color.red, color.green, color.blue);
        float r = color.red / color.alpha;
        float g = color.green / color.alpha;
        float b = color.blue / color.alpha;
        return new Scalar(r, g, b);
    }

    /**
     * Gets the normalized RGB values from color sensor B
     *
     * @return The normalized RGB values, Scalar(R, G, B)
     */
    @NonNull
    public Scalar getRGB_B() {
        if (colorSensorB == null) return new Scalar(0, 0, 0);
        NormalizedRGBA color = colorSensorB.getNormalizedColors();
        if (color.alpha == 0) return new Scalar(color.red, color.green, color.blue);
        float r = color.red / color.alpha;
        float g = color.green / color.alpha;
        float b = color.blue / color.alpha;
        return new Scalar(r, g, b);
    }

    /**
     * Gets the distance reading from color sensor A
     *
     * @return Distance (double) or -1 if the color sensor is disconnected
     */
    public double getInchesA() {
        return colorSensorA == null ? -1 : colorSensorA.getDistance(DistanceUnit.INCH);
    }

    /**
     * Gets the distance reading from color sensor B
     *
     * @return Distance (double) or -1 if the color sensor is disconnected
     */
    public double getInchesB() {
        return colorSensorB == null ? -1 : colorSensorB.getDistance(DistanceUnit.INCH);
    }
}
