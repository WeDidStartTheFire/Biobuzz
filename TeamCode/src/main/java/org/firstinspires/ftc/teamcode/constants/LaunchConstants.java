package org.firstinspires.ftc.teamcode.constants;

import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class LaunchConstants {
    public static final double LAUNCHER_HEIGHT = 15.5;
    public static final double LAUNCHER_ANGLE = toRadians(50);
    public static com.pedropathing.control.PIDFCoefficients launcherPIDF = new com.pedropathing.control.PIDFCoefficients(.002, 0, 0, .00055);
    public static PIDFCoefficients launcherReversePIDF = new PIDFCoefficients(80, 0, 0, 20);
    public static double BALL_VEL_TO_MOTOR_VEL_COEFF = 4.45;
    public static double BALL_VEL_TO_MOTOR_VEL_CONST = 466;
}
