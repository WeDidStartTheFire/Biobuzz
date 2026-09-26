package org.firstinspires.ftc.teamcode.robot.mechanisms;

import static org.firstinspires.ftc.teamcode.enums.Hardware.LIMELIGHT;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TelemetryUtils;
import org.firstinspires.ftc.teamcode.enums.HivePosition;
import org.firstinspires.ftc.teamcode.robot.HardwareInitializer;

import java.util.List;

public class Limelight {

    private final @Nullable Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, TelemetryUtils tm) {
        limelight = HardwareInitializer.init(hardwareMap, tm, LIMELIGHT);
        if (limelight != null) limelight.pipelineSwitch(0);
    }

    /**
     * Starts or resumes periodic polling of Limelight data.
     */
    public void start() {
        if (limelight != null) limelight.start();
    }

    /**
     * Stops polling of Limelight data.
     */
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /**
     * @return The fiducial readings from the limelight
     */
    public @Nullable List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getFiducialResults();
    }

    public @Nullable LLResult getLatestResult() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    public double getAverageHiveY() {
        double total = 0;
        int number = 0;

        if (limelight != null && getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : getFiducials()) {
                number++;
                total += fiducial.getTargetPoseRobotSpace().getPosition().y;
            }
        }

        return total / number;
    }

    public HivePosition getHivePosition() {
        if (limelight != null && getFiducials() != null) {
            for (LLResultTypes.FiducialResult fiducial : getFiducials()) {
                if (getAverageHiveY() <= -1.40 && fiducial.getFiducialId() == 21)
                    return HivePosition.STAGE;
                else if (getAverageHiveY() >= -1.1 &&
                    fiducial.getFiducialId() == 21) return HivePosition.AUDIENCE;
                else return HivePosition.TRANSITIONING;
            }
        }
        return HivePosition.UNKNOWN;
    }


}
