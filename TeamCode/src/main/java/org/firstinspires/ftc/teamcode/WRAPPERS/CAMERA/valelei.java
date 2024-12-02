package org.firstinspires.ftc.teamcode.WRAPPERS.CAMERA;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class valelei {
    public Limelight3A camera;
    public List<LLResultTypes.ColorResult> targetCorners;
    public List<List<Double>> lala;
    public List<Double> corners;
    public double corner1;
    public double corner2;
    public double corner3;
    public double corner4;

    public List<Double> getCorners(){
        targetCorners = camera.getLatestResult().getColorResults();
        lala = targetCorners.get(0).getTargetCorners();
        corners.add(camera.getLatestResult().getColorResults().get(0).getTargetCorners().get(0).get(0));
        corners.add(camera.getLatestResult().getColorResults().get(0).getTargetCorners().get(0).get(1));
        corners.add(camera.getLatestResult().getColorResults().get(0).getTargetCorners().get(1).get(0));
        corners.add(camera.getLatestResult().getColorResults().get(0).getTargetCorners().get(1).get(1));

        return corners;
    }
}
