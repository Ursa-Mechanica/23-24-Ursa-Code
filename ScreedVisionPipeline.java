package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Comparator;

public class GrayscalePipeline extends OpenCvPipeline {

    public boolean doThreshold = true;
    public int threshold = 140;

    Telemetry telemetry;

    public GrayscalePipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat input) {
        /* Executed once, when the pipeline is selected */
    }

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    Mat cbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat hierarchy = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        contours = new ArrayList<>();

        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, 2);

        if (doThreshold) Imgproc.threshold(cbMat, thresholdMat, threshold, 255, Imgproc.THRESH_BINARY);

        Imgproc.erode(thresholdMat, thresholdMat, erodeElement);
        Imgproc.erode(thresholdMat, thresholdMat, erodeElement);

        Imgproc.dilate(thresholdMat, thresholdMat, dilateElement);
        Imgproc.dilate(thresholdMat, thresholdMat, dilateElement); 

        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.sort(new Comparator<MatOfPoint>() {
            public int compare(MatOfPoint c1, MatOfPoint c2) {
                return (int) (Imgproc.contourArea(c2) - Imgproc.contourArea(c1));
            }
        });

        MatOfPoint2f m2p = new MatOfPoint2f(contours.get(0).toArray());
        Imgproc.approxPolyDP(m2p, m2p, 0.02 * Imgproc.arcLength(m2p, true), true);
        telemetry.addData("Approx Poly Count", m2p.size().height);
        telemetry.update();

        Imgproc.putText(input, Imgproc.contourArea(contours.get(0)) + "", new Point((Imgproc.moments(contours.get(0)).m10 / Imgproc.moments(contours.get(0)).m00), (Imgproc.moments(contours.get(0)).m01 / Imgproc.moments(contours.get(0)).m00)), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 255, 255), 1);
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, 8);
        
        return input;
    }

    @Override
    public void onViewportTapped() {
        /*
         * Executed everytime when the pipeline view is tapped/clicked.
         * This is executed from the UI thread, so whatever you do here,
         * it must be done it quickly.
         */
    }

}