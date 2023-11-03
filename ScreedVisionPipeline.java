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

public class ScreedVisionPipeline extends OpenCvPipeline {

    public boolean isBlue = true;
    public int threshold = 140;
    public int blur = 10;

    Telemetry telemetry;

    public ScreedVisionPipeline(Telemetry telemetry)
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

        // Blur the image
        Imgproc.blur(input, input, new Size(blur, blur), new Point(-1, -1));

        // Isolate red/blue channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, ((isBlue) ? 2 : 1));

        // Apply threshold value to isolate object
        Imgproc.threshold(cbMat, thresholdMat, threshold, 255, Imgproc.THRESH_BINARY);

        // Erode and dilate filtered image
        Imgproc.erode(thresholdMat, thresholdMat, erodeElement);
        Imgproc.erode(thresholdMat, thresholdMat, erodeElement);
        Imgproc.dilate(thresholdMat, thresholdMat, dilateElement);
        Imgproc.dilate(thresholdMat, thresholdMat, dilateElement); 

        // Find contours around all objects
        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Orders list of contours from largest to smallest (finds largest contour)
        contours.sort(new Comparator<MatOfPoint>() {
            public int compare(MatOfPoint c1, MatOfPoint c2) {
                return (int) (Imgproc.contourArea(c2) - Imgproc.contourArea(c1));
            }
        });

        // Approxmiate the number of polylines in the largest contour
        MatOfPoint2f m2p = new MatOfPoint2f(contours.get(0).toArray());
        Imgproc.approxPolyDP(m2p, m2p, 0.02 * Imgproc.arcLength(m2p, true), true);
        telemetry.addData("Approx Poly Count", m2p.size().height);

        // Find centroid of the largest area
        double centroidX = Imgproc.moments(contours.get(0)).m10 / Imgproc.moments(contours.get(0)).m00;
        double centroidY = Imgproc.moments(contours.get(0)).m01 / Imgproc.moments(contours.get(0)).m00;

        // Detect position of centroid
        // TODO: Make this cleaner
        int position = 0;
        if (centroidX < (input.width() / 3)) position = -1;
        else if (centroidX > (input.width() * 2 / 3)) position = 1;

        telemetry.addData("Position", position);
        telemetry.update();


        // ---- Visiual Guides ----

        // Split screen into thirds
        Imgproc.line(input, new Point(input.width() / 3, 0), new Point(input.width() / 3, input.height()), new Scalar(0, 0, 0), 2);
        Imgproc.line(input, new Point(input.width() * 2 / 3, 0), new Point(input.width() * 2 / 3, input.height()), new Scalar(0, 0, 0), 2);

        // Draw all contours
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, 8);

        // Label largest contour with its area
        Imgproc.putText(input, Imgproc.contourArea(contours.get(0)) + "", new Point(centroidX, centroidY), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 255, 255), 1);
        
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
