package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Comparator;

public class ScreedVisionProcessor implements VisionProcessor {

    public boolean isBlue = false;
    public int threshold = 160;
    public int blur = 10;

    // Telemetry telemetry;

    // public ScreedVisionPipeline(Telemetry telemetry)
    // {
    //     this.telemetry = telemetry;
    // }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        /* Executed once, when the pipeline is selected */
    }

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    Mat cbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat hierarchy = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    ArrayList<MatOfPoint> contours = new ArrayList<>();
    
    int position = 0;

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        contours = new ArrayList<>();
        
        Rect rect = new Rect(0, input.height()/2, input.width(), input.height()/2);

        input = new Mat (input, rect);

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

        if (contours.toArray().length == 0) return null;

        // Orders list of contours from largest to smallest (finds largest contour)
        contours.sort(new Comparator<MatOfPoint>() {
            public int compare(MatOfPoint c1, MatOfPoint c2) {
                return (int) (Imgproc.contourArea(c2) - Imgproc.contourArea(c1));
            }
        });
        
        Rect bounds = Imgproc.boundingRect(new MatOfPoint(contours.get(0).toArray()));

        // for (int i = 0; i < 3; i++) {
        //     // Approxmiate the number of polylines in the largest contour
        //     MatOfPoint2f m2p = new MatOfPoint2f(contours.get(0).toArray());
        //     Imgproc.approxPolyDP(m2p, m2p, 0.02 * Imgproc.arcLength(m2p, true), true);
        //     // telemetry.addData("Approx Poly Count", m2p.size().height);

        //     if (m2p.size().height <= 4) contours.remove(0);
        //     else break;
        // }

        // Find centroid of the largest area
        double centroidX = Imgproc.moments(contours.get(0)).m10 / Imgproc.moments(contours.get(0)).m00;
        double centroidY = Imgproc.moments(contours.get(0)).m01 / Imgproc.moments(contours.get(0)).m00;

        // Detect position of centroid
        position = 0;
        if (centroidX < (input.width() / 3)) position = -1;
        else if (centroidX > (input.width() * 2 / 3)) position = 1;

        // // telemetry.addData("Position", position);
        // // telemetry.update();


        // // ---- Visiual Guides ----

        // // Split screen into thirds
        // Imgproc.line(input, new Point(input.width() / 3, 0), new Point(input.width() / 3, input.height()), new Scalar(0, 0, 0), 2);
        // Imgproc.line(input, new Point(input.width() * 2 / 3, 0), new Point(input.width() * 2 / 3, input.height()), new Scalar(0, 0, 0), 2);

        // // Draw all contours
        // Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, 8);

        // // Label largest contour with its area
        // Imgproc.putText(input, Imgproc.contourArea(contours.get(0)) + "", new Point(centroidX, centroidY), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 255, 255), 1);
        
        return position;
    }
    
    public void setColor(boolean _isBlue)
    {
        isBlue = _isBlue;
    }
    
    public int getPosition()
    {
        return position;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        
        // canvas.drawRect(makeGraphicsRect((Rect) userContext, scaleBmpPxToCanvasPx), paint);
        canvas.drawText("Position: " + (int) position, 10, 25, paint);
        canvas.drawRect(50, 50, 200, 200, paint);
        
    }
    
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}
