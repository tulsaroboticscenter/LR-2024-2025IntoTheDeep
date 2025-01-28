package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.opencv.imgproc.Moments;

public class SampleDetectCustom extends OpenCvPipeline {

    private Telemetry telemetry;
    private Point largestContourCenter = new Point(-1, -1);
    private double largestContourAngle = -1000;

    public SampleDetectCustom(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Define range of yellow in HSV
        Scalar lowerYellow = new Scalar(10, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Threshold the HSV image to get only yellow colors
        Core.inRange(hsv, lowerYellow, upperYellow, mask);

        // Morphological operations to reduce noise
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Initialize variables for the largest contour
        double largestArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 10) { // Minimum area threshold
                Rect boundingRect = Imgproc.boundingRect(contour);
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                if (aspectRatio > 0.3 && aspectRatio < 3.0) { // Aspect ratio for rectangles
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                    }
                }
            }
        }

        if (largestContour != null) {
            // Calculate the center and angle of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            largestContourCenter.x = moments.m10 / moments.m00;
            largestContourCenter.y = moments.m01 / moments.m00;

            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            largestContourAngle = rotatedRect.angle;

            if (rotatedRect.size.width < rotatedRect.size.height) {
                largestContourAngle += 90;
            }

            // Draw the bounding rectangle
            Rect drawRect = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(input, drawRect, new Scalar(0, 255, 255), 2);
        }

        // Display data on telemetry
        telemetry.addData("Center X", largestContourCenter.x);
        telemetry.addData("Center Y", largestContourCenter.y);
        telemetry.addData("Angle", largestContourAngle);
        telemetry.update();

        // Clean up
        hsv.release();
        mask.release();
        kernel.release();
        hierarchy.release();

        return input; // Return the modified frame
    }
}
