package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class TestPipeline extends OpenCvPipeline
{
    final double thresholdValue = 120;
    final double MAX_BINARY_VALUE = 255;

    final int topMargin = 100;          // Top margin to be cropped off
    final int botMargin = 100;          // Bottom margin to be cropped off
    final int leftMargin = 50;          // Left margin to be cropped off
    final int righMargin = 50;          // Right margin to be cropped off
    final int leftRightOffset = 0;    // margin adjustment to mask out unwanted side of camera view
    int adjustedLeftMargin;
    int adjustedRightMargin;

    Mat rotatedRightImage = new Mat();      // the right camear is upside down, so we need to flio thie image
    Mat cSpaceShiftedImage = new Mat();     // Matrix to contain the input image after it is converted to LCrCb colorspace
    Mat singleChannelImage = new Mat();     // Matrix to contain just the Cb chanel
    Mat thresholdImage = new Mat();         // Matrix to contain adjusted image
    Mat scanZoneSample = new Mat();

    enum redCamers {
        theLeftOne,
        theRightOne
    }

    private redCamers chosenCamera = redCamers.theRightOne;

    private double scanZoneValue;

    /***********************************************************
     * This method is taken from the pipeline documentation.   *
     * It serves no essential function, I just hope it smooths *
     * out an initial perceived hickup where a new pipeline    *
     * doesn't appear to kick in right away                    *
     * *********************************************************/
    @Override
    public void init(Mat firstFrame)
    {
        // Compute the scan zone rectangle
        final double frameWidth = firstFrame.cols();
        final double frameHeight = firstFrame.rows();
        Point upperLeft = new Point(leftMargin, topMargin);
        Point lowerRight = new Point(frameWidth - righMargin, frameHeight - botMargin);
        Rect scanZoneRect = new Rect(upperLeft, lowerRight);

        scanZoneSample = firstFrame.submat(scanZoneRect);
    } // End init


    @Override
    public Mat processFrame(Mat input)
    {
        // rotate frames from the right camera
        if (chosenCamera == redCamers.theRightOne) {
            Core.rotate(input, rotatedRightImage, Core.ROTATE_180);
            // convert the input RGB image to LAB color space
            Imgproc.cvtColor(rotatedRightImage, cSpaceShiftedImage, Imgproc.COLOR_RGB2Lab);
            // Adjust margins to mask out the left side of frame
            int adjustedLeftMargin = 100; //leftMargin + leftRightOffset;
            int adjustedRightMargin = 100; //righMargin;
        } else if (chosenCamera == redCamers.theLeftOne) {
            // convert the input RGB image to LAB color space
            Imgproc.cvtColor(input, cSpaceShiftedImage, Imgproc.COLOR_RGB2Lab);
            // Adjust margins to mask out the right side of frame
            int adjustedLeftMargin = 50; //leftMargin;
            int adjustedRightMargin = 150; //righMargin + leftRightOffset;
        }
        // extract just the A channel to isolate the difference in green
        Core.extractChannel(cSpaceShiftedImage, singleChannelImage, 1);
        // use the threshold Imgproc threshold method to enhance the visual separation between rings and mat floor
        Imgproc.threshold(singleChannelImage, thresholdImage,thresholdValue, MAX_BINARY_VALUE, Imgproc.THRESH_TOZERO);

        // Compute the scan zone rectangle
        final double frameWidth = input.cols();
        final double frameHeight = input.rows();
        Point upperLeft = new Point(adjustedLeftMargin, topMargin);
        Point lowerRight = new Point(frameWidth - adjustedRightMargin, frameHeight - botMargin);
        Rect scanZoneRect = new Rect(upperLeft, lowerRight);

        int zoneWidth = scanZoneRect.width;
        int zoneHeight = scanZoneRect.height;

        // create a submat containing just the cropped scan zone
        scanZoneSample = thresholdImage.submat(scanZoneRect);//scanZoneSample = thresholdImage.submat(new Rect(leftMargin, righMargin, zoneWidth, zoneHeight));
        // convert the MAT scanZoneSample into a single value representing its brightess
        scanZoneValue = Core.mean(scanZoneSample).val[0];

        Imgproc.rectangle(
                thresholdImage,
                upperLeft,
                lowerRight,
                new Scalar(255, 0, 0), 4);

        return thresholdImage;

    } // End processFrame


    public void selectCamera(redCamers thisCamera) {

        chosenCamera = thisCamera;

    } // End selectCamera


    public double getScanZoneValue() {

        return scanZoneValue;

    } // End getScanZoneValue


} // End class testPipeline