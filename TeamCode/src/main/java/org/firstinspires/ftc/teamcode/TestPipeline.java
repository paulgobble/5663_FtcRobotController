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
    private boolean leftCameraFoundTSE = false;
    private boolean rightCameraFoundTSE = false;

    public double currentScanValue = 9999;
    public int lastZoneScanned = 99;

    final double thresholdValue = 115; //was 120
    final double MAX_BINARY_VALUE = 235; // was 255

    final int leftMargin = 145;      // Left margin to be cropped off thresholdImage
    final int righMargin = 145;     // Rign margin to be cropped off
    final int topMargin = 100;       // Top margin to be cropped off
    final int botMargin = 100;      // Bottom margin to be cropped off

    Mat workingFrame = new Mat();       // one Matrix to reuse

    @Override
    public Mat processFrame(Mat input)
    {
        /**********************
         * Process the frame  *
         **********************/
        // convert the input RGB image to LAB color space
        Imgproc.cvtColor(input, workingFrame, Imgproc.COLOR_RGB2Lab); // was Imgproc.COLOR_RGB2YCrCb
        // extract just the A channel to isolate the difference in green
        Core.extractChannel(workingFrame, workingFrame, 1);
        // use the threshold Imgproc threshold method to enhance the visual separation between rings and mat floor
        Imgproc.threshold(workingFrame, workingFrame,thresholdValue, MAX_BINARY_VALUE, Imgproc.THRESH_TOZERO);


        /*****************************
         * Loop over the target area *
         * taking samples            *
         *****************************/
        final int frameWidth = input.cols();
        final int frameHeight = input.rows();

        int scanWidth = 50; // was 30
        int scanHeight = frameHeight - topMargin - botMargin;
        int leftScanPadding = 0;
        int rightScanPadding = 0;
        int scanStep = 25;

        double testThreshold = 4; // max value to test as TSE green - was 10
        int leftScanZoneLimit = 4;      // zone furthest to right to be considered a left scan zone
        int rightScanZoneLimit = 10;    // zone furthest to right to be considered a right scan zone

        double thisScanValue;
        boolean foundTSE;

        int scanLoopCounter = 0;

        Mat scanZoneSample = new Mat();

        outerloop:
        for (int thisX = leftScanPadding; thisX < frameWidth - scanWidth - rightScanPadding; thisX = thisX + scanStep) {

            // copy just target zone to a new matrix
            scanZoneSample = workingFrame.submat(new Rect(thisX, topMargin, scanWidth, scanHeight));
            // convert the matrix single color channel averaged numeric value
            thisScanValue = Core.mean(scanZoneSample).val[0];
            // decide if this value indicates presence of the TSE
            foundTSE = (thisScanValue <= testThreshold);
            // If a TSE is found, determine where it was seen and pass it back to robot
            if (foundTSE){
                if(scanLoopCounter <= leftScanZoneLimit) {
                    leftCameraFoundTSE = true;
                    rightCameraFoundTSE = false;
                    currentScanValue = thisScanValue;
                    break outerloop;
                } else if (scanLoopCounter <= rightScanZoneLimit){
                    leftCameraFoundTSE = false;
                    rightCameraFoundTSE = true;
                    currentScanValue = thisScanValue;
                    break outerloop;
                }
            } else {
                leftCameraFoundTSE = false;
                rightCameraFoundTSE = false;
                currentScanValue = 7777;
            }
            lastZoneScanned = scanLoopCounter;
            // increment the counter
            scanLoopCounter += 1;
        } // end for

        scanZoneSample.release();

        // Compute the scan zone rectangle
        Point upperLeft = new Point(leftMargin, topMargin);
        Point lowerRight = new Point(frameWidth - righMargin, frameHeight - botMargin);
        Rect scanZoneRect = new Rect(upperLeft, lowerRight);

        Imgproc.rectangle(
                workingFrame,
                upperLeft,
                lowerRight,
                new Scalar(255, 0, 0), 1); //

        return workingFrame;
    } // End processFrame



    public boolean didLeftCameraFindTSE() {
        return leftCameraFoundTSE;
    } // End didLeftCameraFindTSE



    public boolean didRightCameraFindTSE() {
        return rightCameraFoundTSE;
    } // End didRightCameraFindTSE


} // End class testPipeline