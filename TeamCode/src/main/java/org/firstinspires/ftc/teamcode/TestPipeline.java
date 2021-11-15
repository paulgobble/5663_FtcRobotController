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

    final int leftMargin = 140;      // Left margin to be cropped off thresholdImage
    final int righMargin = 140;     // Rign margin to be cropped off
    final int topMargin = 100;       // Top margin to be cropped off
    final int botMargin = 100;      // Bottom margin to be cropped off

    Mat cSpaceShiftedImage = new Mat();     // Matrix to contain the input image after it is converted to LCrCb colorspace
    Mat singleChannelImage = new Mat();   // Matrix to contain just the Cb chanel
    Mat thresholdImage = new Mat();     // Matrix to contain adjusted image
    Mat scanZoneSample = new Mat();

    double scanZoneValue;

    @Override
    public Mat processFrame(Mat input)
    {
        // convert the input RGB image to YCrCb color space
        Imgproc.cvtColor(input, cSpaceShiftedImage, Imgproc.COLOR_RGB2Lab); // was Imgproc.COLOR_RGB2YCrCb
        // extract just the Cb (?) channel to isolate the difference in red
        Core.extractChannel(cSpaceShiftedImage, singleChannelImage, 1);
        // use the threshold Imgproc threshold method to enhance the visual separation between rings and mat floor
        Imgproc.threshold(singleChannelImage, thresholdImage,thresholdValue, MAX_BINARY_VALUE, Imgproc.THRESH_TOZERO);

        // Compute the scan zone rectangle
        final double frameWidth = input.cols();
        final double frameHeight = input.rows();
        Point upperLeft = new Point(leftMargin, topMargin);
        Point lowerRight = new Point(frameWidth - righMargin, frameHeight - botMargin);
        Rect scanZoneRect = new Rect(upperLeft, lowerRight);

        int zoneWidth = scanZoneRect.width;
        int zoneHeight = scanZoneRect.height;

        // create a submat containing jsut the cropped scan zone
        scanZoneSample = thresholdImage.submat(new Rect(leftMargin, righMargin, zoneWidth, zoneHeight));

        // convert the MAT scanZoneSample into a single value representing its brightess
        scanZoneValue = Core.mean(scanZoneSample).val[0];

        Imgproc.rectangle(
                thresholdImage,
                upperLeft,
                lowerRight,
                new Scalar(255, 0, 0), 1); // t = 120

        return thresholdImage;
    }
}