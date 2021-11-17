/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name="Cam Test - Switchable", group="Camera Test")
@Disabled
public class SwitchableWebcamExample extends LinearOpMode
{
    //TestPipeline redPipeline = new TestPipeline();

    WebcamName WebCamL;
    WebcamName WebCamR;
    String currentCamera = "Right";
    double currentCameraValue = 0;
    OpenCvSwitchableWebcam switchableWebcam;

    public int leftMargin = 140;      // Left margin to be cropped off thresholdImage
    public int righMargin = 140;     // Rign margin to be cropped off
    public int topMargin = 100;       // Top margin to be cropped off
    public int botMargin = 100;      // Bottom margin to be cropped off

    @Override
    public void runOpMode() throws InterruptedException
    {
        WebCamL = hardwareMap.get(WebcamName.class, "webCamL");
        WebCamR = hardwareMap.get(WebcamName.class, "webCamR");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, WebCamR, WebCamL);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                switchableWebcam.setPipeline(new TestPipelineInternal());

                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine("PRESS A/B TO SWITCH CAMERA\n");
            telemetry.addData("Current Camera Name ", currentCamera);
            telemetry.addData("Current Camera Value", currentCameraValue);
            telemetry.update();

            if(gamepad1.a)
            {
                switchableWebcam.setActiveCamera(WebCamL);
                currentCamera = "Left";
            }
            else if(gamepad1.b)
            {
                switchableWebcam.setActiveCamera(WebCamR);
                currentCamera = "Right";
            }

            sleep(100);

            /*while (true){
                redPipeline.selectCamera(TestPipeline.redCamers.theLeftOne);
                telemetry.addData("Current Camera", "Left");
                telemetry.addData("Zone sample value", redPipeline.getScanZoneValue());
                telemetry.addData("AdjustedLeftMargin", redPipeline.adjustedLeftMargin);
                telemetry.addData("AdjustedRightMargin", redPipeline.adjustedRightMargin);
                telemetry.update();
                sleep(3000);
                break;
            }

            while (true){
                redPipeline.selectCamera(TestPipeline.redCamers.theRightOne);
                telemetry.addData("Current Camera", "Right");
                telemetry.addData("Zone sample value", redPipeline.getScanZoneValue());
                telemetry.addData("AdjustedLeftMargin", redPipeline.adjustedLeftMargin);
                telemetry.addData("AdjustedRightMargin", redPipeline.adjustedRightMargin);
                telemetry.update();
                sleep(3000);
                break;
            }*/
        }
    }

    class TestPipelineInternal extends OpenCvPipeline
    {
        final double thresholdValue = 120;
        final double MAX_BINARY_VALUE = 255;

        /*final int leftMargin = 140;      // Left margin to be cropped off thresholdImage
        final int righMargin = 140;     // Rign margin to be cropped off
        final int topMargin = 100;       // Top margin to be cropped off
        final int botMargin = 100;      // Bottom margin to be cropped off*/

        Mat cSpaceShiftedImage = new Mat();     // Matrix to contain the input image after it is converted to LCrCb colorspace
        Mat singleChannelImage = new Mat();   // Matrix to contain just the Cb chanel
        Mat rotateRightCameraImage = new Mat(); // since the right camera is upsidedown, lets rotate the image 180
        Mat thresholdImage = new Mat();     // Matrix to contain adjusted image
        Mat scanZoneSample = new Mat();

        //double scanZoneValue = 0;

        @Override
        public Mat processFrame(Mat input)
        {
            if (currentCamera.equals("Left")) {
                leftMargin = 70;
                righMargin = 140;
            } else if (currentCamera.equals("Right")) {
                leftMargin = 140;
                righMargin = 70;
            }

            // convert the input RGB image to YCrCb color space
            Imgproc.cvtColor(input, cSpaceShiftedImage, Imgproc.COLOR_RGB2Lab); // was Imgproc.COLOR_RGB2YCrCb
            // extract just the Cb (?) channel to isolate the difference in red
            Core.extractChannel(cSpaceShiftedImage, singleChannelImage, 1);

            // Rotate image from right camera
            //if (currentCamera.equals("Right")) {
            //    Core.rotate(singleChannelImage, rotateRightCameraImage, Core.ROTATE_180);
            //}

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
            currentCameraValue = Core.mean(scanZoneSample).val[0];

            Imgproc.rectangle(
                    thresholdImage,
                    upperLeft,
                    lowerRight,
                    new Scalar(255, 0, 0), 1); //

            return thresholdImage;
        }
    }
}