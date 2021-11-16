/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name="CamTest_Back_n_Forth", group="Camera Utilities")
//@Disabled
public class CamTest_Back_n_Forth extends OpMode {
    WebcamName WebCamL;
    WebcamName WebCamR;
    String currentCamera = "Left";
    double currentCameraValue = 0;
    OpenCvSwitchableWebcam switchableWebcam;

    public int leftMargin = 140;      // Left margin to be cropped off thresholdImage
    public int righMargin = 140;     // Rign margin to be cropped off
    public int topMargin = 100;       // Top margin to be cropped off
    public int botMargin = 100;      // Bottom margin to be cropped off

    //Robot robot = new Robot();

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //robot.init(hardwareMap);

        WebCamL = hardwareMap.get(WebcamName.class, "webCamL");
        WebCamR = hardwareMap.get(WebcamName.class, "webCamR");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, WebCamL, WebCamR);

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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    } // end init



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    } // End init_loop



    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    } // End start



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Current Camera Name ", currentCamera);
        telemetry.addData("Current Camera Value", currentCameraValue);
        telemetry.addData("Runtime", runtime);

        if (runtime.seconds() > 3) {
            runtime.reset();
            toggleCameras();
        }

    } // End loop



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    } // End stop


    public void toggleCameras(){

        if (currentCamera.equals("Left")){
            currentCamera = "Right";
        } else {
            currentCamera = "Left";
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
                leftMargin = 110;
                righMargin = 150;
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


} // End Class CamTest_Back_n_Forth