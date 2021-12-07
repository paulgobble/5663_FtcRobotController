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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


public class Robot {
    /* Public Variables */

    // Arm Encoder Positions for Shipping Hub Levels
    public final int level_1_position = 7400; // lowest level - was 7500, then 7250, then 7350
    public final int level_2_position = 6950; // middle level - was 5800
    public final int level_3_position = 6300; // highest level - was 5000 then 6300

    //  Pipeline Results *
    public Boolean leftCameraFoundTSE = false;     // Was the TSE found on the left side of the webcam frame
    public Boolean rightCameraFoundTSE = false;    // Was the TSE found on the right side of the webcam frame
    public Boolean visionScanComplete = false;     // has the pipeline completed a scan


    /* Public OpMode members. */
    public DcMotor FRDrive = null;
    public DcMotor FLDrive = null;
    public DcMotor BRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BotArm = null;
    public DcMotor BotArm2 = null;
    public DcMotor Spin = null;
    public DcMotor Spin2 = null;

    public Servo ArmGrip = null;

    public ColorSensor LineStopper = null;

    public OpenCvCamera WebCamC = null;

    enum driveModes {
        Standard,
        Flipped
    }
    public driveModes currentDriveMode = driveModes.Standard;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");
        BotArm = hwMap.get(DcMotor.class, "BotArm");
        BotArm2 = hwMap.get(DcMotor.class, "BotArm2");
        Spin = hwMap.get(DcMotor.class, "Spin");
        Spin2 = hwMap.get(DcMotor.class, "Spin2");
        // Set direction of motors
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BotArm.setDirection(DcMotor.Direction.REVERSE);
        BotArm2.setDirection(DcMotor.Direction.FORWARD);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin2.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);
        BotArm.setPower(0);
        BotArm2.setPower(0);
        Spin.setPower(0);
        Spin2.setPower(0);
        // Set all motors to run without encoders.
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BotArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set the zero power mode to brake for arm and spin motors
        BotArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BotArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        ArmGrip = hwMap.get(Servo.class, "ArmGrip");

        LineStopper = hwMap.get(ColorSensor.class,"lineStopper");

        // Define webcams
        // Step 1. Get live viewport
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        // Step 2. Create a webcam instance
        WebcamName webcamName = hwMap.get(WebcamName.class, "webCamCenter");
        OpenCvCamera WebCamC = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Step 3. Open the Camera Device
        WebCamC.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                WebCamC.setPipeline(new RedPipeline_internal());
                WebCamC.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void DriveMecanum(double strafe, double drive, double turn, boolean Creep, boolean Turbo) {

        //Calculate needed power
        double modifier = 0.50;

        if (Creep) {
            modifier = 0.1;
        } else if (Turbo){
            modifier = 1;
        }

        if (currentDriveMode == driveModes.Flipped) {

            modifier *= -1;

        }

        double FRPower = -strafe + drive - turn;
        double FLPower = strafe + drive + turn;
        double BRPower = strafe + drive - turn;
        double BLPower = -strafe + drive + turn;

        //Ensure that power does not fo over 1
        double maxPower = Math.max(FRPower, Math.max(FLPower, Math.max(BRPower, BLPower)));

        if (maxPower > 1) {
            FRPower = FRPower / maxPower;
            FLPower = FLPower / maxPower;
            BRPower = BRPower / maxPower;
            BLPower = BLPower / maxPower;
        }
        //Apply the power to the wheels
        FRDrive.setPower(FRPower * modifier);
        FLDrive.setPower(FLPower * modifier);
        BRDrive.setPower(BRPower * modifier);
        BLDrive.setPower(BLPower * modifier);
    }

    /* moving encoderDrive from Auton to Robot */
    /* removing all time checks and telemetry  */
    public void encoderDrive(double speed, double FLInches, double FRInches, double BLInches, double BRInches)
    {
        //Set up variables for erncoders to wheel distance.
        final double     COUNTS_PER_MOTOR_REV    = 383.6 ;  //GoBilda Motor 28 counts per motor rev (28*13.7=383.6)
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
        final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        //Create targets for motors.
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Ensure that the opmode is still active.
        //if (opModeIsActive()) {

            //Get new target positions.
            newFLTarget = FLDrive.getCurrentPosition() + (int)(FLInches * COUNTS_PER_INCH);
            newFRTarget = FRDrive.getCurrentPosition() + (int)(FRInches * COUNTS_PER_INCH);
            newBLTarget = BLDrive.getCurrentPosition() + (int)(BLInches * COUNTS_PER_INCH);
            newBRTarget = BRDrive.getCurrentPosition() + (int)(BRInches * COUNTS_PER_INCH);

            //Set the new target positions.
            FLDrive.setTargetPosition(newFLTarget);
            FRDrive.setTargetPosition(newFRTarget);
            BLDrive.setTargetPosition(newBLTarget);
            BRDrive.setTargetPosition(newBRTarget);

            //Set mode to run to position.
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Reset time.
            //segmentTime.reset();

            //Apply power to motors.
            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed));
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed));

            //Detect whether or not the robot is running.
            while (FLDrive.isBusy() && FRDrive.isBusy()&& BLDrive.isBusy()&& BRDrive.isBusy()) {

                //Telemetry

            }

            //Stop motors.
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            //Turn off RUN_TO_POSITION.
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    } //encoderDrive



    public void Arm(double armPower) {

        BotArm.setPower(armPower);
        BotArm2.setPower(armPower);

    } // End Arm

    public void SpinDucks (double duckPower) {

        Spin.setPower(duckPower);
        Spin2.setPower(duckPower);

    } // End SpinDucks

    public void FlipGrip (double flipperPosition) {

        ArmGrip.setPosition(flipperPosition);

    } // End FlipGrip


    // SETTER - Toggle the value of forwardDriveMode
    public void setForwardDriveMode () {
        if (currentDriveMode == driveModes.Standard) {
            currentDriveMode = driveModes.Flipped;
        } else {
            currentDriveMode = driveModes.Standard;
        }
    }







    /*************************************
     *  Nested EasyOpenCV pipeline class *
     *************************************/
    class RedPipeline_internal extends OpenCvPipeline
    {
        final double thresholdValue = 120;
        final double MAX_BINARY_VALUE = 255;

        final int leftMargin = 145;      // Left margin to be cropped off thresholdImage
        final int righMargin = 145;     // Rign margin to be cropped off
        final int topMargin = 100;       // Top margin to be cropped off
        final int botMargin = 100;      // Bottom margin to be cropped off

        Mat workingFrame = new Mat();       // one Matrix to reuse
        //Mat cSpaceShiftedImage = new Mat();     // Matrix to contain the input image after it is converted to LCrCb colorspace
        //Mat singleChannelImage = new Mat();   // Matrix to contain just the Cb chanel
        //Mat thresholdImage = new Mat();     // Matrix to contain adjusted image
        //Mat scanZoneSample = new Mat();

        //double scanZoneValue = 0;

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

            double testThreshold = 10; // max value to test as TSE green
            int leftScanZoneLimit = 4;      // zone furthest to right to be considered a left scan zone
            int rightScanZoneLimit = 10;    // zone furthest to right to be considered a right scan zone

            double thisScanValue;
            boolean foundTSE;

            int scanLoopCounter = 0;

            if (!visionScanComplete) {

                Mat scanZoneSample = new Mat();

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
                        } else if (scanLoopCounter <= rightScanZoneLimit){
                            rightCameraFoundTSE = true;
                        }
                    }
                    // increment the counter
                    scanLoopCounter += 1;
                }
                visionScanComplete = true;

                scanZoneSample.release();
            }

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

    } // End class RedPipeline_internal

} // End Robot.class