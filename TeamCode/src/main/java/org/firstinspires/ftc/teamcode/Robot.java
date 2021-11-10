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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Set;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Robot {
    /* Public Variables */
    public double FRPosition;
    public double FLPosition;
    public double BRPosition;
    public double BLPosition;


    /* Public OpMode members. */
    public DcMotor FRDrive = null;
    public DcMotor FLDrive = null;
    public DcMotor BRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BotArm = null;
    public DcMotor Spin = null;
    public DcMotor Spin2 = null;

    public Servo ArmGrip = null;

    public OpenCvCamera WebCamL = null;

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
        Spin = hwMap.get(DcMotor.class, "Spin");
        Spin2 = hwMap.get(DcMotor.class, "Spin2");
        // Set direction of motors
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BotArm.setDirection(DcMotor.Direction.REVERSE);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin2.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);
        BotArm.setPower(0);
        Spin.setPower(0);
        Spin2.setPower(0);
        // Set all motors to run without encoders.
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        ArmGrip = hwMap.get(Servo.class, "ArmGrip");

        // Define webcams
        int cameraMonitorViewIdL = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamNameL = hwMap.get(WebcamName.class, "webCamL");
        OpenCvCamera webCamL = OpenCvCameraFactory.getInstance().createWebcam(webcamNameL, cameraMonitorViewIdL);
        webCamL.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // start streaming
                webCamL.startStreaming(1280, 720);
            }
            @Override
            public void onError(int errorCode)
            {
                // do something with error
            }
        });
    }

    public void DriveMecanum(double strafe, double drive, double turn, boolean modify) {

        //Calculate needed power
        double modifier = 0.50;
        if (modify) {
            modifier = 0.1;
        } else {
            modifier = 0.5;
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

    public void Arm(double armPower) {

        BotArm.setPower(armPower);

    }

    public void SpinDucks (double duckPower) {

        Spin.setPower(duckPower);
        Spin2.setPower(duckPower);

    }

    public void FlipGrip (double flipperPosition) {

        ArmGrip.setPosition(flipperPosition);

    }

    public void checkPositions () {
        FLPosition = FLDrive.getCurrentPosition();
        FRPosition = FRDrive.getCurrentPosition();
        BLPosition = BLDrive.getCurrentPosition();
        BRPosition = BLDrive.getCurrentPosition();
    }
}