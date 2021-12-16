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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Teleop", group="Iterative Opmode")
//@Disabled
public class Teleop extends OpMode {
    // Declare OpMode members.

    Robot robot = new Robot();

    /* Rate limit gamepad button presses to every 500ms. */
    private final static int ButtonLockout = 500;

    private Deadline buttonPressLimit;

    private Deadline duckSpinSlowLimit;
    private Deadline duckSpinMediumLimit;


    private ElapsedTime runtime = new ElapsedTime();
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        buttonPressLimit = new Deadline(ButtonLockout, TimeUnit.MILLISECONDS);

        duckSpinSlowLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        duckSpinMediumLimit = new Deadline(1250, TimeUnit.MILLISECONDS);

        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        robot.DriveMecanum(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger > 0, gamepad1.right_bumper);


        // Spin D-pod control
        boolean spinLeft = gamepad2.dpad_left;
        boolean spinRight = gamepad2.dpad_right;
        double duckSpeed = .75; // was .6

        //double duck_spin_slow_speed = .25;
        //int duck_spin_slow_time = 500;
        //double duck_spin_medium_speed = .75;
        //int duck_spin_medium_time = 1250;
        //double duck_spin_fast_speed = 1;
        //int duck_spin_fast_time = 2000;

        //duckSpinSlowLimit.reset();
        //duckSpinMediumLimit.reset();
        if (spinLeft) {
            //spin duck left
            robot.SpinDucks(duckSpeed);
        } else if (spinRight) {
            // Spion duck right
            robot.SpinDucks(duckSpeed * -1);
        } else {
            // do nothing
            robot.SpinDucks(0);
        }

        // BotArm D-pod control
        boolean armUp = gamepad2.dpad_up;
        boolean armDown = gamepad2.dpad_down;

        boolean armCreep = gamepad2.right_bumper;

        double armSpeed = 1;

        if (armCreep) {
            armSpeed = .3;
            telemetry.addData("Arm Speed", "Creep");
        } else {
            telemetry.addData("Arm Speed", "Normal");
        }


        if (armUp && !spinLeft && !spinRight) {  // disable arm if spin engaged. Hoping to solve problem with sloppy d pad pressing spin and arm together
            //raise the wing
            robot.Arm(armSpeed);
        } else if (armDown && !spinLeft && !spinRight) {
            // lower the wing
            robot.Arm(armSpeed * -1);
        } else {
            // do nothing
            robot.Arm(0);
        }


        // Flipper Gripper Trigger control
        boolean openGrip = gamepad2.left_trigger > 0;  // these assignments seam backwards, but they work
        boolean closeGrip = gamepad2.right_trigger > 0;
        boolean openWide = gamepad2.left_bumper;
        telemetry.addData("Open Grip", openGrip);
        telemetry.addData("CloseGrip", closeGrip);
        if (openGrip) {
            robot.FlipGrip(robot.grip_open);
        } else if (closeGrip) {
            robot.FlipGrip(robot.grip_tight);
        } else if (openWide) {
            robot.FlipGrip(robot.grip_wide);
        }

        handleButtons();  // test for pressed toggle buttons


        // Show the elapsed game time and wheel power.
        telemetry.addData("      Status", "Run Time: " + runtime.toString());
        telemetry.addData("     FRDrive", robot.FRDrive.getCurrentPosition());
        telemetry.addData("     FLDrive", robot.FLDrive.getCurrentPosition());
        telemetry.addData("     BRDrive", robot.BRDrive.getCurrentPosition());
        telemetry.addData("     BLDrive", robot.BLDrive.getCurrentPosition());
        telemetry.addData("       Creep", gamepad1.right_trigger > 0);
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Turbo", gamepad1.right_bumper);
        telemetry.addData( "Color red", robot.LineStopper.red());
        telemetry.addData( "Color blue", robot.LineStopper.blue());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    /*******************************************
     * Method to handle gamepad toggle buttons *
     * presses and debounce                    *
     *                                         *
     * Im not really sure if this does anything anymore  *
     *******************************************/
    private void handleButtons () {

        // check if we've waited long enough
        if (!buttonPressLimit.hasExpired()) {
            return;
        }

        boolean openGrip = gamepad2.left_trigger > 0;

        // Set Robot Options
        if (gamepad1.left_trigger > 0) {
            //robot.setForwardDriveMode();  - pulling plug on the Flip button
            buttonPressLimit.reset();
        }

    } // End handleButtons

} // End Class