// Version 2.0

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auton_Blue_Ducks", group="Autonomous")
//@Disabled
public class Auton_Blue_Ducks extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();

    /* Setup Camera */
    public OpenCvCamera WebCamC = null;
    TestPipeline pipeline;

    enum hubLevels {
        One,  // bottom
        Two,  // middle
        Three // top
    }

    hubLevels targetLevel = hubLevels.Three;

    int increaseArmPosition;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime segmentTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Define webcam
        // Step 1. Get live viewport
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Step 2. Create a webcam instance
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCamCenter");
        OpenCvCamera WebCamC = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // Step 2.5 create a new pipeline object?
        pipeline = new TestPipeline();
        // Step 3. Open the Camera Device
        WebCamC.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                WebCamC.setPipeline(pipeline);
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

        //Reset all encoders to have a fresh start when the match starts.
        robot.FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Spin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // THE SCRIPT

        // Stage 00
        stop_n_stare();

        // Stage 01
        drive_2_hub();

        // Stage 02
        raise_arm();

        // Stage 03
        turn2hub();

        // Stage 04
        releaseBlock();

        // Stage 05
        drive_away_from_hub();

        // SCRIPT Stage 06
        salute_the_crowd();

        //Stage 07
        strafe_twards_duck();

        //Stage 08
        drive_to_ducks();

        //Stage 09
        drive_closer_to_ducks();

        //Stage 10
        spin_duck();

        //Stage 11
        back_up_from_ducks();

        // Stage 12
        lower_arm();

        //Stage 13
        ark_strafe();

        //Stage 14
        ark_turn();

        // SCRIPT Stage 14.5
        strafe_to_wall();

        // SCRIPT Stage 15
        driveToLine();

        //SCRIPT Stage 16
        //final_drive_into_Storage_Unit();

        // SCRIPT Stage xx - unused
        //drive_just_a_bit_more();

    } // end runOpMode


    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/

    private void decoderRingA() {  // use this decoder ring for Red Warehouse and Blue Duck

        if (pipeline.didLeftCameraFindTSE()) {
            targetLevel = hubLevels.One;
            //telemetry.addData("Target Level", targetLevel);
        } else if (pipeline.didRightCameraFindTSE()) {
            targetLevel = hubLevels.Two;
            //telemetry.addData("Target Level", targetLevel);
        } else {
            targetLevel = hubLevels.Three;
            //telemetry.addData("Target Level", targetLevel);
        }
        telemetry.addData("Stage:", "xx, decoderRingA");
        telemetry.addData("Scan Value", pipeline.currentScanValue);
        telemetry.addData("Target Level", targetLevel);
        telemetry.addData("Left is", pipeline.didLeftCameraFindTSE());
        telemetry.addData("Rght is", pipeline.didRightCameraFindTSE());
        telemetry.addData("Last Zone Scnned", pipeline.lastZoneScanned);
        telemetry.update();
        sleep(1000);

        //robot.WebCamC.stopStreaming();
    }


    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *        SCRIPT          *
     *                        *
     **************************/

    // SCRIPT Stage 00
    private void stop_n_stare(){
        if (opModeIsActive()) {

            telemetry.addData("Stage:", "00, stop_n_stare");
            telemetry.update();

            sleep(100); // give robot's pipeline time to startflowing - was 1000

            decoderRingA();         // take the data provided by the pipeline and decode it to discover the targetLevel

        }
    } // End stop_n_stare


    // SCRIPT Stage 01
    private void drive_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_2_hub");
            telemetry.update();

            robot.FlipGrip(robot.grip_tight); //Close Gripper
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -9.5; // all same sign indicates drive
            double FR_distance = -9.5;
            double BL_distance = -9.5;
            double BR_distance = -9.5; // as -16

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);
        }

    } //end drive2hub


    // SCRIPT Stage 02
    private void raise_arm(){

        robot.FlipGrip(robot.grip_tight); //Close Gripper

        if (targetLevel == hubLevels.Three) {
            increaseArmPosition = robot.level_3_position;
        } else if (targetLevel == hubLevels.Two) {
            increaseArmPosition = robot.level_2_position;
        } else {
            increaseArmPosition = robot.level_1_position;
        }

        int desiredArmPosition = robot.BotArm.getCurrentPosition() + increaseArmPosition;

        telemetry.addData("Stage:", "02, raise_arm");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        sleep(500); //debut - time to read

        while (opModeIsActive()){

            //telemetry.addData("Note", "In While loop *****");
            //telemetry.update();
            //sleep(2000);

            if(robot.BotArm.getCurrentPosition() < desiredArmPosition)
            {
                robot.BotArm.setPower(0.95);
                robot.BotArm2.setPower(0.95);
                telemetry.addData("Note", "In While loop, TRUE ");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                sleep(100); // time for motors to react?
            }
            else
            {
                robot.BotArm.setPower(0);
                robot.BotArm2.setPower(0);
                telemetry.addData("Note", "In While loop, FALSE");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                //sleep(2000);
                break;  // try a break statement here
            }

        }

        telemetry.addData("Note", "Done Arm motion. Now drop");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        robot.BotArm.setPower(0);

        sleep(500); // just waiting for time to read the telemetry

    } //end raise_arm


    // SCRIPT Stage 03
    private void turn2hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "03, turn2hub");
            telemetry.update();

            robot.FlipGrip(robot.grip_tight); //Close Gripper

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -7.8; // all same sign indicates drive
            double FR_distance = 7.8;
            double BL_distance = -7.8;
            double BR_distance = 7.8; // was 8 then 7

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);
        }
    } //end turn2hub


    // SCRIPT Stage 04
    private void releaseBlock(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "04, realeaseBlock");
            telemetry.update();

            robot.FlipGrip(robot.grip_open); //Open gripper

            sleep(300);

            robot.FlipGrip(robot.grip_wide);

            sleep(100); //Sleep to leave time to open the Gripper - was 500
        }
    } //end releaseBlock


    // SCRIPT Stage 05
    private void drive_away_from_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "05, drive_away_from_hub");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 13; // all same sign indicates drive
            double FR_distance = 13;
            double BL_distance = 13;
            double BR_distance = 13; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end drive_away_from_hub


    // SCRIPT Stage 06
    private void salute_the_crowd(){

        robot.FlipGrip(robot.grip_rest); // close the Gripper Flipper

        int desiredArmPosition = 4000; // was 3000

        telemetry.addData("Stage:", "06, salute_the_crowd");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        //sleep(2000); //debut - time to read

        while (opModeIsActive()){

            if(robot.BotArm.getCurrentPosition() > desiredArmPosition)
            {
                robot.BotArm.setPower(-0.95);
                robot.BotArm2.setPower(-0.95);
                telemetry.addData("Note", "In While loop, TRUE ");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                sleep(100); // time for motors to react?
            }
            else
            {
                robot.BotArm.setPower(0);
                robot.BotArm2.setPower(0);
                telemetry.addData("Note", "In While loop, FALSE");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                //sleep(2000);
                break;  // try a break statement here
            }

        }

        telemetry.addData("Note", "Done Arm motion.");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        robot.BotArm.setPower(0);

        sleep(500); // just waiting for time to read the telemetry

    } //end salute_the_crowd


    // SCRIPT Stage 07
    private void strafe_twards_duck(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "07, strfe_twards_duck");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -18;
            double FR_distance = 18;
            double BL_distance = 18; // was 15.5
            double BR_distance = -18; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end strafe_twards_duck


    // SCRIPT Stage 08
    private void drive_to_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "08, drive to ducks");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 8; // all same sign indicates drive
            double FR_distance = 8;
            double BL_distance = 8;
            double BR_distance = 8; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end drive_to_ducks


    // SCRIPT Stage 09
    private void drive_closer_to_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "09, drive to ducks");
            telemetry.update();

            // Start spinning
            robot.Spin.setPower(.4);
            robot.Spin2.setPower(.4); // was .6

            // Drive Targets - move forward
            double speed = .05;
            double FL_Distance = 21; // all same sign indicates drive
            double FR_distance = 21;
            double BL_distance = 21;
            double BR_distance = 21; // was 25 then 21

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end drive_closer_to_ducks


    // SCRIPT Stage 10
    private void spin_duck(){

        int SpinTime = 500;

        //sleep(1000); //debut - time to read

        while (opModeIsActive()){

            telemetry.addData("Stage:", "10, spin_duck");
            telemetry.update();

            robot.Spin.setPower(.4);
            robot.Spin2.setPower(.4); // was .6

            sleep(SpinTime);
            break;
        }

        robot.Spin.setPower(0);
        robot.Spin2.setPower(0);

    } //end spin_duck


    // SCRIPT Stage 11
    private void back_up_from_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "11, back up from ducks");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3; // was .5
            double FL_Distance = -9; // all same sign indicates drive
            double FR_distance = -9;
            double BL_distance = -9;
            double BR_distance = -9; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end back_up_from_ducks_strafe_to_unit


    // SCRIPT Stage 12
    private void lower_arm(){

        robot.FlipGrip(robot.grip_tight); // close the Gripper Flipper

        //int decreaseArmPosition = -5500; //

        int desiredArmPosition = 1;  // can this be 0?

        telemetry.addData("Stage:", "12, lower_arm");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        //sleep(2000); //debut - time to read

        while (opModeIsActive()){

            if(robot.BotArm.getCurrentPosition() > desiredArmPosition)
            {
                robot.BotArm.setPower(-0.95);
                robot.BotArm2.setPower(-0.95);
                telemetry.addData("Note", "In While loop, TRUE ");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                sleep(100); // time for motors to react?
            }
            else
            {
                robot.BotArm.setPower(0);
                robot.BotArm2.setPower(0);
                telemetry.addData("Note", "In While loop, FALSE");
                telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
                telemetry.addData("Arm Destinan", desiredArmPosition);
                telemetry.update();
                //sleep(2000);
                break;  // try a break statement here
            }

        }

        telemetry.addData("Note", "Done Arm motion.");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        robot.BotArm.setPower(0);

        //sleep(2000);

        robot.FlipGrip(robot.grip_open);

        sleep(500); // just waiting for time to read the telemetry

    } //end lower_arm


    // SCRIPT Stage 13
    private void ark_strafe(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "13, ark strafe");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -16;
            double FR_distance = 16; // was 14
            double BL_distance = 16;
            double BR_distance = -16; // was 18 and, more recent, 20

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end ark_strafe


    // SCRIPT Stage 14
    private void ark_turn(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "14, ark_turn");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 8; // all same sign indicates drive
            double FR_distance = -10; // was 14
            double BL_distance = 8;
            double BR_distance = -10; // was 18 and, more recent, 20

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end ark_turn



    // SCRIPT Stage 14.5
    private void strafe_to_wall(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "14.5, strafe_to_wall");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .5;
            double FL_Distance = -7;
            double FR_distance = 7;
            double BL_distance = 7;
            double BR_distance = -7;

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end strafe_to_wall



    // Script Stage 15
    private void driveToLine(){

        double stopTime = runtime.seconds() + 1.5;
        // while(opModeIsActive() && runtime.seconds() < stopTime){
        while(opModeIsActive()){

            telemetry.addData("Stage:", "15, driveToLine");
            telemetry.update();

            while(robot.LineStopper.blue() < 160){

                robot.FLDrive.setPower(-.1);
                robot.FRDrive.setPower(-.1);
                robot.BLDrive.setPower(-.1);
                robot.BRDrive.setPower(-.1);

                sleep(300); // was 500

                robot.FLDrive.setPower(0);
                robot.FRDrive.setPower(0);
                robot.BLDrive.setPower(0);
                robot.BRDrive.setPower(0);

                if (robot.LineStopper.blue() > 160) {
                    break;
                }

                sleep(300); // was 500

            }
            robot.FLDrive.setPower(0);
            robot.FRDrive.setPower(0);
            robot.BLDrive.setPower(0);
            robot.BRDrive.setPower(0);

            // back up a bit
            double speed = .4; // was .2
            double FL_Distance = 2; // all same sign indicates drive
            double FR_distance = 2;
            double BL_distance = 2;
            double BR_distance = 2; //was 3.5

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);
            break;
        }

    } // end driveToLine


    // SCRIPT Stage 16
    private void final_drive_into_Storage_Unit(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "16, final_drive_into_Storage_Unit");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .2; // was .3
            double FL_Distance = -1; // all same sign indicates drive
            double FR_distance = -1;
            double BL_distance = -1;
            double BR_distance = -1; //was 3.5

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);
        }

    } //end final_drive_into_Storage_Unit


    // SCRIPT Stage xx - unused
    private void drive_just_a_bit_more(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "11, drive_just_a_bit_more");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .2; // was .3
            double FL_Distance = -2; // all same sign indicates drive
            double FR_distance = -2;
            double BL_distance = -2;
            double BR_distance = -2; //was -3

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

        }

    } //end

}  // end class Auton_Blue_Ducks