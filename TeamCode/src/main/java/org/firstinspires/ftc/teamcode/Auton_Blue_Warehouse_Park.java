package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auton_Blue_Warehouse_Park", group="Autonomous")
//@Disabled
public class Auton_Blue_Warehouse_Park extends LinearOpMode {

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

    private int increaseArmPosition;

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

        // SCRIPT Stage 00
        stop_n_stare();

        // Stage 01
        drive_2_hub();

        // Stage 02
        raise_arm();

        // Stage 03
        turn_2_hub();

        // Stage 04
        dropCargo();

        //Stage 05
        turn_back();

        //Stage 06
        lower_arm();

        //Stage 07
        strafe_to_warehouse_position();

        //Stage 08
        //outta_the_way();

    } // end runOpMode


    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/


    private void decoderRingB() {  // use this decoder ring for Red Duck and Blue Warehouse

        if (pipeline.didLeftCameraFindTSE()) {
            targetLevel = hubLevels.Two;
            //telemetry.addData("Target Level", targetLevel);
        } else if (pipeline.didRightCameraFindTSE()) {
            targetLevel = hubLevels.Three;
            //telemetry.addData("Target Level", targetLevel);
        } else {
            targetLevel = hubLevels.One;
            //telemetry.addData("Target Level", targetLevel);
        }
        telemetry.addData("Stage:", "xx, decoderRingB");
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
            telemetry.addData("Scan Value", pipeline.currentScanValue);
            telemetry.update();

            sleep(100); // give robot's pipeline time to startflowing - was 1000

            decoderRingB();         // take the data provided by the pipeline and decode it to discover the targetLevel

        }
    } // End stop_n_stare


    // SCRIPT Stage 01
    private void drive_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_2_hub");
            telemetry.update();

            robot.FlipGrip(robot.grip_tight);
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -9; // all same sign indicates drive
            double FR_distance = -9;
            double BL_distance = -9;
            double BR_distance = -9; // was -10

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end drive_2_hub


    // SCRIPT Stage 02
    private void raise_arm(){

        robot.FlipGrip(robot.grip_tight);

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

        //sleep(2000);

        //robot.FlipGrip(.3);

        sleep(1000); // just waiting for time to read the telemetry

    } //end raise_arm


    // SCRIPT Stage 03
    private void turn_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "03, trun_2_hub");
            telemetry.update();

            robot.FlipGrip(robot.grip_tight);

            // Drive Targets - turn towards hub
            double speed = .3;
            double FL_Distance = 9; // alternating signs indicate a turn
            double FR_distance = -9;
            double BL_distance = 9;
            double BR_distance = -9; // flipped - was 10

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(1000);

        }

    } //end turn_2_hub


    // SCRIPT Stage 04
    private void dropCargo(){

        telemetry.addData("Stage:", "04, realeaseBlock");
        telemetry.update();

        robot.FlipGrip(robot.grip_open); //Open gripper

        sleep(300);

        robot.FlipGrip(robot.grip_wide);

        sleep(100); //Sleep to leave time to open the Gripper - was 1000

    } // end dropCargo


    // SCRIPT Stage 05
    private void turn_back(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "05, turn_back");
            telemetry.update();

            robot.FlipGrip(robot.grip_wide); // open grippers wider for more clearance
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 10; // all same sign indicates drive
            double FR_distance = -10;
            double BL_distance = 10;
            double BR_distance = -10; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end turn_back


    // SCRIPT Stage 06
    private void lower_arm(){

        robot.FlipGrip(robot.grip_tight);

        //int increaseArmPosition = 5500; // was 5000

        int desiredArmPosition = 0; // robot.BotArm.getCurrentPosition() 1 increaseArmPosition;

        telemetry.addData("Stage:", "06, raise_arm");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        sleep(500); //debut - time to read

        while (opModeIsActive()){

            //telemetry.addData("Note", "In While loop *****");
            //telemetry.update();
            //sleep(2000);

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

        telemetry.addData("Note", "Done Arm motion. Now drop");
        telemetry.addData("Arm Position", robot.BotArm.getCurrentPosition());
        telemetry.addData("Arm Destinan", desiredArmPosition);
        telemetry.update();

        robot.BotArm.setPower(0);

        //sleep(2000);

        sleep(500); // just waiting for time to read the telemetry

    } //end raise_arm


    // SCRIPT Stage 07
    private void strafe_to_warehouse_position(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "07, strafe_to_warehouse_position");
            telemetry.update();

            robot.FlipGrip(.2);
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -22; // all same sign indicates drive
            double FR_distance = 22;
            double BL_distance = 22;
            double BR_distance = -22; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            // Drive Targets - turn towards hub
            speed = .3;
            FL_Distance = 30; // alternating signs indicate a turn
            FR_distance = 30;
            BL_distance = 30;
            BR_distance = 30; //was 12

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end strafe_to_warehouse_position


    // SCRIPT Stage 08
    private void outta_the_way(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "08, outta_the_way");
            telemetry.update();

            robot.FlipGrip(robot.grip_tight);
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 30; // all same sign indicates drive
            double FR_distance = -30;
            double BL_distance = -30;
            double BR_distance = 30; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);
        }

    } //end outta_the_way

}  // end class Auton_Blue_Ducks