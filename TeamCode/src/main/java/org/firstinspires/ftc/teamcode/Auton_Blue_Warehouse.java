package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton_Blue_Warehouse", group="Autonomous")
//@Disabled
public class Auton_Blue_Warehouse extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();

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

        robot.init(hardwareMap);  //I'm guessing we don't need to call a hMap method.  Robot's init method takes care of this

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

        // Stage 01.5
        turn_2_hub();

        // Stage 02.7
        dropCargo();

        //Stage 04
        drive_n_turn_back();

        //Stage 03
        lower_arm();

        //Stage 05
        strafe_to_warehouse_position();

        //Stage 06
        outta_the_way();


    } // end runOpMode


    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/


    private void decoderRingB() {  // use this decoder ring for Red Duck and Blue Warehouse

        if (robot.leftCameraFoundTSE) {
            targetLevel = hubLevels.Two;
            telemetry.addData("Target Level", targetLevel);
        } else if (robot.rightCameraFoundTSE) {
            targetLevel = hubLevels.Three;
            telemetry.addData("Target Level", targetLevel);
        } else {
            targetLevel = hubLevels.One;
            telemetry.addData("Target Level", targetLevel);
        }
        telemetry.update();

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

            sleep(1000); // give robot's pipeline time to startflowing

            decoderRingB();         // take the data provided by the pipeline and decode it to discover the targetLevel

        }
    } // End stop_n_stare


    // SCRIPT Stage 01
    private void drive_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_2_hub");
            telemetry.update();

            robot.FlipGrip(.1);
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


    // SCRIPT Stage 01.5
    private void turn_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01.5, trun_2_hub");
            telemetry.update();

            // Drive Targets - turn towards hub
            double speed = .3;
            double FL_Distance = 10; // alternating signs indicate a turn
            double FR_distance = -10;
            double BL_distance = 10;
            double BR_distance = -10; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(1000);

        }

    } //end turn_2_hub


    // SCRIPT Stage 02
    private void raise_arm(){


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


    // SCRIPT Stage 2.7
    private void dropCargo(){

        robot.FlipGrip(.3);
        sleep(1000);

    }


    // SCRIPT Stage 03
    private void lower_arm(){

        robot.FlipGrip(.05);

        //int increaseArmPosition = 5500; // was 5000

        int desiredArmPosition = 0; // robot.BotArm.getCurrentPosition() 1 increaseArmPosition;

        telemetry.addData("Stage:", "02, raise_arm");
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


    // SCRIPT Stage 04
    private void drive_n_turn_back(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
            telemetry.update();

            robot.FlipGrip(.05);
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

    } //end drive_n_turn


    // SCRIPT Stage 05
    private void strafe_to_warehouse_position(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
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

    } //end drive_n_turn


    // SCRIPT Stage 06
    private void outta_the_way(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
            telemetry.update();

            robot.FlipGrip(.2);
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

}  // end class Auton_Red_Ducks