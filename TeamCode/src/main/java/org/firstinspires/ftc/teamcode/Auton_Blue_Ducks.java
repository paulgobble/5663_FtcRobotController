// Version 0.0

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton_Blue_Ducks", group="Autonomous")
//@Disabled
public class Auton_Blue_Ducks extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();

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

        // Stage 00
        stop_n_stare();

        // Stage 01
        drive_2_hub();

        // Stage 02
        raise_arm();

        // Stage 02.5
        turn2hub();

        // Stage 02.7
        releaseBlock();

        // Stage 04
        drive_away_from_hub();

        // SCRIPT Stage 04.5
        salute_the_crowd();

        //Stage 05
        strafe_twards_duck();

        //Stage 06
        drive_to_ducks();

        //Stage 07
        drive_closer_to_ducks();

        //Stage 08
        spin_duck();

        //Stage 07 - again
        drive_closer_to_ducks();

        //Stage 08 - again
        spin_duck();

        //Stage 09
        back_up_from_ducks_strafe_to_unit();

        // Stage 03
        lower_arm();

        //Stage 10
        ark_strafe();

        // SCRIPT Stage 11
        drive_just_a_bit_more();

    } // end runOpMode




    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/

    private void decoderRingA() {  // use this decoder ring for Red Warehouse and Blue Duck

        if (robot.leftCameraFoundTSE) {
            targetLevel = hubLevels.One;
            telemetry.addData("Target Level", targetLevel);
        } else if (robot.rightCameraFoundTSE) {
            targetLevel = hubLevels.Two;
            telemetry.addData("Target Level", targetLevel);
        } else {
            targetLevel = hubLevels.Three;
            telemetry.addData("Target Level", targetLevel);
        }
        telemetry.update();
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

            decoderRingA();         // take the data provided by the pipeline and decode it to discover the targetLevel

        }
    } // End stop_n_stare


    // SCRIPT Stage 01
    private void drive_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
            telemetry.update();

            robot.FlipGrip(.2); //Close Gripper
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

    // SCRIPT Stage 01.5
    private void turn2hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -9; // all same sign indicates drive
            double FR_distance = 9;
            double BL_distance = -9;
            double BR_distance = 9; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);
        }
    } //end turn2hub

    // SCRIPT Stage 01.7
    private void releaseBlock(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01.7, realeaseBlock");
            telemetry.update();

            robot.FlipGrip(.3); //Open gripper

            sleep(500); //Sleep to leave time to open the Gripper
        }
    } //end releaseBlock

    // SCRIPT Stage 02
    private void raise_arm(){

        robot.FlipGrip(.2); //Close Gripper

        if (targetLevel == hubLevels.Three) {
            increaseArmPosition = 5000; // was 5000
        } else if (targetLevel == hubLevels.Two) {
            increaseArmPosition = 5800;
        } else {
            increaseArmPosition = 6300;
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



        sleep(500); // just waiting for time to read the telemetry

    } //end raise_arm


    // SCRIPT Stage 03
    private void lower_arm(){

        robot.FlipGrip(.2); // close the Gripper Flipper

        //int decreaseArmPosition = -5500; //

        int desiredArmPosition = 1;  // can this be 0?

        telemetry.addData("Stage:", "03, lower_arm");
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

        robot.FlipGrip(.3);

        sleep(500); // just waiting for time to read the telemetry

    } //end lower_arm



    // SCRIPT Stage 04
    private void drive_away_from_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "04, drive away");
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

    } //end


    // SCRIPT Stage 04.5
    private void salute_the_crowd(){

        robot.FlipGrip(.2); // close the Gripper Flipper

        int desiredArmPosition = 4000; // was 3000

        telemetry.addData("Stage:", "04.5, salute_the_crowd");
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

        sleep(500); // just waiting for time to read the telemetry

    } //end salute_the_crowd


    // SCRIPT Stage 05
    private void strafe_twards_duck(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "05, strfe_twards_duck");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -15.5; // all same sign indicates drive
            double FR_distance = 15.5;
            double BL_distance = 15.5;
            double BR_distance = -15.5; // flipped

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end

    // SCRIPT Stage 06
    private void drive_to_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "06, drive to ducks");
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

    } //end

    // SCRIPT Stage 07
    private void drive_closer_to_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "07, drive to ducks");
            telemetry.update();

            // Start spinning
            robot.Spin.setPower(.4);
            robot.Spin2.setPower(.4); // was .6

            // Drive Targets - move forward
            double speed = .05;
            double FL_Distance = 10; // all same sign indicates drive
            double FR_distance = 10;
            double BL_distance = 10;
            double BR_distance = 10; // was 6

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            //sleep(500);

        }

    } //end


    // SCRIPT Stage 08
    private void spin_duck(){

        int SpinTime = 2500;

        //sleep(1000); //debut - time to read

        while (opModeIsActive()){

            robot.Spin.setPower(.4);
            robot.Spin2.setPower(.4); // was .6

            sleep(SpinTime);
            break;
        }

        robot.Spin.setPower(0);
        robot.Spin2.setPower(0);

    } //end spin_duck

    // SCRIPT Stage 09
    private void back_up_from_ducks_strafe_to_unit(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "09, back up from ducks and strafe to unit");
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

    } //end

    // SCRIPT Stage 10
    private void ark_strafe(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "10, ark strafe");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -40; // all same sign indicates drive
            double FR_distance = 40;
            double BL_distance = 22;
            double BR_distance = -22; //  was 30 and 18

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end


    // SCRIPT Stage 11
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

}  // end class Auton_Red_Ducks