// Version 0.0

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton_Red_Ducks", group="Autonomous")
//@Disabled
public class Auton_Red_Ducks extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();

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

        // Stage 01
        drive_n_turn_2_hub();

        // Stage 02
        raise_arm();

        // Stage 03
        lower_arm();

        // Stage 04
        drive_away_from_hub();

        //Stage 05
        strafe_twards_duck();

        //Stage 06
        drive_to_ducks();

        //Stage 07
        drive_closer_to_ducks();

        //Stage 08
        spin_duck();

        //Stage 09
        back_up_from_ducks_strafe_to_unit();

        //Stage 10
        ark_strafe();

    } // end runOpMode




    /*********************************
     *                               *
     *   Internal opMode Functions   *
     *                               *
     *********************************/






    /**************************
     *                        *
     *  Autonomous  Segments  *
     *                        *
     *        SCRIPT          *
     *                        *
     **************************/

    // SCRIPT Stage 01
    private void drive_n_turn_2_hub(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "01, drive_n_trun_2_hub");
            telemetry.update();

            robot.FlipGrip(.2);
            sleep(500); // wait for good grip

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = -11; // all same sign indicates drive
            double FR_distance = -11;
            double BL_distance = -11;
            double BR_distance = -11; // as -16

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            // Drive Targets - turn towards hub
            speed = .3;
            FL_Distance = 9; // alternating signs indicate a turn
            FR_distance = -9;
            BL_distance = 9;
            BR_distance = -9; //was 12

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end drive_n_turn


    // SCRIPT Stage 02
    private void raise_arm(){

        int increaseArmPosition = 5500; // was 5000

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

        robot.FlipGrip(.3);

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
            double FL_Distance = 10; // all same sign indicates drive
            double FR_distance = 10;
            double BL_distance = 10;
            double BR_distance = 10; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end



    // SCRIPT Stage 05
    private void strafe_twards_duck(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "05, strfe_twards_duck");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .3;
            double FL_Distance = 18; // all same sign indicates drive
            double FR_distance = -18;
            double BL_distance = -18;
            double BR_distance = 18; //

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

            sleep(500);

        }

    } //end

    // SCRIPT Stage 07
    private void drive_closer_to_ducks(){

        // insure the opMode is still active
        if (opModeIsActive()){

            telemetry.addData("Stage:", "07, drive to ducks");
            telemetry.update();

            // Drive Targets - move forward
            double speed = .05;
            double FL_Distance = 2; // all same sign indicates drive
            double FR_distance = 2;
            double BL_distance = 2;
            double BR_distance = 2; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end


    // SCRIPT Stage 08
    private void spin_duck(){

        int SpinTime = 2500;

        sleep(1000); //debut - time to read

        while (opModeIsActive()){

            robot.Spin.setPower(-.6);
            robot.Spin2.setPower(-.6);

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
            double speed = .5;
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
            double speed = .5;
            double FL_Distance = 10; // all same sign indicates drive
            double FR_distance = -10;
            double BL_distance = -10;
            double BR_distance = 10; //

            // Call encoderDrive
            robot.encoderDrive(speed, FL_Distance, FR_distance, BL_distance, BR_distance);

            sleep(500);

        }

    } //end

}  // end class Auton_Red_Ducks