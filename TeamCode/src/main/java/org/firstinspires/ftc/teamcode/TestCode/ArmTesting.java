package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ArmTesting", group = "Competition")
@Disabled

public class ArmTesting extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public ArmTesting(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        MechControlLibrary armControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread armController = new Thread(armControl);


        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Robot state = ", "INITIALIZED");
//        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
//        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        int angle1Pos=0;
        int angle2Pos=0;
        double bucketAngle=0.5;
        int bumpCount=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        telemetry.addData("Arm Angle 1:",robot.motorArmAngle1.getCurrentPosition());
        telemetry.addData("Arm Angle 2:",robot.motorArmAngle2.getCurrentPosition());
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        double currentTime = 0;


        waitForStart();
        armController.start();
        currentTime = runtime.time();

        while(opModeIsActive()) {
            if(!gamepad1.a){
                toggleIntake=true;
            }

            if(gamepad1.a&&toggleIntake&&!isDeployed){
                toggleIntake=false;
                intakeDown=!intakeDown;
            }
            if(intakeDown){
                armControl.intakeOn(isDeployed, currentTime);
            }else if(gamepad1.b){
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }else{
                armControl.intakeOff(isDeployed,false);
            }


            /*
            //intake control
            if(gamepad1.a) {
                //turns intake on
                armControl.intakeOn(isDeployed);
            }else if(gamepad1.b) {
                //reverses intake
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }else{
                //turns intake off
                armControl.intakeOff(isDeployed);
            }
            */
            //intake ramp controls (GP1, Dpad)
            /*
            if(gamepad1.dpad_left){
                robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
            } else if(gamepad1.dpad_right){
                robot.intakeTilt.setPosition(robot.INTAKE_TILT_OUTPUT);
            } else{
            }
            */


            //chainsaw controls (GP1, Bumpers)
            if(gamepad1.left_bumper){
                robot.motorChainsaw.setPower(robot.CHAIN_POW);
            }else if(gamepad1.right_bumper){
                robot.motorChainsaw.setPower(-robot.CHAIN_POW);
            }else{
                robot.motorChainsaw.setPower(0);
            }


            //arm control section
            telemetry.addData("Arm Angle 1 = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm Angle 2 = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.update();

            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(!gamepad1.x){
                toggleReadyDown=true;
            }
            if(!gamepad1.dpad_up){
                toggleReadyUp=true;
            }

            //adds 1 to bumpCount if x isn't held down
            if(gamepad1.x && toggleReadyDown){
                toggleReadyDown=false;
                if(bumpCount<3) {
                    bumpCount += 1;
                }
            }

            //removes 1 from bumpCount if dpad up isn't held down
            if(gamepad1.dpad_up && toggleReadyUp){
                toggleReadyUp=false;
                if(bumpCount>1) {
                    bumpCount -= 1;
                }
            }

            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
                armControl.resetIntake();
                intakeDown=false;
            }

            //move arm to scoring positions
            if(bumpCount==1){
                armControl.scoringPos1();
            }else if(bumpCount==2){
                armControl.scoringPos2();
            }else if(bumpCount==3){
                armControl.scoringPos3();
            }

            //reset arm to zero with or without scoring
            if(gamepad1.dpad_down){
                bumpCount=0;
                isDeployed=false;
                armControl.moveToZero(false);
            }else{

            }

            //reset arm to zero AFTER SCORING
            if(gamepad1.y){
                if(isDeployed){
                    armControl.resetArm();
                    bumpCount=0;
                    isDeployed=false;
                }
            }
            if(gamepad1.right_trigger>0.25){
                armControl.incrementUp();
            }else if(gamepad1.left_trigger>0.25){
                armControl.incrementDown();
            }else{

            }


            //bucket control
            if(gamepad1.dpad_right){
                bucketAngle=-1;
            } else{
                bucketAngle=0.5;
                if(bumpCount==3){
                    bucketAngle=0.75;
                }
            }
            robot.bucketDump.setPosition(bucketAngle);



            /**
             * #################################################################################
             * #################################################################################
             * #################      PROVIDE USER FEEDBACK    #################################
             * #################################################################################
             * #################################################################################
             *
            telemetry.addData("Motor Shooter1 Encoder = ", robot.motorShooter1.getCurrentPosition());
            telemetry.addData("Motor Shooter2 Encoder = ", robot.motorShooter2.getCurrentPosition());
            telemetry.addData("Motor RF Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Motor LF Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Motor RR Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Motor LR Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Calculated Distance = ", drive.calcDistance(0,0,0,0,0));
            telemetry.addData("Shooter RPM = ", (robot.motorShooter1.getVelocity() / 28 * 60));
            telemetry.update(); */
        }   // end of while opModeIsActive()
        armControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


