package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpThreads", group = "Competition")
@Disabled

public class TeleOpThreads extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOpThreads(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread armController = new Thread(mechControl);
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
        telemetry.update();
        double bucketAngle=0.5;
        int bumpCount=0;
        int chainsawMode=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        boolean chainsawReady=false;
        double turn, drive, left, right, max;

        ElapsedTime runtime = new ElapsedTime();
        double currentTime = 0;


        waitForStart();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        armController.start();

        currentTime = runtime.time();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            Left Stick - forward/backward
            Right Stick - turn
            A - intake
            B - reverse intake
            X - toggle scoring position down
            Y - intake position
            Dpad up - toggle scoring position up
            Dpad down - reset arm to zero
            Dpad right - dump bucket
            Left bumper - chainsaw direction 1
            Right bumper - chainsaw direction 2
            */

//drive control section (GP1, Joysticks)
            drive = -gamepad1.left_stick_y*robot.DRIVE_MULTIPLIER;
            turn  =  gamepad1.right_stick_x*robot.TURN_MULTIPLIER;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            if(Math.abs(left)>1){
                left/=Math.abs(left);
            }
            if(Math.abs(right)>1){
                left/=Math.abs(right);
            }
            //100% power forward
            if(-gamepad1.left_stick_y>0.95&&-gamepad1.right_stick_y>0.95){
                left=1;
                right=1;
            }
            //100% power in reverse
            if(-gamepad1.left_stick_y<0-.95&&-gamepad1.right_stick_y<-0.95){
                left=-1;
                right=-1;
            }

            // Output the safe vales to the motor drives.
            robot.motorL1.setPower(left);
            robot.motorL2.setPower(left);
            robot.motorR1.setPower(right);
            robot.motorR2.setPower(right);
//end of drive controls

//intake control section (GP1, A)
            //allows for intake toggle and not button hold down
            if(!gamepad1.a){
                toggleIntake=true;
            }

            //if intake isn't deployed, deploy it & vice versa
            if(gamepad1.a&&toggleIntake&&!isDeployed){
                toggleIntake=false;
                intakeDown=!intakeDown;
                currentTime = runtime.time();
            }
            //check if intake needs to be reversed and then deploy or retract
            if(!gamepad1.b) {
                if (intakeDown) {
                    mechControl.intakeOn(isDeployed, currentTime);
                } else {
                    mechControl.intakeOff(isDeployed,false);
                }
            }else{
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }
//end of intake controls

//chainsaw control section (GP1, Bumpers)
            //chainsaw ready if bumpers are not held down
            if(!gamepad1.right_bumper&&!gamepad1.left_bumper){
                chainsawReady=true;
            }
            /*CHAINSAW MODES:
            0: off
            1: Blue side
            2: Red side
            */
            //red side (left bumper) chainsaw toggle
            if(gamepad1.right_bumper&&chainsawReady){
                if(chainsawMode==0){
                    chainsawMode=1;
                }else if(chainsawMode==1){
                    chainsawMode=0;
                }
            }
            //blue side (right bumper) chainsaw toggle
            if(gamepad1.left_bumper&&chainsawReady){
                if(chainsawMode==0){
                    chainsawMode=2;
                }else if(chainsawMode==2){
                    chainsawMode=0;
                }
            }
            //start or stop chainsaw based on mode
            //
            if(chainsawMode==1){
                mechControl.chainsawRampRed();
            }else if(chainsawMode==2){
                mechControl.chainsawRampBlue();
            }else{
                robot.motorChainsaw.setPower(0);
            }
//end of chainsaw controls

//arm control section (GP1, X, Y, Dpad Down)
            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(!gamepad1.x){
                toggleReadyDown=true;
            }
            if(!gamepad1.dpad_up){
                toggleReadyUp=true;
            }
            //end of arm toggle checks

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
                mechControl.resetIntake();
                intakeDown=false;
            }

            //move arm to scoring positions
            if(bumpCount==1){
                mechControl.scoringPos1();
            }else if(bumpCount==2){
                mechControl.scoringPos2();
            }else if(bumpCount==3){
                mechControl.scoringPos3();
            }

            //reset arm to zero with or without scoring
            if(gamepad1.dpad_down){
                bumpCount=0;
                isDeployed=false;
                mechControl.moveToZero(false);
            }else{

            }

            //reset arm to zero AFTER SCORING
            if(gamepad1.y){
                if(isDeployed){
                    mechControl.resetArm();
                    bumpCount=0;
                    isDeployed=false;
                }
            }
//end of arm controls

//bucket control section (GP1, Dpad Right)
            if(gamepad1.dpad_right){
                bucketAngle=-1;
            } else{
                bucketAngle=0.5;
                if(bumpCount==3){
                    bucketAngle=0.75;
                }
            }
            robot.bucketDump.setPosition(bucketAngle);
//end of bucket controls


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
        //stops mechanism thread
        mechControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


