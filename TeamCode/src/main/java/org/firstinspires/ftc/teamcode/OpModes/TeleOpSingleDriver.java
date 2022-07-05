package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
//import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp 1 Driver", group = "Competition")
//  @Disabled

public class TeleOpSingleDriver extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOpSingleDriver(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
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

        boolean TSEMode=false;
        boolean TSEtoggle=false;

        ElapsedTime runtime = new ElapsedTime();
        double currentTime = 0;

        double chainsawPower=1;
        boolean chainsawToggle=false;

        double turn, drive, left, right, max;

        waitForStart();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        mechController.start();
        turretController.start();
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
            if(drive<0){
                drive*=robot.REVERSE_MULTIPLIER;
            }
            turn  =  gamepad1.right_stick_x*robot.TURN_MULTIPLIER;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            if(Math.abs(left)>1){
                left/=Math.abs(left);
            }
            if(Math.abs(right)>1){
                right/=Math.abs(right);
            }
            //100% power forward
            if(-gamepad1.left_stick_y>0.95&&-gamepad1.right_stick_y>0.95){
                left=1;
                right=1;
            }
            //100% power in reverse
            if(-gamepad1.left_stick_y<-0.95&&-gamepad1.right_stick_y<-0.95){
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

            if(gamepad1.a&&toggleIntake){
                toggleIntake=false;
                intakeDown=!intakeDown;
                isDeployed=false;
                bumpCount=0;
                currentTime = runtime.time();
            }
            //automatically retract intake
            if(intakeDown&&robot.bucketSensor.getDistance(DistanceUnit.MM) < 120&&robot.motorArmAngle1.getCurrentPosition()>900){
                intakeDown=false;
                isDeployed=false;
                bumpCount=0;
            }
            //check if intake needs to be reversed and then deploy or retract
            if(!gamepad1.b) {
                if (intakeDown) {
                    if(Math.abs(robot.turrentEncoder.getCurrentPosition())<=5) {
                        mechControl.intakeOn(isDeployed, currentTime);
                    }
                } else {
                    mechControl.intakeOff(isDeployed, TSEMode);
                }
            }else{
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }
            mechControl.beaterOn(intakeDown,robot.motorArmAngle1.getCurrentPosition());
//end of intake controls

//chainsaw control section (GP1, Bumpers)
            if(!gamepad1.dpad_up&&!gamepad1.dpad_down){
                chainsawToggle=true;
            }

            //adjust chainsaw power
            if(gamepad1.dpad_up&&chainsawToggle&&chainsawPower<1){
                chainsawToggle=false;
                chainsawPower+=0.2;
            }else if(gamepad1.dpad_down&&chainsawToggle&&chainsawPower>0.4){
                chainsawToggle=false;
                chainsawPower-=0.2;
            }

            //reset chainsaw power to 1
            if(gamepad1.right_bumper){
                chainsawPower=1;
            }

            //apply chainsaw power
            if(gamepad1.right_trigger>0.5){
                robot.motorChainsaw.setPower(chainsawPower);
            }
            else if(gamepad1.left_trigger>0.5){
                robot.motorChainsaw.setPower(-chainsawPower);
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
            if(!gamepad1.y){
                TSEtoggle=true;
            }
            if(gamepad1.y&&TSEtoggle&&!isDeployed){
                TSEtoggle=false;
                TSEMode=!TSEMode;
            }
            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
                mechControl.resetIntake();
                intakeDown=false;
            }

            //move arm to scoring positions
            if(TSEMode) {
                //mode to pick up TSE
                if (bumpCount == 1) {
                    mechControl.TSEDown();
                } else if (bumpCount == 2) {
                    mechControl.TSEresting();
                } else if (bumpCount == 3) {
                    mechControl.TSEtop();
                }
            }else{
                //move arm to score
                if (bumpCount == 1) {
                    mechControl.scoringPos1();
                } else if (bumpCount == 2) {
                    mechControl.scoringPos2();
                } else if (bumpCount == 3) {
                    mechControl.scoringPos3();
                }
            }

            //reset arm to zero with or without scoring
            if(gamepad1.dpad_left){
                bumpCount=0;
                isDeployed=false;
                mechControl.moveToZero(TSEMode);
            }
//end of arm controls

//bucket control section (GP1, Dpad Right)
            if(gamepad1.dpad_right&&!TSEMode){
                if(bumpCount==1){
                    bucketAngle=0.15;
                }else if(bumpCount==2){
                    bucketAngle=0.05;
                }else {
                    bucketAngle = 0.25;
                }
            }else if(gamepad1.dpad_right&&bumpCount==3&&TSEMode) {
                bucketAngle = 0;
            }else{
                if(intakeDown){
                    bucketAngle=0.4;
                }else if(bumpCount==1&&!TSEMode&&robot.motorArmAngle1.getCurrentPosition()<750) {
                    bucketAngle = 0.6;
                }else if(bumpCount==2&&!TSEMode){
                    bucketAngle=0.55;
                }else if(bumpCount==3&&!TSEMode) {
                    bucketAngle = 0.75;
                }else if(bumpCount==1&&TSEMode){
                    bucketAngle=0.4;
                }else if(bumpCount==3&&TSEMode){
                    bucketAngle=0.7;
                }else{
                    if(robot.motorArmAngle1.getCurrentPosition()<500) {
                        bucketAngle = 0.5;
                    }
                }
            }
            robot.bucketDump.setPosition(bucketAngle);
//end of bucket controls

//sweeper controls
            robot.sweeperPink.setPosition(robot.PINK_SWEEPER_UP);
            robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_UP);
//end of sweeper controls

//keep turret zeroed
            turretControl.setTargetPosition(0);

            telemetry.addData("TSE MODE: ",TSEMode);
            telemetry.addData("","");
            telemetry.addData("Arm angle 1:",robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm angle 2:",robot.motorArmAngle2.getCurrentPosition());
            telemetry.addData("Last Intake Servo Pos",robot.bucketDump.getPosition());
            telemetry.addData("Turret Current Angle: ",robot.turrentEncoder.getCurrentPosition());
            telemetry.addData("Left Power: ",left);
            telemetry.addData("Right Power: ",right);
            telemetry.addData("Chainsaw Power: ",chainsawPower);
            telemetry.addData("Happy Driving ",")");
            telemetry.update();
        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();
        turretControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


