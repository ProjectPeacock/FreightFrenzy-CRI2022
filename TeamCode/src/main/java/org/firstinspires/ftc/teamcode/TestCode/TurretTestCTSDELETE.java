//  change comments
/*
    IMPORTANT NOTE: This program is strictly to be used to refine the turret control library and
    for general turret thread testing.
 */
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Turret testing & debugging", group = "Dev")
@Disabled
public class TurretTestCTSDELETE extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public void runOpMode(){
        //mechanism control thread
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        //turret control thread
        TurretControlThread turretControl = new TurretControlThread(robot, 0);
        Thread turretController = new Thread(turretControl);
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();
        double bucketAngle=0.5;
        int bumpCount=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        boolean turretToggle=false;

        boolean TSEMode=false;
        boolean TSEtoggle=false;

        double chainsawPower=1;
        boolean chainsawToggle=false;

        int turretPreset=0;
        double turn, drive, left, right, max;
        int turretPosition=0;
        int turretThreshold=2;

        waitForStart();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        mechController.start();
        turretController.start();

        while(opModeIsActive()) {

            //intake control section (GP1, A)
            //allows for intake toggle and not button hold down
            //if intake isn't deployed, deploy it & vice versa
            //check if intake needs to be reversed and then deploy or retract

//end of intake controls

//arm control section (GP2, X, Dpad Down)
            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(!gamepad2.x){
                toggleReadyDown=true;
            }
            if(!gamepad2.dpad_up){
                toggleReadyUp=true;
            }
            //end of arm toggle checks

            //adds 1 to bumpCount if x isn't held down
            if(gamepad2.x && toggleReadyDown){
                toggleReadyDown=false;
                if(bumpCount<3) {
                    bumpCount += 1;
                }
            }

            //removes 1 from bumpCount if dpad up isn't held down
            if(gamepad2.dpad_up && toggleReadyUp){
                toggleReadyUp=false;
                if(bumpCount>1) {
                    bumpCount -= 1;
                }
            }
            if(!gamepad2.y){
                TSEtoggle=true;
            }
            if(gamepad2.y&&TSEtoggle&&!isDeployed){
                TSEtoggle=false;
                TSEMode=!TSEMode;
            }

            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
                mechControl.resetIntake();
                intakeDown=false;
            }

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
            if (gamepad2.dpad_left) {
                bumpCount = 0;
                isDeployed = false;
                mechControl.moveToZero(false);
                turretPreset=0;
            }
//end of arm controls

            if (gamepad1.right_bumper) {
                turretControl.setTurretRotation(-0.2);
            } else if (gamepad1.left_bumper) {
                turretControl.setTurretRotation(0.2);
            } else {
//                turretControl.setTurretRotation(0);
            }

            //turret control section (GP2, left stick)
            if(!intakeDown){
                if(!gamepad2.left_bumper&&!gamepad2.right_bumper){
                    turretToggle=true;
                }
                if(turretToggle&&gamepad2.left_bumper&&turretPreset<robot.TURRET_INCREMENTS){
                    turretToggle=false;
                    turretPreset++;
                }else if(turretToggle&&gamepad2.right_bumper&&turretPreset>-robot.TURRET_INCREMENTS){
                    turretToggle=false;
                    turretPreset--;
                }
                if(gamepad2.right_stick_button){
                    turretPreset=0;
                }
            }
            turretPosition = turretPreset * robot.TURRET_STEP;
            //apply angle to turret PID
            turretPosition = Range.clip(turretPosition, -robot.TURRET_MAX_POSITION, robot.TURRET_MAX_POSITION);

            turretControl.setTargetPosition(turretPosition);
//end of turret control section




/**            if(!intakeDown){
                if(gamepad2.left_bumper){
                    turretPreset++;
                }else if(gamepad2.right_bumper){
                    turretPreset--;
                }

                if(gamepad2.right_stick_button){
                    turretPreset=0;
                }
            }

            turretPreset = Range.clip(turretPreset, -robot.TURRET_MAX_POSITION, robot.TURRET_MAX_POSITION);
            turretPosition = turretPreset;
//            turretPosition=turretPreset*robot.TURRET_STEP;

            //apply angle to turret PID

            turretControl.setTargetPosition(turretPosition);
 **/
//end of turret control section

//bucket control section (GP2, Dpad Right)
            if(gamepad2.dpad_right&&!TSEMode){
                if(bumpCount==1){
                    bucketAngle=0.15;
                }else if(bumpCount==2){
                    bucketAngle=0.05;
                }else {
                    bucketAngle = 0.25;
                }
            }else if(gamepad2.dpad_right&&bumpCount==3&&TSEMode){
                bucketAngle=0;
            }else{
                if(intakeDown){
                    bucketAngle=0.4;
                }else if(bumpCount==1&&!TSEMode&&robot.motorArmAngle1.getCurrentPosition()<750) {
                    bucketAngle = 0.6;
                }else if(bumpCount==2&&!TSEMode){
                    bucketAngle=0.55;
                }else if(bumpCount==3&&!TSEMode){
                    bucketAngle=0.75;
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

            telemetry.addData("Current Error = ", turretControl.error);
            telemetry.addData("Turret Last Error = ", turretControl.lastError);
            telemetry.addData("Rotation Speed = ", turretControl.rotationSpeed);
            telemetry.addData("Turret Current Angle: ",robot.turrentEncoder.getCurrentPosition());
            telemetry.addData("Turret Target Angle: ",turretPosition);
            telemetry.addData("TSE MODE: ",TSEMode);
            telemetry.addData("","");
            telemetry.addData("Last Intake Servo Pos",robot.bucketDump.getPosition());
            telemetry.addData("Turret Preset: ",turretPreset);
            telemetry.addData("Chainsaw Power: ",chainsawPower);
            telemetry.addData("Happy Driving ",")");
            telemetry.update();
        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();                 // shut down the arm control thread
        turretControl.stop();               // shut down the turret control thread
        requestOpModeStop();                // shut down the opmode
    }   // end of runOpMode method

}   // end of TeleOp.java class


