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
// change comments
/*
12/07/21 added  back code to drive to goal, changed bucket dump setting,
         added SCORING_POSITION state to initialization. use dpad to set level
*/
package org.firstinspires.ftc.teamcode.OpModes;

//
// import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.AutoParams;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.Threads.AutoMechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

import java.util.List;

@Autonomous(name="CRI Code Builder", group="Development")
@Disabled
public class CRI_TEST_Code extends LinearOpMode {

    public static final String TFOD_MODEL_ASSET = "PP_FF_TSEv3-Green.tflite";
    public static final String[] LABELS = {
            "TSEv3"
    };

    private DataLogger Dl;
    private static final String VUFORIA_KEY =
            "ARLYRsf/////AAABmWpsWSsfQU1zkK0B5+iOOr0tULkAWVuhNuM3EbMfgb1+zbcOEG8fRRe3G+iLqL1/iAlTYqqoLetWeulG8hkCOOtkMyHwjS/Ir8/2vUVgC36M/wb9a7Ni2zuSrlEanb9jPVsNqq+71/uzTpS3TNvJI8WeICQNPAq3qMwmfqnCphVlC6h2ZSLsAR3wcdzknFmtpApdOp1jHJvITPeD/CMdAXjZDN0XJwJNQJ6qtaYSLGC23vJdQ2b1aeqnJauOvswapsG7BlmR7m891VN92rNEcOX7WmMT4L0JOM0yKKhPfF/aSROwIdNtSOpQW4qEKVjw3aMU1QDZ0jj5SnRV8RPO0hGiHtXy6QJcZsSj/Y6q5nyf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean blueAlliance = false;       //red if false, blue if true

    /* Declare OpMode members. */
    private HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State setupState = State.ALLIANCE_SELECT;     // default setupState configuration
    private State runState = State.SET_DISTANCES;
    private DriveClass drive = new DriveClass(robot, opMode);

    /* Declare DataLogger variables */
    private String action = "";


    @Override
    public void runOpMode() {


        boolean running = true;
        long startDelay = 0;
        double timeElapsed;

        // set default values
        int scoreLevel = 1;

        double hubDistance = 0;
        double turnError = 2;
        double bucketAngle = 0.3;
        double turretPosition = 0;

        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        boolean turretToggle=false;
        boolean TSEMode=false;


        //No bonus elements if false, bonus elements if true
        boolean bonusElements = false;

        //carousel if false, warehouse if true
        String fieldPosition = "warehouseSide";

        // warehouse park
        boolean warehousePark = true;
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        //mechanism control thread
        AutoMechControlLibrary mechControl = new AutoMechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);

        //turret control thread
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);

        AutoParams params = new AutoParams();

        telemetry.addData("Robot State = ", "Control Threads initialized");
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        telemetry.addData("Robot State = ", "Vuforia/Tensorflow initialized");
        telemetry.update();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Robot State = ", "Hardware initialized");
        telemetry.update();

        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        // initialize servos
        armControl.initArms();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(0.4);

        // reset the sweeper bar to stored position
        drive.resetTSEBar();

        telemetry.addData("Robot State = ", "Servos initialized");
        telemetry.update();

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // monitor the position of the TSE on the field
        while(!opModeIsActive() && running){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLeft() < 150) {
                            scoreLevel = 1;
                        } else if (recognition.getLeft() > 150 && recognition.getLeft() < 400 ) {
                            scoreLevel = 2;
                        } else if (recognition.getLeft() > 400) {
                            scoreLevel = 3;
                        }
                        i++;
                        if (updatedRecognitions.size() == 0) scoreLevel = 1;
                    }     // if (Recognition...
                    // Send telemetry message to signify robot waiting;
                    telemetry.addData("Robot Status : ", "READY TO RUN");    //
                    telemetry.addData("Scanning for : ", "TSE");
                    if (scoreLevel == 1){
                        telemetry.addData("Detected Level = ","Bottom");
                    }else if (scoreLevel == 2){
                        telemetry.addData("Detected Level = ","Middle");
                    } else{
                        telemetry.addData("Detected Level = ","Top");
                    }
                    telemetry.addData("Press X to : ", "ABORT Program");
                    telemetry.update();
                }   // if (updatedRecog...)
            }   // end of if (tfod != null)

            if(gamepad1.x || gamepad2.x) running = false;   // abort the program
        }   // end of while(!opModeIsActive...

        if(!running) requestOpModeStop();   // user requested to abort setup

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        mechController.start();
        turretController.start();

        runtime.reset();

        int nextAction = 1;

        while (opModeIsActive() && (running)) {
            robot.motorArmAngle1.setTargetPosition(0);
            robot.motorArmAngle2.setTargetPosition(0);
            switch(runState){
                case TEST:
                    drive.driveTurn(-90, 2);
                    runState = State.HALT;
                    break;
                case SET_DISTANCES:
                    // Setup parameters per settings
                    params.initParams(blueAlliance, fieldPosition);

                    runState = State.MIDDLE_SCORE_SHIPPING;
                    break;

                case CAROUSEL_TEST2:
                    // drop TSE bar to clear the TSE
                    drive.deployTSEBar();

                    // drive forward into position to push the TSE out of the way
                    drive.driveStraightPID(-34);

                    if(nextAction == 1){
                        runState = State.CAROUSEL_TEST_DUCK;
                    } else {
                        runState = State.CAROUSEL_SCORE_SHIPPING;
                    }

                    break;

                case MIDDLE_SCORE_SHIPPING:

                    // drive forward towards the alliance shipping hub
//                    drive.driveStraightPID(-10);


                    drive.driveStraight(-0.4, 5);
                    // rotate towards the Alliance Hub
                    drive.driveTurn(35, 2);

                    // drive forward towards the alliance shipping hub
                    drive.driveStraight(-0.4, 5);


                    scoreLevel = 2;

                    //place the shipping element in the correct level
                    if(scoreLevel == 1) {
                        mechControl.scoringPos1();
                        sleep(750);
                    } else if(scoreLevel == 2) {

                        mechControl.scoringPos2();
                        sleep(750);
                    } else {
                        mechControl.scoringPos3();
                        sleep(750);
                    }

                    // drive forward towards the alliance shipping hub
                    drive.driveStraight(-0.4, 5);

                    drive.bucketDump();
                    sleep(350);

                    // back away from the hub
                    drive.driveStraight(0.6, 5);

                    mechControl.moveToZero();

                    // Turn towards the wall
                    drive.driveTurn(0, 2);

                    // drive forward towards the outside wall
                    drive.driveStraight(0.6, 5);

                    // Turn towards the carousel
                    drive.driveTurn(90, 4);
                    drive.driveTurn(90, 2);

                    // drive towards the barricade and prepare to go over it
//                    drive.driveTime(0.25, 50);
                    drive.setDrivePower(0.25, 0.25, 0.25, 0.25);
                    sleep(750);
                    drive.setDrivePower(0,0,0,0);

                    // halt the drive motors
                    drive.motorsHalt();

                    runState = State.MIDDLE_DUCK_CAROUSEL;

                    break;

                case MIDDLE_DUCK_CAROUSEL:

                    drive.driveStraight(1, 20);

                    // realign position towards duck carousel
                    drive.driveTurn(96, 2);

                    // drive closer to the duck carousel
                    drive.driveStraight(0.3, 35);

                    robot.motorChainsaw.setPower(-robot.CHAIN_POW*0.75);

                    drive.driveTurn(25, 2);
                    drive.setDrivePower(0.2, 0.2, 0.2, 0.2);

                    sleep(3500);

                    /*

                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 32) && opModeIsActive()&&((runtime.time()-timeElapsed)<1.00))) {
                        if (((runtime.time()-timeElapsed)>1.5)){
                            break;
                        }


                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                        //   telemetry.addData("elapsed time = ", runtime.time()-timeElapsed);
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor
                    */


                    drive.motorsHalt();

                    runState = State.HALT;

                    break;

                case CAROUSEL_SCORE_SHIPPING:
                    // turn off the chainsaw
                    robot.motorChainsaw.setPower(0);

                    // reset the TSE bar so that we don't bump into anything
                    drive.resetTSEBar();

                    drive.driveStraight(0.5, 5);

                    // turn towards alliance shipping hub
                    drive.driveTurn(30, 2);

                    // drive towards the rough terrain
                    drive.driveStraight(-0.3,20);

                    // drive over the rough terrain
                    drive.driveStraight(-1,19);
                    drive.motorsHalt();

                    // correct the orientation of the robot
                    drive.driveTurn(90, 2);

                    // drive back towards the bar to reset position
                    drive.driveTime(0.2, 1);

                    // halt the drive motors
                    drive.motorsHalt();

                    runState = State.CAROUSEL_TEST4;

                    break;

                case CAROUSEL_TEST3:
                    //Make sure turret is in the right position and then deploy intake
                    intakeDown = true;
                    isDeployed = true;
                    if(Math.abs(robot.turrentEncoder.getCurrentPosition())<=5) {
                        mechControl.setArmAction(1);
                    } else {
                        // do we need to put the turret in the right place???
                        telemetry.addData("ERROR: ", "Turret is not in the correct location");
                        telemetry.update();
                        sleep(5000);
                    }
                    sleep(1000);

                    drive.setDrivePower(0.2,0.2, 0.4, 0.4);


                    // collect element
//                    drive.setDrivePower(0.2, 0.2, 0.2, 0.2);
//                    sleep(2000);
//                    drive.motorsHalt();
                    while(robot.motorR1.getCurrentPosition() < 1000){
                        while(robot.bucketSensor.getDistance(DistanceUnit.MM) > 120){

                        }

                    }

                    drive.driveTime(0.3, 1.5);
                    drive.motorsHalt();

                    // shut off intake and return arm to correct position

                    mechControl.setArmAction(2);    // turn intake off
                    sleep(750);
                    mechControl.setArmAction(0);    // disable intake

                    runState = State.HALT;
                    break;

                case CAROUSEL_TEST4:
                    //place the shipping element in the correct level
                    if(scoreLevel == 1) {
                        mechControl.scoringPos1();
                        sleep(750);
                        drive.bucketDump();
                        sleep(250);
                        mechControl.moveToZero();
                        sleep(2000);
                    } else if(scoreLevel == 2) {

                        mechControl.scoringPos2();
                        sleep(750);
                        drive.bucketDump();
                        sleep(250);
                        mechControl.moveToZero();
                        sleep(2000);
                    } else {
                        mechControl.scoringPos3();
                        sleep(750);
                        drive.bucketDump();
                        sleep(250);
                        mechControl.moveToZero();
                        sleep(2000);
                    }

                    // drive back over the obstacle bar
                    drive.driveTime(0.8, 1);

                    drive.driveTime(0.3, 3);
                    drive.motorsHalt();

                    // cancel out of the program
                    runState = State.HALT;

                    break;

                case CAROUSEL_TEST_DUCK:
                    // turn towards duck for scoring position
                    drive.driveTurn(30, 2);

                    // drive towards the duck carousel
                    drive.driveStraightPID(25);

                    // drive slowly towards the duck carousel
                    drive.setDrivePower(0.25, 0.25,0.25,0.25);
                    // turn on the chainsaw
                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 32) && opModeIsActive()&&((runtime.time()-timeElapsed)<1.00))) {
                        if (((runtime.time()-timeElapsed)>1.5)){
                            break;
                        }
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(-robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                        //   telemetry.addData("elapsed time = ", runtime.time()-timeElapsed);
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                    sleep(2500);

                    drive.motorsHalt();

                    // straighten the robot out
                    drive.driveTurn(30, 2);

                    // drive back to position to score the shipping element
                    drive.driveStraightPID(-32);

                    runState = State.CAROUSEL_SCORE_SHIPPING;

                    break;

                case TEST_BALANCED:
                    // move arm into drive position
                    mechControl.moveToZero();
                    sleep(500);

                    //Make sure turret is in the right position and then deploy intake
                    intakeDown = true;
                    isDeployed = true;
                    if(Math.abs(robot.turrentEncoder.getCurrentPosition())<=5) {
                        mechControl.setArmAction(1);
//                        robot.motorIntake.setPower(robot.INTAKE_POW);
                    } else {
                        // do we need to put the turret in the right place???
                        telemetry.addData("ERROR: ", "Turret is not in the correct location");
                        telemetry.update();
                        sleep(5000);
                    }

                    // collect element
                    drive.setDrivePower(0.2, 0.2, 0.2, 0.2);
                    sleep(2000);
                    drive.motorsHalt();

                    // shut off intake and return arm to correct position

                    mechControl.setArmAction(2);    // turn intake off
                    sleep(750);
                    mechControl.setArmAction(0);    // disable intake

                    // move to scoring position
                    drive.setDrivePower(-0.2, -0.2, -0.2, -0.2);

                    // move arm to scoring position while moving towards the hub
                    robot.bucketDump.setPosition(0.75);
                    turretPosition = 0.4;

                    mechControl.scoringPos3();
                    turretControl.setTargetPosition((int)(turretPosition * robot.TURRET_MAX_POSITION));

                    sleep(2000);
                    drive.motorsHalt();

                    // dump the block
                    telemetry.addData("Status: ", "Bucket Dump");
                    telemetry.update();
                    sleep(2000);
                    drive.bucketDump();

                    // put the arm back in place
                    mechControl.moveToZero();

                    // go pick up another block

                    runState = State.HALT;

                    break;






                case LEVEL_ADJUST:
                    telemetry.addData("Working on LEVEL_SELECT = ", "Now");
                    telemetry.addData("TSE Position = ", scoreLevel);
                    telemetry.update();

                    runState = State.SLEEP_DELAY;
                    break;

                case SLEEP_DELAY:
                    telemetry.addData("Working on Sleep Delay = ", "Now");
                    telemetry.update();

                    // sleep for set delay time. turn chainsaw on to show robot isn't dead
                    robot.motorChainsaw.setPower(0.2);
                    sleep(startDelay);
                    robot.motorChainsaw.setPower(0);

                    // move the TSE out of the way
                    runState = State.MOVE_TSE_STRAIGHT; // default to STRAIGHT

                    // If the TSE is straight in front of the robot, go straight, otherwise, arc
                    if(fieldPosition != "warehouseSide") {
                        if (!blueAlliance && scoreLevel == 1){  // red alliance, level 1
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                        if(blueAlliance && scoreLevel == 3){
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                    } else {
                        if (blueAlliance && scoreLevel == 1){  // red alliance, level 1
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                        if(!blueAlliance && scoreLevel == 3){
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                    }// end of if(!warehouseSide)

                    // deploy sweeper bar
                    drive.deployTSEBar();

                    break;

                case MOVE_TSE_STRAIGHT:

                    // if the TSE is not in front of the robot, arc turn to move it out of the way
                    //drive forward and push TSE out of the way
                    drive.driveStraight(params.reverseSpeed, params.tseDistance);
                    sleep(250);

                    //back up to turn to the shipping hub
                    drive.driveStraight(params.forwardSpeed, params.tseReturnDist);
                    sleep(250);

                    runState = State.X_SCORE;       // score in the hub
                    break;

                case MOVE_TSE_ARC:

                    // if the TSE is not in front of the robot, arc turn to move it out of the way
                    //drive forward and push TSE out of the way

                    drive.driveArcTurn(params.powerLeft, params.powerRight, params.arcTime);
                    sleep(250);

                    //Drive into position to approach the hub
                    drive.driveStraight(params.forwardSpeed, params.arcTurnReturn);
                    sleep(250);

                    // realign the robot to face forward
                    /*
                     ***** This code should be unnecessary for this portion
                    drive.driveTurn(0, params.turnError);

                    // drive forward to get to scoring position
                    drive.driveStraight(params.forwardSpeed, -7);
                    drive.driveStraight(params.forwardSpeed, -params.forwardDistance);

                    */

                    runState = State.X_SCORE;       // score in the hub
                    break;

                case X_SCORE:
                    turnError = 2;

                    telemetry.addData("Working on X_Score = ", "Now");
                    telemetry.update();

                    //turn towards the hub
                    drive.driveTurn(params.turnAngle, params.turnError);

                    // drive over barrier
                    drive.driveStraight(1, 20);

                    // straighten the bot
                    drive.driveTurn(params.turnAngle, params.turnError);

                    // realign position by reversing into the bar
                    drive.driveTime(-0.2, params.driveByTime);

                    //move arm to scoring positions
                    if(scoreLevel ==1){             //bottom
                        mechControl.scoringPos3();
                        hubDistance = params.hubDistance1;
                        bucketAngle = params.bucketAngle1;
                    }else if(scoreLevel ==2){       // middle
                        mechControl.scoringPos2();
                        hubDistance = params.hubDistance2;
                        bucketAngle = params.bucketAngle2;
                    }else if(scoreLevel ==3){       // top
                        mechControl.scoringPos1();
                        hubDistance = params.hubDistance3;
                        bucketAngle = params.bucketAngle3;
                    }
                    sleep(500);

                    //drive towards the shipping hub to score
                    drive.driveStraight(params.reverseSpeed, hubDistance);
                    sleep(350);

                    //dump bucket
                    robot.bucketDump.setPosition(bucketAngle);
                    sleep(500);     // allow time to dump the cube

                    // drive away from the alliance hub
                    drive.driveStraight(params.forwardSpeed, (hubDistance+params.extraDistance));

                    //reset arms
                    robot.bucketDump.setPosition(0.5);
                    mechControl.moveToZero();


                    // reset the sweeper bar
                    drive.resetTSEBar();

                    runState = State.HALT;
                    break;



                case HALT:

                    mechControl.moveToZero();          // reset the arm to the right initialized position
                //    turretControl.resetTurret();    // reset the arm to the right initialized position

                    // shut down all motors
                    robot.motorChainsaw.setPower(0);
                    robot.motorIntake.setPower(0);
                    drive.motorsHalt();

                    running = false;        // exit the program loop
                    mechControl.stop();
                    turretControl.stop();
                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        mechControl.stop();
        turretControl.stop();

        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        MIDDLE_TEST, MIDDLE_SCORE_SHIPPING, MIDDLE_DUCK_CAROUSEL, TEST, CAROUSEL_TEST2, CAROUSEL_TEST3, CAROUSEL_TEST4, CAROUSEL_TEST_DUCK, CAROUSEL_SCORE_SHIPPING, ALLIANCE_SELECT, DELAY_LENGTH, FIELD_SIDE_SELECT, SELECT_BONUS, VERIFY_CONFIG, TEST_CONFIG, TEST_BALANCED,
        SLEEP_DELAY, LEVEL_ADJUST, MOVE_TSE_STRAIGHT, MOVE_TSE_ARC, X_SCORE, RED_CAROUSEL, BLUE_CAROUSEL, BONUS_SCORES,
        BLUE_WAREHOUSE_BONUS, RED_WAREHOUSE_BONUS, STORAGE_PARK, WAREHOUSE_PARK, HALT, SELECT_PARK,
        SET_DISTANCES
    }   // end of enum State

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    public void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime:       ");
        Dl.addField("Alliance:      ");
        Dl.addField("State:         ");
        Dl.addField("Action:        ");
        Dl.addField("Gyro value:    ");
        Dl.addField("Dist. Sensor:  ");
        Dl.addField("L1 Encoder:    ");
        Dl.addField("L2 Encoder:    ");
        Dl.addField("R1 Encoder:    ");
        Dl.addField("R2 Encoder:    ");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    public void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        if(blueAlliance) {
            Dl.addField("Blue Alliance");
        } else {
            Dl.addField("Red Alliance");
        }
        Dl.addField(String.valueOf(runState));
        Dl.addField(String.valueOf(action));
        Dl.addField(String.valueOf(drive.getZAngle()));
        Dl.addField(String.valueOf(robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM)));
        Dl.addField(String.valueOf(robot.motorL1.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorL2.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorR1.getCurrentPosition()));
        Dl.addField(String.valueOf(robot.motorR2.getCurrentPosition()));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }
}
