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
package org.firstinspires.ftc.teamcode.TestCode;

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
import org.firstinspires.ftc.teamcode.Libs.DriveClassLogging;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

import java.util.List;

@Autonomous(name="State Auto Logging", group="Test")
@Disabled
public class StateFullAutoLogging extends LinearOpMode {

    public static final String TFOD_MODEL_ASSET = "PP_FF_TSEv3-Green.tflite";
    public static final String[] LABELS = {
            "TSEv3"
    };

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

    @Override
    public void runOpMode() {

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /* Declare OpMode members. */
        HardwareProfile robot   = new HardwareProfile();
        LinearOpMode opMode = this;
        ElapsedTime   runtime = new ElapsedTime();
        State setupState = State.ALLIANCE_SELECT;     // default setupState configuration
        State runState = State.SET_DISTANCES;
        boolean debugMode = true;

        // do we need these threads in an autonomous opmode?
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
        AutoParams params = new AutoParams();

        boolean autoReady = false;
        boolean running = true;
        double startTime;
        String goalPosition = "";
        long startDelay = 0;
        double timeElapsed;

        // set default values
        int hubFactor = 1;
        int scoreLevel = 1;

        double hubDistance = 0;
        double turnError = 2;
        double bucketAngle = 0.3;

        //red if false, blue if true
        boolean blueAlliance = false;

        //No bonus elements if false, bonus elements if true
        boolean bonusElements = false;

        //carousel if false, warehouse if true
//        boolean warehouseSide = false;
        String fieldPosition = "warehouseSide";

        // warehouse park
        boolean warehousePark = true;

        if(debugMode){
            setupState = State.TEST_CONFIG;           // manually created config for testing only
            scoreLevel = 2;
        }

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

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

        turretController.start();

        DriveClassLogging drive = new DriveClassLogging(robot, opMode);
        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        // initialize servos
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(0.5);

        // reset the sweeper bar to stored position
        drive.resetTSEBar();
        drive.startLogging();

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Allow drivers to setup for the game.

        while (!autoReady){

            switch (setupState) {
                case ALLIANCE_SELECT:
                    telemetry.addData("Drive Motor Encoders:",robot.motorR1.getCurrentPosition());
                    telemetry.addData("Which Alliance are you on?", "");
                    telemetry.addData("Press DPAD_LEFT  == ", " BLUE Alliance");
                    telemetry.addData("Press DPAD_RIGHT  == ", " RED Alliance");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.dpad_left || gamepad1.dpad_right){
                        blueAlliance = gamepad1.dpad_left;
                        setupState = State.DELAY_LENGTH;    // give option to add delay
                        if(blueAlliance){
                            telemetry.addData("Alliance ==  ", "BLUE");
                        } else {
                            telemetry.addData("Alliance ==  ", "RED");
                        }
                        telemetry.update();
                        sleep(2000);
                    }   // end of if(gamepad1.x...
                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }
                    break;

                case DELAY_LENGTH:
                    telemetry.addData("Add a Delay", "");
                    telemetry.addData("Press DPAD_UP  ", " To Increase Delay");
                    telemetry.addData("Press DPAD_DOWN ", " To Reduce Delay");
                    telemetry.addData("Current Delay = ", startDelay);
                    telemetry.addData(" ","");
                    telemetry.addData("Press A to continue", "");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.dpad_up && (runtime.time()-timeElapsed) >0.3) {
                        if(startDelay < 10){       // limit the max delay to 10 seconds
                            startDelay = startDelay + 1;
                        }
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up ...

                    if(gamepad1.dpad_down && (runtime.time()-timeElapsed) >0.3) {
                        if(startDelay > 0){       // confirm that the delay is not negative
                            startDelay = startDelay - 1;
                        } else startDelay = 0;
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_down...

                    if(gamepad1.a){         // exit the setup
                        setupState = State.FIELD_SIDE_SELECT;
                        startDelay = startDelay * 1000;
                        telemetry.addData("# of Seconds Delay == ", startDelay);
                        telemetry.update();
                        sleep(2000);
                    }   // end of if(gamepad1.a...

                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }

                    break;

                case FIELD_SIDE_SELECT:
                    telemetry.addData("What side of the field?", "");
                    telemetry.addData("Press DPAD Left  == ", " CAROUSEL Side");
                    telemetry.addData("Press DPAD Right == ", " WAREHOUSE Side");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.dpad_left || gamepad1.dpad_right){
//                        warehouseSide = !gamepad1.dpad_left;

                        if(fieldPosition != "warehouseSide") {
                            telemetry.addData("Side of the field == ", "CAROUSEL Side");
                            setupState = State.SELECT_PARK;
                        } else {
                            telemetry.addData("Side of the field == ", "WAREHOUSE Side");
                            setupState = State.SELECT_BONUS;
                        }
                        telemetry.update();
                        sleep(2000);
                    }   // end of if(gamepad1.dpad_left...

                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }

                    break;

                case SELECT_BONUS:
                    telemetry.addData("Score Bonus Elements?", "");
                    telemetry.addData("Press A == ", " YES");
                    telemetry.addData("Press B == ", " NO");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.a || gamepad1.b){
                        bonusElements = gamepad1.a;

                        if(bonusElements) {
                            telemetry.addData("Score bonus elements == ", "YES");
                        } else {
                            telemetry.addData("Score bonus elements == ", "NO");
                        }
                        setupState = State.SELECT_PARK;
                        telemetry.update();
                        sleep(2000);
                    }   // end of if(gamepad1.a...

                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }
                    break;

                case SELECT_PARK:
                    telemetry.addData("Where to park?", "");
                    telemetry.addData("Press DPAD Up  == ", " Park in Warehouse");
                    telemetry.addData("Press DPAD Down == ", " Park in Storage");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.dpad_up || gamepad1.dpad_down){
                        //sleep(1000);
                        warehousePark = gamepad1.dpad_up;
                        if(warehousePark) {
                            telemetry.addData("Park Location == ", "WAREHOUSE");
                        } else {
                            telemetry.addData("Park Location == ", "STORAGE");
                        }
                        telemetry.update();
                        sleep(2000);
                        setupState = State.VERIFY_CONFIG;
                        //red carousel
                        if (!blueAlliance && (fieldPosition != "warehouseSide")) {
                            hubFactor = 1;
                            //blue carousel
                        } else if (blueAlliance && (fieldPosition != "warehouseSide")) {
                            hubFactor = -1;
                            //red warehouse
                        } else if (!blueAlliance && (fieldPosition == "warehouseSide")) {
                            hubFactor = -1;
                            //blue warehouse
                        } else {
                            hubFactor = 1;
                        }
                        if (hubFactor == 1) {
                            goalPosition = "Right";
                        } else {
                            goalPosition = "Left";
                        }

                    }   // end of if(gamepad1.dpad_left...

                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }
                    break;

                case VERIFY_CONFIG:
                    telemetry.addData("Verify the setup", "");
                    if(blueAlliance){
                        telemetry.addData("Alliance          == ", "BLUE");
                    } else {
                        telemetry.addData("Alliance          == ", "RED");
                    }   // end of if(alliance)
                    telemetry.addData("Start Delay == ", startDelay);
                    if(fieldPosition != "warehouseSide"){
                        telemetry.addData("Side of the field ==  ", "WAREHOUSE");
                    } else {
                        telemetry.addData("Side of the field ==  ", "CAROUSEL");
                    }   //end of if(fieldSide)
                    telemetry.addData("Goal position ==  ", goalPosition);
                    if(warehousePark) {
                        telemetry.addData("Park Location == ", "WAREHOUSE");
                    } else {
                        telemetry.addData("Park Location == ", "STORAGE");
                    }
                    telemetry.addData("","");
                    telemetry.addData("","");
                    telemetry.addData("Press A to Confirm or B to start over","");
                    telemetry.addData("Press X to ","ABORT");
                    telemetry.update();

                    if(gamepad1.b || gamepad1.a){
                        if (gamepad1.b){
                            autoReady = false;
                            startDelay = 0;     // reset the start delay
                            setupState = State.ALLIANCE_SELECT;
                        }  else {
                            autoReady = true;
                        }   // end of if (gamepad1.b)
                    }   // end of if(gamepad1.b || gamepad1.a)
                    if(gamepad1.x) {
                        running = false;
                        autoReady = true;
                    }   // end of if(gamepad1.x || gamepad2.x)
                    break;

                case TEST_CONFIG:
                    blueAlliance = false;        // true for blue, false for red
                    startDelay = 0;          // put start delay in ms
                    fieldPosition = "warehouseSide";       // true for warehouse, false for carousel
                    warehousePark = false;   // true for warehouse, false for storage
                    hubFactor = 1;   // red carousel

                    if(blueAlliance){
                        telemetry.addData("Alliance          == ", "BLUE");
                    } else {
                        telemetry.addData("Alliance          == ", "RED");
                    }   // end of if(alliance)
                    telemetry.addData("Start Delay == ", startDelay);
                    if(fieldPosition == "warehouseSide"){
                        telemetry.addData("Side of the field ==  ", "WAREHOUSE");
                    } else {
                        telemetry.addData("Side of the field ==  ", "CAROUSEL");
                    }   //end of if(fieldSide)
                    if(warehousePark) {
                        telemetry.addData("Park Location == ", "WAREHOUSE");
                    } else {
                        telemetry.addData("Park Location == ", "STORAGE");
                    }
                    telemetry.addData("","");
                    telemetry.addData("","");
                    telemetry.addData("Press A to Confirm","");
                    telemetry.addData("Press X to ","ABORT");
                    telemetry.update();

                    if(gamepad1.a){
                        autoReady = true;
                    }   // end of if(gamepad1.a...

                    if(gamepad1.x){
                        autoReady = true;
                        running = false;
                    }   // end of if(gamepad1.x...
                    break;
            }   // end of switch(setupState)
        }   // end of while(autoReady)

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

        if(debugMode){
            runState = State.SET_DISTANCES;

            telemetry.addData("> ","DEBUG MODE ENABLED ");
            telemetry.addData("> runState = ",runState);
            telemetry.addData("> scoreLevel = ",scoreLevel);
            telemetry.update();
        }   // end if(debugMode)

        runtime.reset();

        startTime = runtime.time() + startDelay;

        while (opModeIsActive() && (running)) {

            switch(runState){
                case SET_DISTANCES:
                    // Setup parameters per settings
                    params.initParams(blueAlliance, fieldPosition);

                    runState = State.LEVEL_ADJUST;
                    break;

                case LEVEL_ADJUST:
                    if(debugMode) {
                        telemetry.addData("TSE Position = ", scoreLevel);
                        telemetry.addData("Working on LEVEL_SELECT = ", "Now");
                        telemetry.update();
                    } else {
                        telemetry.addData("TSE Position = ", scoreLevel);
                        telemetry.update();
                    } // end if(debugMode)

                    runState = State.SLEEP_DELAY;
                    break;

                case SLEEP_DELAY:
                    if(debugMode) {
                        telemetry.addData("Working on Sleep Delay = ", "Now");
                        telemetry.update();
//                        startDelay = 2000;
                    }   // end if(debugMode)
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
                    }   // end of if(!warehouseSide)

                    // deploy sweeper bar
                    drive.deployTSEBar();

                    break;

                case MOVE_TSE_STRAIGHT:
                    // if the TSE is not in front of the robot, arc turn to move it out of the way
                    //drive forward and push TSE out of the way
                    drive.driveStraight(params.forwardSpeed, params.tseDistance);
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

                    //Return to starting position
                    drive.driveArcTurn(-params.powerLeft, -params.powerRight, params.arcTime);
                    sleep(250);

                    // realign the robot to face forward
                    drive.driveTurn(0, params.turnError);

                    // drive forward to get to scoring position
                    drive.driveStraight(params.forwardSpeed, params.forwardDistance);

                    runState = State.X_SCORE;       // score in the hub
                    break;

                case X_SCORE:
//                    turnError = 2;

                    if(debugMode) {
                        telemetry.addData("Working on X_Score = ", "Now");
                        telemetry.update();
                    }   // end if(debugMode)

                    //turn towards the hub
                    drive.driveTime(params.turnAngle, params.turnError);

                    //move arm to scoring positions
                    if(scoreLevel ==1){             //bottom
                        armControl.scoringPos3();
                        hubDistance = params.hubDistance1;
                        bucketAngle = params.bucketAngle1;
                    }else if(scoreLevel ==2){       // middle
                        armControl.scoringPos2();
                        hubDistance = params.hubDistance2;
                        bucketAngle = params.bucketAngle2;
                    }else if(scoreLevel ==3){       // top
                        armControl.scoringPos1();
                        hubDistance = params.hubDistance3;
                        bucketAngle = params.bucketAngle3;
                    }
                    sleep(500);

                    //drive towards the shipping hub to score
                    drive.driveStraight(params.forwardSpeed, hubDistance);
                    sleep(350);

                    //dump bucket
                    robot.bucketDump.setPosition(bucketAngle);
                    sleep(500);     // allow time to dump the cube

                    // drive away from the alliance hub
                    drive.driveStraight(-params.forwardSpeed, hubDistance);

                    //reset arms
                    robot.bucketDump.setPosition(0.5);
                    armControl.moveToZero();

                    // turn towards outside wall
                    drive.driveTurn(90 * hubFactor, turnError);
                    sleep(350);

                    // drive towards the outside wall
                    while(robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                    drive.motorsHalt();

                    // reset the sweeper bar
                    drive.resetTSEBar();
                    if(blueAlliance && (fieldPosition != "warehouseSide")){        // blue_Carousel
                        runState = State.BLUE_CAROUSEL;
                    } else if(blueAlliance && (fieldPosition == "warehouseSide")){  // blue warehouse
                        if(bonusElements) {
                            runState = State.BLUE_WAREHOUSE_BONUS;
                        } else {
                            runState = State.WAREHOUSE_PARK;
                        }
                    } else if(!blueAlliance && (fieldPosition != "warehouseSide")){  // red carousel
                        runState = State.RED_CAROUSEL;
                    } else {
                        if(bonusElements) {             // red_warehouse
                            runState = State.RED_WAREHOUSE_BONUS;
                        } else {
                            runState = State.WAREHOUSE_PARK;
                        }
                    }   // end if(blueAlliance && !warehouseSide)
                    break;

                case RED_CAROUSEL:
                    if(debugMode) {
                        telemetry.addData("Working on RED_Carousel = ", "Now");
                        telemetry.update();
                    }

                    //go to carousel, red
                    //turn to face carousel
                    drive.driveTurn(0, params.turnError);
                    sleep(350);

                    //drive forward until distance sensor is tripped
                    while(robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                    params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(-robot.CHAIN_POW * 0.75);
                    }   // end while(robot.frontDistance...

                    //drive forward at very low power to keep in contact with carousel
                    drive.setDrivePower(0.1,0.1,0.1,0.1);

                    //sleep to wait for carousel to drop duck to the floor
                    sleep(2500);

                    drive.driveStraight(-params.forwardSpeed, 4);

                    //reposition to face carousel again
                    drive.driveTurn(0,params.turnError);

                    //turn off chainsaw
                    robot.motorChainsaw.setPower(0);

                    if(!warehousePark) {
                        runState = State.STORAGE_PARK;
                    }else {
                        runState = State.WAREHOUSE_PARK;
                    }
                    break;

                case BLUE_CAROUSEL:
                    if(debugMode) {
                        telemetry.addData("Working on BLUE Carousel = ", "Now");
                        telemetry.update();
                    }

                    //go to carousel, blue (movements same as red, in reverse

                    // turn towards the carousel
                    drive.driveTurn(0, params.turnError);

                    while(robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM)>30) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                    params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);
                    }   // end of while(robot.frontDistanceSensor

                    // drive slowly into the carousel to apply constant pressure
                    drive.setDrivePower(0.1,0.1,0.1,0.1);
                    sleep(2500);

                    // turn towards the storage
                    drive.driveTurn(0,params.turnError);

                    robot.motorChainsaw.setPower(0);

                    if(!warehousePark) {
                        runState = State.STORAGE_PARK;
                    }else {
                        runState = State.WAREHOUSE_PARK;
                    }

                    break;

                case BLUE_WAREHOUSE_BONUS:
                    if(debugMode) {
                        telemetry.addData("Working on Blue Warehouse = ", "Now");
                        telemetry.update();
                    }   // end of if(debugMode)

                    // turn towards the warehouse
                    drive.driveTurn(90, params.turnError);

                    //drive forward over the barrier into the warehouse
                    drive.driveTime(1, 0.65);

                    //turn towards scoring elements
                    drive.driveTurn(45, params.turnError);

                    // turn on intake
                    armControl.intakeOn();
                    sleep(350);

                    // turn the intake on to take in scoring elements
                    robot.motorIntake.setPower(1);

                    // drive towards the elements to collect an element
                    drive.driveTime(0.3, 1);
                    robot.motorIntake.setPower(-1); // reverse motor to kick out any excess elements
                    drive.driveTime(-0.3, 1);
                    robot.motorIntake.setPower(0);

                    // put the intake away
                    armControl.intakeOff();

                    // turn towards barrier
                    drive.driveTurn(90, params.turnError);

                    // drive towards barrier
                    drive.driveTime(-0.4, 0.5);

                    // drive over barrier
                    drive.driveTime(-1, 0.65);

                    // turn towards hub
                    drive.driveTurn(45, params.turnError);

                    // set the arm in scoring position - Level 1
                    armControl.scoringPos1();

                    // drive towards hub
                    drive.driveStraight(0.3, 10);

                    //dump bucket
                    robot.bucketDump.setPosition(params.bucketAngle3);
                    sleep(500);

                    //reset arms
                    robot.bucketDump.setPosition(0.5);
                    armControl.moveToZero();

                    // drive back to the barrier
                    drive.driveStraight(0.3, -10);

                    // turn towards barrier
                    drive.driveTurn(90, params.turnError);

                    if(runtime.time() > 25) {
                        runState = State.WAREHOUSE_PARK;
                    }
                    break;

                case RED_WAREHOUSE_BONUS:
                    if(debugMode) {
                        telemetry.addData("Working on Red_Bonus = ", "Now");
                        telemetry.update();
                    }
                    // turn towards the warehouse
                    drive.driveTurn(-90, params.turnError);

                    //drive forward over the barrier into the warehouse
                    drive.driveTime(1, 0.65);

                    //turn towards scoring elements
                    drive.driveTurn(-45, params.turnError);

                    // turn on intake
                    armControl.intakeOn();
                    sleep(350);
                    robot.motorIntake.setPower(1);

                    // drive towards the elements to collect an element
                    drive.driveTime(0.3, 1);
                    robot.motorIntake.setPower(-1); // reverse motor to kick out any excess elements
                    drive.driveTime(-0.3, 1);
                    robot.motorIntake.setPower(0);

                    // put the intake away
                    armControl.intakeOff();

                    // turn towards barrier
                    drive.driveTurn(-90, params.turnError);

                    // drive towards barrier
                    drive.driveTime(-0.4, 0.5);

                    // drive over barrier
                    drive.driveTime(-1, 0.65);

                    // turn towards hub
                    drive.driveTurn(-45, params.turnError);

                    // set the arm in scoring position - Level 1
                    armControl.scoringPos1();

                    // drive towards hub
                    drive.driveStraight(0.3, 10);

                    //dump bucket
                    robot.bucketDump.setPosition(params.bucketAngle1);
                    sleep(500);

                    //reset arms
                    robot.bucketDump.setPosition(0.5);
                    armControl.moveToZero();

                    // drive back to the barrier
                    drive.driveStraight(0.3, -10);

                    // turn towards barrier
                    drive.driveTurn(-90, params.turnError);

                    if(runtime.time() > 25) {
                        runState = State.WAREHOUSE_PARK;
                        params.warehouseParkDistance = 20;
                    }
                    break;

                case STORAGE_PARK:
                    if(debugMode) {
                        telemetry.addData("Working on X_Score = ", "Now");
                        telemetry.update();
//                        sleep(5000);
                    }
                    drive.driveStraight(-params.forwardSpeed, 4);
                    drive.driveTurn(15, params.turnError);
                    drive.driveStraight(-params.forwardSpeed,12);

                    runState = State.HALT;
                    break;

                case WAREHOUSE_PARK:
                    if(debugMode) {
                        telemetry.addData("Working on Warehouse Park = ", "Now");
                        telemetry.update();
//                        sleep(5000);
                    }
                    drive.driveStraight(-params.forwardSpeed, 4);
                    sleep(250);

                    // turn towards the warehouse
                    drive.driveTurn(90 * params.hubFactor, params.turnError);
                    drive.driveTurn(90 * params.hubFactor, params.turnError);

                    // turn on the chainsaw so that we know the program is still running
                    robot.motorChainsaw.setPower(0.2);

                    // wait until there is just a few seconds to go park
                    while(startTime - runtime.time() > 3){
                        // do nothing
                    }   // end of while(startTime - runtime.time() > 3)

                    drive.driveStraight(-1 * params.hubFactor, params.warehouseParkDistance);
                    robot.motorChainsaw.setPower(0);
                    runState = State.HALT;
                    break;

                case HALT:
                    if(debugMode) {
                        telemetry.addData("Working on Halt = ", "Now");
                        telemetry.update();
                    }   // if(debugMode)

                    armControl.moveToZero();          // reset the arm to the right initialized position
                    turretControl.resetTurret();    // reset the arm to the right initialized position

                    // shut down all motors
                    robot.motorChainsaw.setPower(0);
                    robot.motorIntake.setPower(0);
                    drive.motorsHalt();

                    running = false;        // exit the program loop
                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        drive.stopLogging();
        turretControl.stop();
        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        ALLIANCE_SELECT, DELAY_LENGTH, FIELD_SIDE_SELECT, SELECT_BONUS, VERIFY_CONFIG, TEST_CONFIG,
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

}
