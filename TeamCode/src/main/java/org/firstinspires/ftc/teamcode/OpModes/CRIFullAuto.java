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
import org.firstinspires.ftc.teamcode.Libs.CRIAutoParams;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.Threads.AutoMechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

import java.util.List;

@Autonomous(name="CRI Auto", group="Competition")
//@Disabled
public class CRIFullAuto extends LinearOpMode {

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
    private State runState = State.INIT_PARAMS;
    private DriveClass drive = new DriveClass(robot, opMode);
    boolean debugMode = false;

    /* Declare DataLogger variables */
    private String action = "";

    @Override
    public void runOpMode() {

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();


        // do we need these threads in an autonomous opmode?
    /*    MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
*/
        CRIAutoParams params = new CRIAutoParams();

        boolean autoReady = false;
        boolean running = true;
        boolean scoreDuck = true;
        double startTime;
        String goalPosition = "";
        long startDelay = 0;
        double currentTime;

        // set default values
        int scoreLevel = 1;

        double hubDistance = 0;
        double turnError = 2;
        double bucketAngle = 0.3;

        //No bonus elements if false, bonus elements if true
        boolean bonusElements = false;

        /*
         *  Field positions
         *  - 1: Warehouse Side
         *  - 2: Middle of Field
         *  - 3: Carousel Side
         */
        int fieldPosition = 1;


        // warehouse park
        boolean warehousePark = true;

        if(debugMode){
            setupState = State.TEST_CONFIG;           // manually created config for testing only
            scoreLevel = 2;
            createDl();
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

    //    turretController.start();

        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        // initialize servos
        armControl.initArms();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(0.5);

        // reset the sweeper bar to stored position
        drive.resetTSEBar();

        currentTime = runtime.time();   // initialize currentTime to confirm button press time

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //mechanism control thread
        AutoMechControlLibrary mechControl = new AutoMechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);

        //turret control thread
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);

        // Allow drivers to setup for the game.

        while (!autoReady){

            switch (setupState) {
                case ALLIANCE_SELECT:
//                    telemetry.addData("Drive Motor Encoders:",robot.motorR1.getCurrentPosition());
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
                    if(debugMode) {
                        logData();
                    }

                    if(gamepad1.dpad_up && (runtime.time()-currentTime) >0.3) {
                        if(startDelay < 10){       // limit the max delay to 10 seconds
                            startDelay = startDelay + 1;
                        }
                        currentTime = runtime.time();
                    } //end of if(gamepad1.dpad_up ...

                    if(gamepad1.dpad_down && (runtime.time()-currentTime) >0.3) {
                        if(startDelay > 0){       // confirm that the delay is not negative
                            startDelay = startDelay - 1;
                        } else startDelay = 0;
                        currentTime = runtime.time();
                    } //end of if(gamepad1.dpad_down...

                    if(gamepad1.a){         // exit the setup
                        setupState = State.FIELD_POSITION_SELECT;
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

                case FIELD_POSITION_SELECT:
                    telemetry.addData("Select Field Position", "");
                    telemetry.addData("Press DPAD_UP  ", " To change selection");
                    telemetry.addData("Press DPAD_DOWN ", " To change selection");
                    if(fieldPosition == 1) {
                        telemetry.addData("Position to Select = ", "Warehouse Side");
                    } else if(fieldPosition ==2){
                        telemetry.addData("Position to Select = ", "Middle of the field");
                    } else {
                        telemetry.addData("Position to Select = ", "Carousel Side");
                    }
                    telemetry.addData(" ","");
                    telemetry.addData("Press A to continue", "");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();
                    if(debugMode) {
                        logData();
                    }

                    if(gamepad1.dpad_up && (runtime.time()-currentTime) >0.3) {
                        if(fieldPosition < 4){       // limit the max delay to 10 seconds
                            fieldPosition = fieldPosition + 1;
                        } else fieldPosition =1;
                        currentTime = runtime.time();
                    } //end of if(gamepad1.dpad_up ...

                    if(gamepad1.dpad_down && (runtime.time()-currentTime) >0.3) {
                        if(fieldPosition > 2){       // confirm that the delay is not negative
                            fieldPosition = fieldPosition - 1;
                        } else fieldPosition = 3;
                        currentTime = runtime.time();
                    } //end of if(gamepad1.dpad_down...

                    if(gamepad1.a && (runtime.time()-currentTime > 0.3)){         // exit the setup
                        if(fieldPosition == 1) {
                            telemetry.addData("Position to Select = ", "Warehouse Side");
                            setupState = State.SELECT_PARK;
                        } else if(fieldPosition ==2){
                            telemetry.addData("Position to Select = ", "Middle of the field");
                            setupState = State.SELECT_PARK;
                        } else {
                            telemetry.addData("Position to Select = ", "Carousel Side");
                            setupState = State.SELECT_CAR_OPTIONS;
                        }
                        telemetry.update();
                        sleep(2000);
                    }   // end of if(gamepad1.a...

                    if(gamepad1.x){
                        running = false;
                        autoReady = true;
                    }

                    break;

                case SELECT_CAR_OPTIONS:
                    telemetry.addData("Score Ducks?", "");
                    telemetry.addData("Press DPAD_LEFT == ", " YES");
                    telemetry.addData("Press DPAD_RIGHT == ", " NO");
                    telemetry.addData("Press X to abort program ","");
                    telemetry.update();

                    if(gamepad1.dpad_left || gamepad1.dpad_right){
                        scoreDuck = gamepad1.dpad_left;

                        if(scoreDuck) {
                            telemetry.addData("Score duck in Auto == ", "YES");
                        } else {
                            telemetry.addData("Score duck in Auto == ", "NO");
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
                    if(fieldPosition == 3) {
                        telemetry.addData("Where to park?", "");
                        telemetry.addData("Press A  == ", " Park in Warehouse");
                        telemetry.addData("Press B == ", " Park in Storage");
                        telemetry.addData("Press X to abort program ", "");
                        telemetry.update();

                        if (gamepad1.a || gamepad1.b) {
                            //sleep(1000);
                            warehousePark = gamepad1.a;
                            if (warehousePark) {
                                telemetry.addData("Park Location == ", "WAREHOUSE");
                            } else {
                                telemetry.addData("Park Location == ", "STORAGE");
                            }
                            telemetry.update();
                            sleep(2000);
                            setupState = State.VERIFY_CONFIG;
                            //red carousel

                        }   // end of if(gamepad1.dpad_left...

                        if (gamepad1.x) {
                            running = false;
                            autoReady = true;
                        }
                    } else {
                        warehousePark = true;
                        setupState = State.VERIFY_CONFIG;
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
                    if(fieldPosition == 1) {
                        telemetry.addData("Position to Select = ", "Warehouse Side");
                    } else if(fieldPosition ==2){
                        telemetry.addData("Position to Select = ", "Middle of the field");
                    } else {
                        telemetry.addData("Position to Select = ", "Carousel Side");
                    }

                    if(scoreDuck) {
                        telemetry.addData("Score duck in Auto == ", "YES");
                    } else {
                        telemetry.addData("Score duck in Auto == ", "NO");
                    }

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
                    fieldPosition = 3;       // true for warehouse, false for carousel
                    warehousePark = false;   // true for warehouse, false for storage

                    if(blueAlliance){
                        telemetry.addData("Alliance          == ", "BLUE");
                    } else {
                        telemetry.addData("Alliance          == ", "RED");
                    }   // end of if(alliance)
                    telemetry.addData("Start Delay == ", startDelay);
                    if(fieldPosition == 1) {
                        telemetry.addData("Position to Select = ", "Warehouse Side");
                    } else if(fieldPosition ==2){
                        telemetry.addData("Position to Select = ", "Middle of the field");
                    } else {
                        telemetry.addData("Position to Select = ", "Carousel Side");
                    }
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

        if(!running) {
            requestOpModeStop();   // user requested to abort setup
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        mechController.start();
        turretController.start();

        if(debugMode){
            runState = State.INIT_PARAMS;

            action = "Initialized Settings";
            logData();      // Write data to the data logger

            telemetry.addData("> ","DEBUG MODE ENABLED ");
            telemetry.addData("> runState = ",runState);
            telemetry.addData("> scoreLevel = ",scoreLevel);
            telemetry.update();
        }   // end if(debugMode)

        runtime.reset();

        while (opModeIsActive() && (running)) {
            robot.motorArmAngle1.setTargetPosition(0);
            robot.motorArmAngle2.setTargetPosition(0);
            switch(runState){
                case TEST:
                    drive.driveTurn(-90, 2);
                    runState = State.HALT;
                    break;
                case INIT_PARAMS:
                    // Setup parameters per settings
                    params.initParams(blueAlliance, fieldPosition);

                    if (debugMode) {
                        logData();      // Write data to the data logger
                    }

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
                    }   // end if(debugMode)
                    // sleep for set delay time. turn chainsaw on to show robot isn't dead
                    robot.motorChainsaw.setPower(0.2);
                    sleep(startDelay);
                    robot.motorChainsaw.setPower(0);

                    // If the TSE is straight in front of the robot, go straight, otherwise, arc
                    if(fieldPosition == 1) {
                        // move the TSE out of the way
                        runState = State.MOVE_TSE_STRAIGHT; // default to STRAIGHT

                        if (!blueAlliance && scoreLevel == 1){  // red alliance, level 1
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                        if(blueAlliance && scoreLevel == 3){
                            runState = State.MOVE_TSE_ARC; // default to STRAIGHT
                        }
                    } else if (fieldPosition == 2) {
                        runState = State.MIDDLE_SCORING;
                    } else {
                        runState = State.CAROUSEL_SCORING;
                    } // end of if(fieldPosition == 3)

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

                    runState = State.WAREHOUSE_SCORE;       // score in the hub
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

                    runState = State.WAREHOUSE_SCORE;       // score in the hub
                    break;

                /*********************************************************************
                 * *******************************************************************
                 * ******* B E G I N N I N G    O F     C A R O U S E L **************
                 * *******************************************************************
                 ********************************************************************/

                case CAROUSEL_SCORING:
                    // drive forward to push the TSE out of the way
                    drive.driveStraight(-0.5, 30);

                    // turn towards alliance shipping hub
                    drive.driveTurn(params.hubFactor * 90, params.turnError);

                    // drive forward towards the obstacle
                    drive.driveStraight(-0.5, 15);

                    // store sweeper bar
                    drive.resetTSEBar();

                    // drive towards the barricade and prepare to go over it
                    drive.setDrivePower(-0.20, -0.20, -0.20, -0.20);
                    sleep(1000);
                    drive.setDrivePower(0,0,0,0);

                    // drive over the rough terrain
                    drive.driveStraight(-1,20);
                    drive.motorsHalt();

                    // correct the orientation of the robot
                    drive.driveTurn(params.hubFactor * 90, params.turnError);

                    // drive back towards the bar to reset position
                    drive.setDrivePower(0.25, 0.25, 0.25, 0.25);
                    sleep(1000);
                    drive.setDrivePower(0,0,0,0);

                    //position the arm to place the shipping element in the correct level
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

                    drive.bucketDump();
                    sleep(350);

                    // back away from the hub
//                    drive.driveStraight(0.6, 5);

                    // wait until the bot has backed away from the hub to return the arm to position
                    mechControl.moveToZero();

                    // drive back over the barrier & realign position
                    drive.driveStraight(1, 20);
                    drive.driveTurn(params.hubFactor * 90, params.turnError);

                    // check to see if the robot should just park or if the robot should score the duck
                    if(scoreDuck){
                        runState = State.CAROUSEL_DUCK;
                    } else {
                        runState = State.CAROUSEL_STORAGE_PARK;
                    }

//                    runState = State.HALT;

                    break;


                case CAROUSEL_DUCK:
                    // realign the robot
                    drive.driveTurn(params.hubFactor * 90, params.turnError);

                    // get closer to the outside wall
                    drive.driveStraight(0.5, 20);

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    // drive towards the outside wall using distance sensor
                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30) && opModeIsActive()&&((runtime.time()-currentTime)<1.00))) {
                        if (((runtime.time()-currentTime)>1.5)){
                            break;
                        }
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                        //   telemetry.addData("elapsed time = ", runtime.time()-currentTime);
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                    drive.motorsHalt();
                    telemetry.addData("Reached","Wall");
                    telemetry.update();
                    //go to carousel, red
                    //turn to face carousel
                    drive.driveTurn(0, params.turnError);
                    sleep(350);
                    drive.driveTurn(0, params.turnError);       // double check angle towards carousel

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    //drive forward until distance sensor is tripped
                    // drive towards the carousel
                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30)&&opModeIsActive()&&((runtime.time()-currentTime)<1.25))) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(-robot.CHAIN_POW * 0.75);
                    }   // end while(robot.frontDistance...

                    //drive forward at very low power to keep in contact with carousel
                    drive.setDrivePower(0.1,0.1,0.1,0.1);

                    //sleep to wait for carousel to drop duck to the floor
                    sleep(4000);

                    //reposition to face carousel again
                    drive.driveTurn(0,params.turnError);

                    // drive away from the carousel
                    drive.driveStraight(params.reverseSpeed, 2);

                    //turn off chainsaw
                    robot.motorChainsaw.setPower(0);

                    if(!warehousePark) {
                        runState = State.CAROUSEL_STORAGE_PARK;
                    }else {
                        runState = State.WAREHOUSE_PARK;
                    }

                    // turn towards the
                    break;

                case CAROUSEL_STORAGE_PARK:
                    if(debugMode) {
                        telemetry.addData("Working on X_Score = ", "Now");
                        telemetry.update();
//                        sleep(5000);
                    }

                    if(scoreDuck) {
                        drive.driveStraight(params.reverseSpeed, 1);
                        drive.driveTurn((-15 * params.hubFactor), params.turnError);
                        drive.driveStraight(params.reverseSpeed, 12);

                        drive.driveTurn(0, params.turnError);
                    } else {
                        // drive towards the carousel
                        while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30)&&opModeIsActive()&&((runtime.time()-currentTime)<1.25))) {
                            drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                    params.forwardSpeed, params.forwardSpeed);
                            robot.motorChainsaw.setPower(-robot.CHAIN_POW * 0.75);
                        }
                    }
                    runState = State.HALT;
                    break;

                /*********************************************************************
                 * *******************************************************************
                 * ************ E N D     O F       C A R O U S E L ******************
                 * *******************************************************************
                 ********************************************************************/


                /*********************************************************************
                 * *******************************************************************
                 * ******* B E G I N N I N G    O F     M I D D L E ******************
                 * *******************************************************************
                 ********************************************************************/


                case MIDDLE_SCORING:
                    // drive forward towards the alliance shipping hub, away from the wall
                    drive.driveStraight(-0.4, 5);

                    // rotate towards the Alliance Hub
                    drive.driveTurn(params.midTurnHubAngle, params.turnError);

                    // drive forward towards the alliance shipping hub
                    drive.driveStraight(-0.4, 5);

                    //position the arm to place the shipping element in the correct level
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

                    // wait until the bot has backed away from the hub to return the arm to position
                    mechControl.moveToZero();

                    // Turn towards the wall
                    drive.driveTurn(0, params.turnError);

                    // drive forward towards the outside wall
                    drive.driveStraight(0.6, 5);

                    // Turn towards the carousel
                    drive.driveTurn(params.hubFactor * 90, 4);
                    drive.driveTurn(params.hubFactor * 90, params.turnError);

                    // drive towards the barricade and prepare to go over it
                    drive.setDrivePower(0.25, 0.25, 0.25, 0.25);
                    sleep(750);
                    drive.setDrivePower(0,0,0,0);

                    // halt the drive motors
                    drive.motorsHalt();

                    runState = State.MIDDLE_DUCK;

                    break;

                case MIDDLE_DUCK:

                    // drive over the barrier
                    drive.driveStraight(1, 20);

                    // realign position towards duck carousel
                    // Angle slightly away from the carousel so that to position for instigating carousel
                    drive.driveTurn(params.hubFactor * 96, params.turnError);

                    // drive closer to the duck carousel
                    drive.driveStraight(0.3, 35);

                    // turn the chainsaw on
                    robot.motorChainsaw.setPower(-params.hubFactor* robot.CHAIN_POW*0.75);

                    // turn towards the carousel
                    drive.driveTurn(25, params.turnError);

                    // drive into the carousel to release the duck
                    drive.setDrivePower(0.2, 0.2, 0.2, 0.2);

                    sleep(3500);

                    drive.motorsHalt();

                    if(!warehousePark){
                        // go to MIDDLE_STORAGE_PARK
                        runState = State.MIDDLE_STORAGE_PARK;
                    } else {
                        // go to MIDDLE_WAREHOUSE_PARK
                        runState = State.MIDDLE_WAREHOUSE_PARK;
                    }

                    break;

                case MIDDLE_WAREHOUSE_PARK:
                    // drive away from the carousel & return to the middle starting position to wait to park
                    drive.driveStraight(0.5, 5);

                    // turn towards the barrier
                    drive.driveTurn(params.hubFactor * 90, 2);

                    // drive to the barrier
                    drive.driveStraight(0.5, 30);
                    drive.driveTurn(params.hubFactor * 90, 2);

                    // roll into the barrier to finalize position to wait for end
                    drive.setDrivePower(0.25, 0.25, 0.25, 0.25);
                    sleep(1500);
                    drive.setDrivePower(0,0,0,0);
                    drive.driveTurn(params.hubFactor * 90, 2);

                    // drive over the barrier
                    drive.driveStraight(1, 20);
                    drive.driveTurn(params.hubFactor * 90, 2);

                    // wait until there is only a few seconds left to go park
                    while(((runtime.time()) < params.warehouseParkDelay) && opModeIsActive()){
                        telemetry.addData("Time Left = ", (runtime.time()));
                        telemetry.addData("Hubfactor = ", params.hubFactor);
                        telemetry.update();
                        // do nothing
                    }   // end of while(((runtime.time()) < params.warehouseParkDelay) && opModeIsActive())

                    // drive to the warehouse
                    drive.driveStraight(1, 60);

                    // shut off the drive motors
                    drive.motorsHalt();

                    runState = State.HALT;

                    break;

                case MIDDLE_STORAGE_PARK:
                    // back away from the carousel
                    drive.driveStraight(0.4, 5);

                    // rotate towards the outside wall
                    drive.driveTurn(params.hubFactor*90, params.turnError);

                    // drive closer to the outside wall
                    drive.driveStraight(0.3, 5);

                    // rotate away from the carousel
                    drive.driveTurn(params.hubFactor * 170, params.turnError);

                    // wait until there is only a couple of seconds before driving into the storage
                    while(((runtime.time()) < params.warehouseParkDelay) && opModeIsActive()){
                        telemetry.addData("Time Left = ", (runtime.time()));
                        telemetry.addData("Hubfactor = ", params.hubFactor);
                        telemetry.update();
                        // do nothing
                    }   // end of while(((runtime.time()) < params.warehouseParkDelay) && opModeIsActive())

                    // drive into the storage
                    drive.driveStraight(0.4, 25);

                    break;


                /*********************************************************************
                 * *******************************************************************
                 * ************* E N D    O F     M I D D L E ************************
                 * *******************************************************************
                 ********************************************************************/

                /*********************************************************************
                 * *******************************************************************
                 * ******* B E G I N N I N G    O F     W A R E H O U S E ************
                 * *******************************************************************
                 ********************************************************************/



                case WAREHOUSE_SCORE:
                    turnError = 2;

                    if(debugMode) {
                        telemetry.addData("Working on X_Score = ", "Now");
                        telemetry.update();
                    }   // end if(debugMode)

                    //turn towards the hub
                    drive.driveTurn(params.turnAngle, params.turnError);

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
                    drive.driveStraight(params.reverseSpeed, hubDistance);
                    sleep(350);

                    //dump bucket
                    robot.bucketDump.setPosition(bucketAngle);
                    sleep(500);     // allow time to dump the cube

                    // drive away from the alliance hub
                    drive.driveStraight(params.forwardSpeed, (hubDistance+params.extraDistance));

                    //reset arms
                    robot.bucketDump.setPosition(0.5);
                    armControl.moveToZero();

                    // reset the sweeper bar
                    drive.resetTSEBar();

                    // go park
                    runState = State.WAREHOUSE_PARK;
                    break;

                case RED_CAROUSEL:
                    if(debugMode) {
                        telemetry.addData("Working on RED_Carousel = ", "Now");
                        telemetry.update();
                    }

                    // turn towards outside wall
                    drive.driveTurn((90 * params.hubFactor), turnError);
                    sleep(350);

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    // drive towards the outside wall using distance sensor
                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 32) && opModeIsActive()&&((runtime.time()-currentTime)<1.00))) {
                        if (((runtime.time()-currentTime)>1.5)){
                          break;
                        }
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                    //   telemetry.addData("elapsed time = ", runtime.time()-currentTime);
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                    drive.motorsHalt();
                    telemetry.addData("Reached","Wall");
                    telemetry.update();
                    //go to carousel, red
                    //turn to face carousel
                    drive.driveTurn(0, params.turnError);
                    sleep(350);
                    drive.driveTurn(0, params.turnError);       // double check angle towards carousel

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    //drive forward until distance sensor is tripped
                    // drive towards the carousel
                    while(((robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM) > 30)&&opModeIsActive()&&((runtime.time()-currentTime)<1.25))) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                    params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(-robot.CHAIN_POW * 0.75);
                    }   // end while(robot.frontDistance...

                    //drive forward at very low power to keep in contact with carousel
                    drive.setDrivePower(0.1,0.1,0.1,0.1);

                    //sleep to wait for carousel to drop duck to the floor
                    sleep(4000);

                    //reposition to face carousel again
                    drive.driveTurn(0,params.turnError);

                    // drive away from the carousel
                    drive.driveStraight(params.reverseSpeed, 2);
                    /*
                    while(robot.frontDistanceSensor.getDistance(DistanceUnit.CM) < 30) {
                        drive.setDrivePower(params.reverseSpeed, params.reverseSpeed,
                                params.reverseSpeed, params.reverseSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                     */

//                    drive.driveStraight(params.reverseSpeed, 4);

                    //turn off chainsaw
                    robot.motorChainsaw.setPower(0);

                    if(!warehousePark) {
                        runState = State.CAROUSEL_STORAGE_PARK;
                    }else {
                        runState = State.WAREHOUSE_PARK;
                    }
                    break;

                case BLUE_CAROUSEL:
                    // turn towards outside wall
                    drive.driveTurn((90 * params.hubFactor), turnError);
                    sleep(350);

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    // drive towards the outside wall using distance sensor
                    while(((robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM) > 32)&&opModeIsActive()&&((runtime.time()-currentTime)<1.0))) {
                        if (((runtime.time()-currentTime)>1.5)){
                            break;
                        }
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor

                    drive.motorsHalt();

                    if(debugMode) {
                        telemetry.addData("Working on BLUE Carousel = ", "Now");
                        telemetry.update();
                    }

                    //go to carousel, blue (movements same as red, in reverse

                    // turn towards the carousel
                    drive.driveTurn(0, params.turnError);
                    sleep(350);
                    drive.driveTurn(0, params.turnError);

                    // set elapsedtime to current time
                    currentTime = runtime.time();
                    while(((robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM)>30)&&opModeIsActive()&&((runtime.time()-currentTime)<1.25))) {
                        drive.setDrivePower(params.forwardSpeed, params.forwardSpeed,
                                    params.forwardSpeed, params.forwardSpeed);
                        robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);
                    }   // end of while(robot.frontDistanceSensor

                    // drive slowly into the carousel to apply constant pressure
                    drive.setDrivePower(0.1,0.1,0.1,0.1);
                    sleep(4000);

                    drive.driveTurn(0,params.turnError);

                    drive.driveStraight(params.reverseSpeed, 2);
                    // turn towards the storage

                    robot.motorChainsaw.setPower(0);

                    // drive away from the carousel to make room to rotate
                 /*   while(robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM) < 30) {
//                        drive.setDrivePower(params.reverseSpeed, params.reverseSpeed,
//                                params.reverseSpeed, params.reverseSpeed);

                        telemetry.addData("Headed towards ","outside wall");
                        telemetry.addData("distance to wall = ", robot.frontDistanceSensorBlue.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }   // end of while(robot.frontDistanceSensor
*/
                    // drive away from the carousel
                 //   drive.driveStraight(params.reverseSpeed, 1);

                    if(!warehousePark) {
                        runState = State.CAROUSEL_STORAGE_PARK;
                    }else {
                        runState = State.WAREHOUSE_PARK;
                    }

                    break;

                case WAREHOUSE_PARK:
                    if(debugMode) {
                        telemetry.addData("Working on Warehouse Park = ", "Now");
                        telemetry.update();
//                        sleep(5000);
                    }
//                    drive.driveStraight(params.reverseSpeed, 4);
                    sleep(250);

                    // turn towards the warehouse
                    drive.driveTurn((90 * -params.hubFactor), params.turnError);
                    drive.driveTurn((90 * -params.hubFactor), params.turnError);

                    // turn on the chainsaw so that we know the program is still running
                    robot.motorChainsaw.setPower(0.2);

                    // wait until there is just a few seconds to go park
                    while(((runtime.time()) < params.warehouseParkDelay) && opModeIsActive()){
                        telemetry.addData("Time Left = ", (runtime.time()));
                        telemetry.addData("Hubfactor = ", params.hubFactor);
                        telemetry.update();
                        // do nothing
                    }   // end of while(startTime - runtime.time() > 3)

                    drive.driveStraight(1, params.warehouseParkDistance);
                    robot.motorChainsaw.setPower(0);
                    runState = State.HALT;
                    break;

                case HALT:
                    if(debugMode) {
                        telemetry.addData("Working on Halt = ", "Now");
                        telemetry.update();
                    }   // if(debugMode)

                    armControl.moveToZero();          // reset the arm to the right initialized position
                //    turretControl.resetTurret();    // reset the arm to the right initialized position

                    // shut down all motors
                    robot.motorChainsaw.setPower(0);
                    robot.motorIntake.setPower(0);
                    drive.motorsHalt();

                    running = false;        // exit the program loop
                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        if(debugMode){
            dlStop();               // stop the data logger
        }
        turretControl.stop();
        mechControl.stop();
        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, ALLIANCE_SELECT, DELAY_LENGTH, FIELD_POSITION_SELECT, SELECT_BONUS, SELECT_CAR_OPTIONS, VERIFY_CONFIG, TEST_CONFIG,
        SLEEP_DELAY, LEVEL_ADJUST, MOVE_TSE_STRAIGHT, MOVE_TSE_ARC, WAREHOUSE_SCORE, RED_CAROUSEL, BLUE_CAROUSEL, BONUS_SCORES,
        BLUE_WAREHOUSE_BONUS, RED_WAREHOUSE_BONUS,  WAREHOUSE_PARK, HALT, SELECT_PARK,
        INIT_PARAMS,
        MIDDLE_SCORING, MIDDLE_DUCK, MIDDLE_STORAGE_PARK, MIDDLE_WAREHOUSE_PARK,
        CAROUSEL_SCORING, CAROUSEL_DUCK, CAROUSEL_STORAGE_PARK
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
