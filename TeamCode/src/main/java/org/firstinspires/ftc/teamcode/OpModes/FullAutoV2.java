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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

import java.util.List;


@Autonomous(name="FullAutoV2", group="Competition")
@Disabled
public class FullAutoV2 extends LinearOpMode {

    public static final String TFOD_MODEL_ASSET = "PP_TSEv2.tflite";
    public static final String[] LABELS = {
            "TSE"
    };

    private static final String VUFORIA_KEY =
            "ARLYRsf/////AAABmWpsWSsfQU1zkK0B5+iOOr0tULkAWVuhNuM3EbMfgb1+zbcOEG8fRRe3G+iLqL1/iAlTYqqoLetWeulG8hkCOOtkMyHwjS/Ir8/2vUVgC36M/wb9a7Ni2zuSrlEanb9jPVsNqq+71/uzTpS3TNvJI8WeICQNPAq3qMwmfqnCphVlC6h2ZSLsAR3wcdzknFmtpApdOp1jHJvITPeD/CMdAXjZDN0XJwJNQJ6qtaYSLGC23vJdQ2b1aeqnJauOvswapsG7BlmR7m891VN92rNEcOX7WmMT4L0JOM0yKKhPfF/aSROwIdNtSOpQW4qEKVjw3aMU1QDZ0jj5SnRV8RPO0hGiHtXy6QJcZsSj/Y6q5nyf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime   runtime = new ElapsedTime();
    private State setupState = State.ALLIANCE_SELECT;

    @Override
    public void runOpMode() {

        boolean autoReady = false;
        boolean running = true;
        double startTime;
        String oldalliance = "";
        String startPosition = "";
        String goalPosition = "";
        long startDelay = 0;
        double timeElapsed;
        double positionFactor = 1;
        int scorePosition = 1;
        double forwardSpeed = -0.32;

        double forwardDistance = 36.0;
        double hubDistance = 8;
        double TSEreturnDist = 10;


        double turnAngle = 65;
        double parkDistance = 35;

        double turnError = 0.5;
        double bucketAngle = -1.0;

        //red if false, blue if true
        boolean alliance = false;

        //carousel if false, warehouse if true
        boolean position = false;

        // warehouse park
        boolean warehousePark = true;

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
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.turrentEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(.5);

        DriveClass drive = new DriveClass(robot, opMode);
        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time
        while (!autoReady) {

            switch (setupState) {
                case ALLIANCE_SELECT:
                    telemetry.addData("Drive Motor Encoders:", robot.motorR1.getCurrentPosition());
                    telemetry.addData("Which Alliance are you on?", "");
                    telemetry.addData("Press X  == ", " BLUE Alliance");
                    telemetry.addData("Press Y  == ", " RED Alliance");
                    telemetry.update();

                    if (gamepad1.x || gamepad1.y) {
                        if (gamepad1.x) {
                            alliance = true;
                        } else {
                            alliance = false;
                        }
                        setupState = State.DELAY_LENGTH;    // give option to add delay
                        telemetry.addData("Goal on ==  ", positionFactor);
                        telemetry.update();
                    }   // end of if(gamepad1.x...
                    break;

                case DELAY_LENGTH:
                    telemetry.addData("Add a Delay", "");
                    telemetry.addData("Press DPAD_UP  ", " To Increase Delay");
                    telemetry.addData("Press DPAD_DOWN ", " To Reduce Delay");
                    telemetry.addData("Current Delay = ", startDelay);

                    telemetry.addData("Press A to continue", "");
                    telemetry.update();

                    if (gamepad1.dpad_up && (runtime.time() - timeElapsed) > 0.3) {
                        if (startDelay < 10) {       // limit the max delay to 10 seconds
                            startDelay = startDelay + 1;
                        }
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if (gamepad1.dpad_down && (runtime.time() - timeElapsed) > 0.3) {
                        if (startDelay > 0) {       // confirm that the delay is not negative
                            startDelay = startDelay - 1;
                        } else startDelay = 0;
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if (gamepad1.a) {         // exit the setup
                        setupState = State.POSITION_SELECT;
                        startDelay = startDelay * 1000;
                    }   // end of if(gamepad1.x...
                    break;

                case POSITION_SELECT:
                    telemetry.addData("What is the position of the robot?", "");
                    telemetry.addData("Press DPAD Left  == ", " CAROUSEL Side");
                    telemetry.addData("Press DPAD Right == ", " WAREHOUSE Side");
                    telemetry.update();

                    if ((gamepad1.dpad_left || gamepad1.dpad_right) && ((runtime.time() - timeElapsed) > 0.3)) {
                        if (gamepad1.dpad_left) {
                            position = false;
                            //sleep(1000);
                        } else {
                            position = true;
                        }
                        setupState = State.SELECT_PARK;
                    }   // end of if(gamepad1.dpad_left...
                    sleep(350);
                    break;

                case SELECT_PARK:
                    telemetry.addData("Warehouse park?", "");
                    telemetry.addData("Press DPAD Up  == ", " Park in Warehouse");
                    telemetry.addData("Press DPAD Down == ", " Park in Storage");
                    telemetry.update();

                    if (gamepad1.dpad_up || gamepad1.dpad_down) {
                        if (gamepad1.dpad_up) {
                            warehousePark = true;
                            //sleep(1000);
                        } else {
                            warehousePark = false;
                        }
                        setupState = State.VERIFY_CONFIG;
                        sleep(350);

                    }  // end of if(gamepad1.dpad_left..
                    break;
                case VERIFY_CONFIG:
                    if (positionFactor == 1) {
                        goalPosition = "Right";
                    } else {
                        goalPosition = "Left";
                    }
                    telemetry.addData("Verify the setup", "");
                    telemetry.addData("Alliance          == ", alliance);
                    telemetry.addData("Start Delay == ", startDelay);
                    telemetry.addData("Starting Position ==  ", startPosition);
                    telemetry.addData("Goal position ==  ", goalPosition);
                    telemetry.addData("Goal on ==  ", positionFactor);
                    telemetry.addData("Warehouse Park ==  ", warehousePark);
                    telemetry.addData("", "");
                    telemetry.addData("Press A to Confirm or B to start over", "");
                    telemetry.update();

                    if (gamepad1.b || gamepad1.a) {
                        if (gamepad1.b) {
                            autoReady = false;
                            startDelay = 0;     // reset the start delay
                            setupState = State.ALLIANCE_SELECT;
                        } else {
                            autoReady = true;
                        }
                    }   // end of if(gamepad1.right_bumper...
                    break;
            }   // end of switch(setupState)
        }   // end of while(autoReady)

        // warehouse side
        if (position) {
            TSEreturnDist = 8;
        }
        //red carousel
        if (!alliance && !position) {
            positionFactor = 1;

            //blue carousel
        } else if (alliance && !position) {
            hubDistance += 5;
            positionFactor = -1;
            //red warehouse
        } else if (!alliance && position) {
            forwardDistance = 28;
            hubDistance -= 3;
            positionFactor = -1;
            parkDistance = 50;
            //blue warehouse
        } else {
            forwardDistance = 28;
            hubDistance -= 3;
            positionFactor = 1;
            parkDistance = 50;
        }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while(!opModeIsActive()){

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
                        if (scorePosition == 1) {
                            telemetry.addData("TSE targeting ", " Top Level");
                        } else if (scorePosition == 2) {
                            telemetry.addData("TSE targeting ", " Middle Level");
                        } else if (scorePosition == 3) {
                            telemetry.addData("TSE targeting ", " Bottom Level");
                        }
                        telemetry.update();

                        if (recognition.getLeft() < 150) {
                            scorePosition = 3;
                        } else if (recognition.getLeft() > 150 && recognition.getLeft() < 400 ) {
                            scorePosition = 2;
                        } else if (recognition.getLeft() > 400) {
                            scorePosition = 1;
                        }
                        i++;

                    }     // if (Recognition...
                    // Send telemetry message to signify robot waiting;
                    /*    telemetry.addData("Robot Status : ", "READY TO RUN");    //
                        telemetry.addData("Scanning for : ", "TSE");
                     */
                    telemetry.addData("Detected Level = ", scorePosition);
                    //   telemetry.addData("Press X to : ", "ABORT Program");
                    //telemetry.update();

                }   // if (updatedRecog...)
            }   // end of if (tfod != null)
        }


        waitForStart();
        startTime = runtime.time() + startDelay;

        // start arm controller thread
        //    armController.start();

        // sleep for set delay time. turn chainsaw on to show robot isn't dead
        robot.motorChainsaw.setPower(0.2);
        sleep(startDelay);
        robot.motorChainsaw.setPower(0);


        while (opModeIsActive() && (running)) {

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
                        telemetry.update();
                        if(!alliance) {
                            if (recognition.getTop() < 220 && recognition.getTop() > 115) {
                                if (recognition.getLeft() < 150) {
                                    scorePosition = 3;
                                }
                                if (recognition.getLeft() > 150) {
                                    scorePosition = 2;
                                }
                            }
                            if (updatedRecognitions.size() == 0) scorePosition = 1;
                        }else{
                            if (recognition.getTop() < 220 && recognition.getTop() > 115) {
                                if (recognition.getLeft() < 175) {
                                    scorePosition = 2;
                                }
                                if (recognition.getLeft() > 400) {
                                    scorePosition = 1;
                                }
                            }
                            if (updatedRecognitions.size() == 0) scorePosition = 3;
                        }
                        i++;

                    }     // if (Recognition...
                    // Send telemetry message to signify robot waiting;
                /*    telemetry.addData("Robot Status : ", "READY TO RUN");    //
                    telemetry.addData("Scanning for : ", "TSE");
                 */
                    telemetry.addData("Detected Level = ", scorePosition);
                 //   telemetry.addData("Press X to : ", "ABORT Program");
                    //telemetry.update();

                }   // if (updatedRecog...)
            }   // end of if (tfod != null)


            //warehouse
            if(position){
                if(scorePosition==1){

                }else if(scorePosition==2){
                    forwardDistance = 10;
                    hubDistance = 14;
                    TSEreturnDist = 0;
                    bucketAngle=-0.55;
                }else{
                    forwardDistance = 10;
                    hubDistance = 12;
                    TSEreturnDist = 0;
                    bucketAngle=-0.75;
                }
                turnAngle=35;
            }else{
                if(scorePosition==1){
                    forwardDistance=32;
                    hubDistance=10;
                    parkDistance-=3;
                    turnAngle=60;
                }else if(scorePosition==2){
                    hubDistance = 7;
                    parkDistance-=1;
                    bucketAngle=-0.55;
                }else{
                    hubDistance = 7.25;
                    parkDistance-=1;
                    bucketAngle=-0.75;
                }
            }


            // Step 1
            //drive forward and push TSE out of the way
            drive.driveStraight(forwardSpeed, forwardDistance);
            sleep(500);

            //back up to turn to the shipping hub
            drive.driveStraight(-forwardSpeed,TSEreturnDist);
            sleep(250);

            //turn towards the hub
            drive.driveTurn(turnAngle * positionFactor, turnError);

            //move arm to scoring positions
            if(scorePosition==1){
                armControl.scoringPos1();
            }else if(scorePosition==2){
                armControl.scoringPos2();
            }else if(scorePosition==3){
                armControl.scoringPos3();
            }
            sleep(500);

            //drive towards the shipping hub to score
            drive.driveStraight(forwardSpeed, hubDistance);
            sleep(500);

            //dump bucket
            robot.bucketDump.setPosition(bucketAngle);
            sleep(500);

            //reset arms
            robot.bucketDump.setPosition(0.5);
            armControl.moveToZero();

            // reverse direction to drive forward to park
            forwardSpeed = forwardSpeed * -1;

            //set forward speed to full power if in either warehouse position
            if(!alliance&&position){
                //red
                forwardSpeed=1;
            }else if(alliance&&position){
                //blue
                forwardSpeed=1;
            }
            if(position){
                if(alliance) {
                    drive.driveTurn(90, turnError);
                }else{
                    drive.driveTurn(-90,turnError);
                }
            //    drive.driveStraight(forwardSpeed,15);
                drive.driveStraight(forwardSpeed,17);
                parkDistance=36;
            }

            //drive to park if on warehouse side, drive to wall if on carousel side
            drive.driveStraight(forwardSpeed, parkDistance);
            sleep(500);

            //go to carousel, red
            if(!alliance&&!position){
                //turn to face carousel
                drive.driveTurn(0,turnError);
                sleep(350);

                //drive forward until distance sensor is tripped
                while(robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM)>29) {
                    drive.setDrivePower(forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
                    robot.motorChainsaw.setPower(-robot.CHAIN_POW*0.75);
                }

                //drive forward at very low power to keep in contact with carousel
                drive.setDrivePower(0.1,0.1,0.1,0.1);

                //sleep to wait for duck to move
                sleep(2500);

                //reposition to face carousel again
                drive.driveTurn(0,turnError);

                //turn off chainsaw
                robot.motorChainsaw.setPower(0);
                if(!warehousePark) {
                    drive.driveStraight(-forwardSpeed, 4);
                    drive.driveTurn(-15,turnError);
                    drive.driveStraight(-forwardSpeed,12);
                }else {
                    //back up away from carousel
                    drive.driveStraight(-forwardSpeed, 8);
                    sleep(250);

                    //turn to face warehouse
                    drive.driveTurn(-90, turnError);

                    //wait for alliance partner to move. turn chainsaw on to show that robot isn't dead
                    robot.motorChainsaw.setPower(0.2);
                    sleep(7000 - startDelay);

                    //park in warehouse
                    drive.driveStraight(1, 100);
                }
            }

            //go to carousel, blue (movements same as red, in reverse
            if(alliance&&!position){

                drive.driveTurn(0,turnError);
                sleep(350);

                while(robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM)>30) {
                    drive.setDrivePower(forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
                    robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);
                }

                drive.setDrivePower(0.1,0.1,0.1,0.1);
                sleep(2500);

                drive.driveTurn(0,turnError);
                robot.motorChainsaw.setPower(0);
                if(!warehousePark) {
                    drive.driveStraight(-forwardSpeed, 4);
                    drive.driveTurn(15,turnError);
                    drive.driveStraight(-forwardSpeed,12);
                }else {

                    drive.driveStraight(-forwardSpeed, 8);
                    sleep(250);

                    drive.driveTurn(90, turnError);

                    robot.motorChainsaw.setPower(0.2);
                    sleep(5000 - startDelay);

                    drive.driveStraight(1, 100);
                }
            }

            //stop motors
            drive.motorsHalt();
            running = false;
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        ALLIANCE_SELECT, POSITION_SELECT, VERIFY_CONFIG, DELAY_LENGTH,SCORING_POSITION,
        FIELD_PARK, HALT, SELECT_PARK
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
