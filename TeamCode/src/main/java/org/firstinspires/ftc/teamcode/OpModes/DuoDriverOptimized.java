package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;

import dalvik.system.DelegateLastClassLoader;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "2 Driver - States", group = "Development")
@Disabled

public class DuoDriverOptimized extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State controlState = State.CLEAR;
    private ElapsedTime runtime = new ElapsedTime();
    private double timer = 0;

    public DuoDriverOptimized(){

    }   // end of DuoDriverOptimized constructor

    public void runOpMode(){
        //mechanism control thread
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        //turret control thread
        TurretControlLibrary turretControl = new TurretControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);


        double bucketAngle=0.5;
        int bumpCount=0;
        int chainsawMode=0;
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

        int intakeSpin = 1;

        int turretPreset=0;
        double turn, drive, left, right, max;
        int turretPosition=0;
        int turretThreshold=2;

        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();
        runtime.reset();        // reset the runtime clock

        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        mechController.start();
        turretController.start();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            GP1/
            Left Stick - forward/backward
            Right Stick - turn
            A - intake / retract intake
            L/R bumpers - chainsaw
            L/R triggers - fast chainsaw
            */


//drive control section (GP1, Joysticks)
            drive = -gamepad1.left_stick_y * robot.DRIVE_MULTIPLIER;
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

            if(gamepad1.a){
                controlState = State.TOGGLE_INTAKE;
            }
            if(gamepad1.b){
                controlState = State.CLEAR;
            }
            if(gamepad1.x){
                controlState = State.CLEAR;
            }
            if(gamepad1.y) {
                controlState = State.CLEAR;
            }

            if (gamepad1.dpad_up){
                controlState = State.CLEAR;
            }

            if(gamepad1.dpad_down){
                controlState = State.CLEAR;
            }

            if(gamepad1.dpad_right){
                controlState = State.CLEAR;
            }
            if(gamepad1.dpad_left){
            controlState = State.CLEAR;
            }
            if(gamepad1.right_bumper){
                controlState = State.CLEAR;
            }
            if(gamepad1.left_bumper){
                controlState = State.CLEAR;
            }

            switch (controlState){

                /**
                 * Control positioning of the Turret
                 */
                case LEFT:
                    break;

                case RIGHT:
                    break;

                case CENTER:
                    break;

                case TOGGLE_INTAKE:
                    if(timeSpan() > 0.1){
                        if(intakeSpin == 1){
                            toggleIntake=false;
                            intakeDown=!intakeDown;
                            isDeployed=false;
                            bumpCount=0;
                        }
                        intakeSpin = intakeSpin * -1;
                        robot.motorIntake.setPower(intakeSpin);
                        controlState = State.CLEAR;
                        resetTimer();
                    }   // end if(timeSpan)
                    break;

                case DEPLOY_INTAKE:
                    robot.motorIntake.setPower(1);
                        intakeDown=!intakeDown;
                        isDeployed=false;
                        bumpCount=0;
                    break;
                case CLEAR:
                    break;

            }

//end of bucket controls

            telemetry.addData("TSE MODE: ",TSEMode);
            telemetry.addData("","");
            telemetry.addData("Last Intake Servo Pos",robot.bucketDump.getPosition());
            telemetry.addData("Turret Current Angle: ",robot.turrentEncoder.getCurrentPosition());
            telemetry.addData("Turret Target Angle: ",turretPosition);
            telemetry.addData("Turret Preset: ",turretPreset);
            telemetry.addData("Left Power: ",left);
            telemetry.addData("Right Power: ",right);
            telemetry.addData("Chainsaw Power: ",chainsawPower);
            telemetry.addData("Happy Driving ",")");
            telemetry.update();
        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();                 // shut down the arm control thread
        turretControl.stop();               // shut down the turret control thread
        requestOpModeStop();                // shut down the opmode
    }   // end of runOpMode method

    enum State {
        LEFT, RIGHT, CENTER, FREE_CONTROL, DEPLOY_INTAKE, TOGGLE_INTAKE, CLEAR
    }

    void resetTimer(){
        timer =  runtime.time();
    }

    double timeSpan(){
        return (runtime.time() - timer);
    }
}   // end of TeleOpDuoDriver.java class