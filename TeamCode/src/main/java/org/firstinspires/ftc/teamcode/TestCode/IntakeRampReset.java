package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Intake Ramp Reset", group = "Concept")
//  @Disabled

public class IntakeRampReset extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public IntakeRampReset(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
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

        waitForStart();

        while(opModeIsActive()) {
            robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO - robot.INTAKE_DEPLOY_BLUE);
            robot.intakeDeployPink.setPosition(robot.PINK_ZERO + robot.INTAKE_DEPLOY_PINK);
            robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
            robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_UP);
            robot.sweeperPink.setPosition(robot.PINK_SWEEPER_UP);
        }   // end of while opModeIsActive()
    }   // end of runOpMode method

}   // end of TeleOp.java class


