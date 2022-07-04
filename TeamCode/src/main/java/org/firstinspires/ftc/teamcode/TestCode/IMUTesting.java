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

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;


@Autonomous(name="IMU Testing", group="Competition")
@Disabled
public class IMUTesting extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime   runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        // arm control thread
      //  MechControlLibrary armControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
     //   Thread armController = new Thread(mechControl);

        boolean autoReady = false;
        boolean running = true;
        double startTime;
        String alliance = "";
        String startPosition = "";
        String goalPosition = "";
        long startDelay = 0;
        double timeElapsed;
        double positionFactor = 1;
        int scorePosition=1;
        double forwardSpeed = -0.32;

        double forwardDistance = 24.0;
        double hubDistance = 10.0;
        double hubDistanceBlue = 14.0;
        double goalAdjust = 0;
        double carouselDistance= 19.5;

        double turnAngle = 65;
        double parkDistance = 12.0;
        double parkAdjust = 24.0;

        double turnError = 0.5;
        double bucketAngle = -1.0;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(.5);

        DriveClass drive = new DriveClass(robot, opMode);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        startTime = runtime.time() + startDelay;

        // sleep for set delay time
        sleep(startDelay);

        while (opModeIsActive() && (running)) {
            drive.driveTurn(90, 0.5);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    } // end of opmode

    /*
     * Enumerate the states of the machine
     */

}
