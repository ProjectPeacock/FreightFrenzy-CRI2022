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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;


@Autonomous(name="AutoParkDriveClass", group="Competition")
@Disabled
public class AutoParkDriveClass extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = -0.4;
    static final double     DRIVE_HEADING    = 0.0;
    static final double     DRIVE_DISTANCE    = 36.0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);

        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1
//        drive.driveStraight(FORWARD_SPEED,DRIVE_DISTANCE);
           // telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
           // telemetry.update();

      //  drive.driveStraight(0.4, 72);
      //  drive.driveTurn(90, 0.5);
        drive.driveStraight(-0.4, 26);
     //   drive.driveTurn(-90, 0.5);
        drive.driveTurn(-90, 0.5);
        drive.driveStraight(0.4, 24);

        telemetry.addData("motor L1 = ", robot.motorL1.getCurrentPosition());
        telemetry.addData("motor L2 = ", robot.motorL2.getCurrentPosition());
        telemetry.addData("motor R1 = ", robot.motorR1.getCurrentPosition());
        telemetry.addData("motor R2 = ", robot.motorR2.getCurrentPosition());
        telemetry.update();


        // Step 2

        drive.motorsHalt();

     //   telemetry.addData("Path", "Complete");
     //   telemetry.update();
        sleep(1000);
    }
}
