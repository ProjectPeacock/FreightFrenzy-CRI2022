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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;


@Autonomous(name="AutoPark", group="compition")
public class AutoPark extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();



    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {
        long startDelay = 0;
       // boolean autoReady = false;
        ElapsedTime runTime = new ElapsedTime();
        double timeElapsed = runtime.time();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);

      /*  while (!autoReady) {
                telemetry.addData("Add a Delay", "");
                telemetry.addData("Press DPAD_UP  ", " To Increase Delay");
                telemetry.addData("Press DPAD_DOWN ", " To Reduce Delay");
                telemetry.addData("Current Delay = ", startDelay);

                telemetry.addData("Press A to continue", "");
                telemetry.update();

                if (gamepad1.dpad_up && (runTime.time() - timeElapsed) > 0.3) {
                    if (startDelay < 25) {       // limit the max delay to 25 seconds
                        startDelay = startDelay + 1;
                    }
                    timeElapsed = runTime.time();
                } //end of if(gamepad1.dpad_up)

                if (gamepad1.dpad_down && (runTime.time() - timeElapsed) > 0.3) {
                    if (startDelay > 0) {       // confirm that the delay is not negative
                        startDelay = startDelay - 1;
                    } else startDelay = 0;
                    timeElapsed = runTime.time();
                } //end of if(gamepad1.dpad_up)

                if (gamepad1.a) {         // exit the setup
                    autoReady = true;
                    startDelay = startDelay * 1000;
                }   // end of if(gamepad1.x...


        } */
                waitForStart();
      //  sleep(startDelay);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1
        robot.motorR1.setPower(FORWARD_SPEED);
        robot.motorL1.setPower(FORWARD_SPEED);
        robot.motorR2.setPower(FORWARD_SPEED);
        robot.motorL2.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Step 2
        robot.motorR1.setPower(0);
        robot.motorL1.setPower(0);
        robot.motorR2.setPower(0.0);
        robot.motorL2.setPower(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
