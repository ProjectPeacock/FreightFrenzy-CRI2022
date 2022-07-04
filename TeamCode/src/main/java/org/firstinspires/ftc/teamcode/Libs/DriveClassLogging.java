// change comments
/*
12/4/21 corrected motor / variable associations in calcDistance
*/
package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;


public class DriveClassLogging {
    private HardwareProfile robot;
    private LinearOpMode opMode;
    private double r1Power;
    private double r2Power;
    private double l1Power;
    private double l2Power;
    public double startZAngle=0;
    private DataLogger imuDl;               // imu datalogger

    //constructor
    public DriveClassLogging(HardwareProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
        //getStartAngle();
    }

    public void startLogging() {
        imuDl        = new DataLogger("IMU_log");
        imuDl.addField("Loop");
        imuDl.addField("targetAngle");
        imuDl.addField("ZAngle");
        imuDl.addField("lastError");
        imuDl.addField("error");
        imuDl.addField("derivative");
        imuDl.addField("rotationSpeed");
        imuDl.addField("iterations");
    }

    public void stopLogging() {
        imuDl.closeDataLogger();
    }

    public void driveStraight(double power, double distance) {
        boolean active = true;

        double l1Start = 0;
        double l2Start = 0;
        double r1Start = 0;
        double r2Start = 0;
        double coastMinimum = 5;
        double coastDistance = .8;
        double coastPower = .75;

        l1Start = robot.motorL1.getCurrentPosition();
        l2Start = robot.motorL2.getCurrentPosition();
        r1Start = robot.motorR1.getCurrentPosition();
        r2Start = robot.motorR2.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            r1Power = Range.clip(power, -1,1);
            l1Power = Range.clip(power, -1,1);
            r2Power = Range.clip(power, -1,1);
            l2Power = Range.clip(power, -1,1);

            /*
             * Apply power to the drive wheels
             */

            if(distance*coastDistance < coastMinimum) {
                if(distance-coastMinimum<=calcDistance(r1Start, r2Start, l1Start, l2Start)){
                    r1Power *=0.65;
                    r2Power *=0.65;
                    l1Power *=0.65;
                    l2Power *=0.65;
                }
            }else {
                if (distance * coastDistance <= calcDistance(r1Start, r2Start, l1Start, l2Start)) {
                    r1Power *= coastPower;
                    r2Power *= coastPower;
                    l1Power *= coastPower;
                    l2Power *= coastPower;
                }

            }
            setDrivePower(r1Power, r2Power, l1Power, r2Power);
            opMode.telemetry.addData("LF Start = ", l1Start);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(r1Start, r2Start, l1Start, l2Start));
            opMode.telemetry.update();


            if(calcDistance(r1Start, r2Start, l1Start, l2Start) >= distance) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

    } // close driveStraight method

    /**
     * Method: calcDistance
     * @param r1Start   - Right Front starting encoder value
     * @param r2Start   - Right Rear starting encoder value
     * @param l1Start   - Left Front starting encoder value
     * @param l2Start   - Left Rear starting encoder value
     */
    public double calcDistance(double r1Start, double r2Start, double l1Start, double l2Start){

        double distanceTraveled = 0;
        double r1Encoder = robot.motorR1.getCurrentPosition();;
        double r2Encoder = robot.motorR2.getCurrentPosition();;
        double l1Encoder = robot.motorL1.getCurrentPosition();;
        double l2Encoder = robot.motorL2.getCurrentPosition();

            distanceTraveled = ((Math.abs(r1Start - r1Encoder) + Math.abs(r2Start - r2Encoder)
                    + Math.abs(l1Start-l1Encoder) + Math.abs(l2Start - l2Encoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);

        return Math.abs(distanceTraveled);
    }  //close calcDistance


    // distance will power side with highest power
    public void driveArcTurn(double powerLeft, double powerRight, double time) {
        ElapsedTime runTime = new ElapsedTime();
        double currentTime = runTime.time();

        while (((runTime.time() - currentTime) < time) && opMode.opModeIsActive()){

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            r1Power = Range.clip(powerRight, -1,1);
            l1Power = Range.clip(powerLeft, -1,1);
            r2Power = Range.clip(powerRight, -1,1);
            l2Power = Range.clip(powerLeft, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(r1Power, r2Power, l1Power, r2Power);

            opMode.idle();

        }   // end of while loop

        motorsHalt();
    }

    public void driveTurn(double targetAngle, double errorFactor) {
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.02;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.8;
        double rotationSpeed = 0.0;
        double derivative = 0, deltaError, lastError = 0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
//        if (targetAngle > 90 || targetAngle < -90) {
//            error = gyro360(targetAngle) - targetAngle;
//        } else {
            error = getZAngle() - targetAngle;
//        }
        imuDl.addField("start");
        imuDl.addField(targetAngle);
        imuDl.addField(getZAngle());
        imuDl.addField(lastError);
        imuDl.addField(error);
        imuDl.addField(derivative);
        imuDl.addField(rotationSpeed);
        imuDl.addField(iterations);
        imuDl.newLine();


        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= errorFactor) && opMode.opModeIsActive()) {
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                if ((rotationSpeed > -0.35) && (rotationSpeed < 0)) {
                    rotationSpeed = -0.32;
                } else if ((rotationSpeed < 0.4) && (rotationSpeed > 0)) {
                    rotationSpeed = 0.4;
                }

                r1Power = rotationSpeed;
                r2Power = rotationSpeed;
                l1Power = -rotationSpeed;
                l2Power = -rotationSpeed;

                setDrivePower(r1Power, r2Power, l2Power, l2Power);

                lastError = error;
                iterations++;

                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", r1Power);
                opMode.telemetry.addData("Left Front = ", l1Power);
                opMode.telemetry.addData("Left Rear = ", l2Power);
                opMode.telemetry.addData("Right Rear = ", r2Power);
                opMode.telemetry.update();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
//                if (targetAngle > 90 || targetAngle < -90) {
//                    error = gyro360(targetAngle) - targetAngle;
//                } else {
                    error = getZAngle() - targetAngle;

                imuDl.addField("inner");
                imuDl.addField(targetAngle);
                imuDl.addField(getZAngle());
                imuDl.addField(lastError);
                imuDl.addField(error);
                imuDl.addField(derivative);
                imuDl.addField(rotationSpeed);
                imuDl.addField(iterations);
                imuDl.newLine();

//                }

            }   // end of while Math.abs(error)
            motorsHalt();

            opMode.sleep(10);   // take 10 ms to allow gyro to settle
            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
            error = getZAngle() - targetAngle;
            imuDl.addField("outer");
            imuDl.addField(targetAngle);
            imuDl.addField(getZAngle());
            imuDl.addField(lastError);
            imuDl.addField(error);
            imuDl.addField(derivative);
            imuDl.addField(rotationSpeed);
            imuDl.addField(iterations);
            imuDl.newLine();

        }       // end of outside while loop

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
    }   //end of the PIDRotate Method

    /**
     * Method getZAngle()
     *  -   This method returns the gyro position of the robot.
     * @return zAngle
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method


    /**
     * Sets power to all four drive motors
     * @param R1 power for right front motor
     * @param L1 power for left front motor
     * @param L2 power for left rear motor
     * @param R2 power for right rear motor
     */
    public void setDrivePower(double R1, double R2, double L1, double L2){
        robot.motorR1.setPower(R1);
        robot.motorR2.setPower(R2);
        robot.motorL1.setPower(L1);
        robot.motorL2.setPower(L2);
    }   // end of the setDrivePower method

    public void driveTime(double power, double time) {
        ElapsedTime runTime = new ElapsedTime();
        double currentTime = runTime.time();

        while (((runTime.time() - currentTime) < time) && opMode.opModeIsActive()){

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            r1Power = Range.clip(power, -1,1);
            l1Power = Range.clip(power, -1,1);
            r2Power = Range.clip(power, -1,1);
            l2Power = Range.clip(power, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(r1Power, r2Power, l1Power, r2Power);

            opMode.idle();

        }   // end of while loop

        motorsHalt();

    } // close driveStraight method


    public void resetTSEBar(){
        robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_UP);
        robot.sweeperPink.setPosition(robot.PINK_SWEEPER_UP);
    }

    public void deployTSEBar(){
        robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_DOWN);
        robot.sweeperPink.setPosition(robot.PINK_SWEEPER_DOWN);
    }

    /*
     * Method motorsHalt
     *  -   stops all drive motors
     */
    public void motorsHalt(){
        robot.motorL1.setPower(0);
        robot.motorL2.setPower(0);
        robot.motorR1.setPower(0);
        robot.motorR2.setPower(0);
    }   // end of motorsHalt method

}       // end of DriveClass class
