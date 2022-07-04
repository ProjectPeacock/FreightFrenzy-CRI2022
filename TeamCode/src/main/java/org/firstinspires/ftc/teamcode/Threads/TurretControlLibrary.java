// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Threads;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class TurretControlLibrary implements Runnable{
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();
    private int turretTargetPosition = 0;

    private int sleepTime;
    private boolean isRunning=true;

    //constructor
    public TurretControlLibrary(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }

    // reset turret - returns turret to 'home' position
    public void resetTurret(){
        turretControl(0);
    }   // end of resetTurret

    public void setTargetPosition(int targetPosition){
        this.turretTargetPosition = targetPosition;
    }

    public int currentTurretPosition(){
        return localRobot.motorIntake.getCurrentPosition();
    }

    private void turretControl(int targetPosition){
        double integral = 0;
        double Cp = 0.0024;
        double Ci = 0.002;
        double Cd = 0.008;
        double maxSpeed = 1;
        double minSpeed =0.08;
        double rotationSpeed;
        double derivative = 0, lastError = 0;

        double error = targetPosition - localRobot.motorIntake.getCurrentPosition();

        // limit the turn position of the turret to the TURRET_MAX_POSITION to avoid
        // damaging the robot
        if(targetPosition > localRobot.TURRET_MAX_POSITION){
            targetPosition = localRobot.TURRET_MAX_POSITION;
        } else if(targetPosition < -localRobot.TURRET_MAX_POSITION){
            targetPosition = -localRobot.TURRET_MAX_POSITION;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) > 2) && this.isRunning) {
            derivative = lastError - error;
            rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

            // Clip servo speed
            rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

            // make sure the servo speed doesn't drop to a level where it is no longer able
            // to rotate
            if ((rotationSpeed < 0) && (rotationSpeed > -minSpeed)) {
                rotationSpeed = -minSpeed;
            } else if ((rotationSpeed > 0) && (rotationSpeed < minSpeed)) {
                rotationSpeed = minSpeed;
            }

            setTurretRotation(rotationSpeed);
            lastError = error;

            error = targetPosition - localRobot.motorIntake.getCurrentPosition();

        }   // end of while Math.abs(error)

        setTurretRotation(0);       // stop the turrets

    }   // end of turretControl() method

    public void setTurretRotation(double rotationSpeed){
        localRobot.turretServoBlue.setPower(-rotationSpeed);
        localRobot.turretServoPink.setPower(-rotationSpeed);
    }

//method that runs whenever thread is running
    public void activeTurretControl(){

        turretControl(this.turretTargetPosition);

    }
//end of default running method

    //thread stop method
    public void stop(){

        localRobot.turretServoBlue.setPower(0);
        localRobot.turretServoPink.setPower(0);
        this.isRunning = false;
    }

//thread run method
    @Override
    public void run() {
        while(this.isRunning){
            activeTurretControl();
            try{
                sleep(sleepTime);
            } catch(InterruptedException e){
                e.printStackTrace();
            }   // end of try{
        }   // end of while(isRunning)
    }   // end of run() method
}   // end of TurretControlLibrary Class
//end of thread run method