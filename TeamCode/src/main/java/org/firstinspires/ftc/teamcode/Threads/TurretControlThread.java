// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Threads;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class TurretControlThread implements Runnable{
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();
    private int turretTargetPosition = 0;

    private int sleepTime;
    private boolean isRunning=true;

    double Cp = 0.0012;
    double Ci = 0.002;
    double Cd = 0.008;
    double minSpeed = -0.40;
    double maxSpeed = 1;
    public double rotationSpeed;
    double derivative = 0;
    public double lastError = 0;
    public double error = 0;


    //constructor
    public TurretControlThread(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime = 0;
//        this.sleepTime=threadSleepDelay;
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

    public void setLastError(double currentError){
        this.lastError = currentError;
    }

    private void turretControl(int targetPosition){

        error = this.turretTargetPosition - currentTurretPosition();

        // limit the turn position of the turret to the TURRET_MAX_POSITION to avoid
        // damaging the robot
        targetPosition = Range.clip(targetPosition, -275, 275);

        if ((Math.abs(error) > 3) && this.isRunning) {

            rotationSpeed = (((Cp * error) + (Cd*this.lastError))* maxSpeed);

            if (rotationSpeed > 0){
                rotationSpeed = Range.clip(rotationSpeed, 0.08, maxSpeed);
            }

            if (rotationSpeed < 0){
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, -0.08);
            }

            setTurretRotation(rotationSpeed);
            setLastError(error);

            error = this.turretTargetPosition - localRobot.motorIntake.getCurrentPosition();

        } else {

            setTurretRotation(0);       // stop the turrets

        }   // end of if Math.abs(error)

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