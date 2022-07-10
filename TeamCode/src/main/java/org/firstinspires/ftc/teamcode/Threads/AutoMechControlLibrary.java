// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Threads;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class AutoMechControlLibrary implements Runnable{
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();

    private int angle1=0;
    private int angle2=0;
    private double arm1Power=1;
    private double arm2Power=1;
    private int sleepTime;
    private boolean isRunning=true;
    public int armAction=0;


    //constructor
    public AutoMechControlLibrary(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }

    public void moveToZero(){
        this.angle1=0;
        this.angle2=0;
    }
//deploy intake method
    public void intakeOn(){
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);



        if(localRobot.motorArmAngle1.getCurrentPosition()<0) {
            this.angle1 = 0;
        }
        if(localRobot.motorArmAngle2.getCurrentPosition()>1000){
            this.angle2=localRobot.ARM_2_RESTING_INTAKE;
        }
        if(localRobot.motorArmAngle2.getCurrentPosition()>=65 && localRobot.motorArmAngle2.getCurrentPosition()<200){
            this.angle1=localRobot.ARM_1_INTAKE;
            if(localRobot.motorArmAngle1.getCurrentPosition()<750){
                arm1Power=0.75;
            }else{
                arm1Power=1;
            }
            if(localRobot.motorArmAngle1.getCurrentPosition()>300){
                this.angle2=localRobot.ARM_2_INTAKE;
            }
        }


        beaterOn(true);
    }
//end of deploy intake method

    private void beaterOn(boolean deployed){
        if(deployed&&localRobot.motorArmAngle1.getCurrentPosition()>900){
            localRobot.motorIntake.setPower(localRobot.INTAKE_POW);
        }
    }

//retract intake method
    private void intakeOff(boolean deployed, boolean TSEMode){
        localRobot.motorIntake.setPower(0);
        if(!deployed){
            angle1=0;
            if(localRobot.motorArmAngle1.getCurrentPosition()<500) {
                if(TSEMode) {
                    angle2 = localRobot.ARM_2_RESTING_TSE;
                }else{
                    angle2=localRobot.ARM_2_RESTING_INTAKE;
                }
            }
        }

        //waits for arm 1 to move up before moving intake to prevent collisions
        if(localRobot.motorArmAngle1.getCurrentPosition()<750){
            localRobot.motorIntake.setPower(0);
            localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
            localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
            localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
        }
    }
//end of retract intake method

//move to scoring positions methods
    //high platform scoring (default)
    public void scoringPos3(){
        arm1Power=0.50;
        arm2Power=0.75;
        if(localRobot.motorArmAngle2.getCurrentPosition()>-750) {
            angle1 = -250;
        }
        angle2=-856;
    }
    //mid platform scoring
    public void scoringPos2(){
        arm1Power=0.6;
        arm2Power=0.6;
        angle1=-500;
        angle2=-470;
    }
    //low platform & shared shipping hub scoring
    public void scoringPos1(){
        arm1Power=0.75;
        arm2Power=0.75;
        angle1=-936;
        if(localRobot.motorArmAngle1.getCurrentPosition()<750) {
            angle2 = -132;
        }
    }
//end of scoring positions methods

//manually set power for arm motors
    public void setPower(double a, double b){
        arm1Power=a;
        arm2Power=b;
    }
//end of manually set power for arm motors

    //return motorArmAngle1 position for auto
    public int getEncoderAngle1(){
        return localRobot.motorArmAngle1.getCurrentPosition();
    }

    //return motorArmAngle2 position for auto
    public int getEncoderAngle2(){
        return localRobot.motorArmAngle2.getCurrentPosition();
    }

//set custom position for arm motors
    public void setAutoPosition(int a, int b){
        localRobot.motorArmAngle1.setTargetPosition(a);
        localRobot.motorArmAngle2.setTargetPosition(b);
    }
//end of set custom position for arm motors

//chainsaw acceleration for Blue Alliance
    public void chainsawRampBlue(){
        for(int i=localRobot.CHAIN_INCREMENTS; i>0;i--){
            localRobot.motorChainsaw.setPower(-localRobot.CHAIN_POW/i);
        }
    }
//end of chainsaw acceleration for Blue Alliance

//chainsaw acceleration for Red Alliance
    public void chainsawRampRed(){
        for(int i=localRobot.CHAIN_INCREMENTS; i>0;i--){
            localRobot.motorChainsaw.setPower(localRobot.CHAIN_POW/i);
        }
    }
//end of chainsaw acceleration for Red Alliance

    /**
     * Method: setArmAction
     * @param a   - Indicates which Arm Action to take
     *            action 0: Null
     *            action 1: intakeOn
     *            action 2: intakeOff
     */

    public void setArmAction(int a){
        this.armAction=a;
    }
//method that runs whenever thread is running

    public void activeMechControl(){
        //action 1: intakeOn
        //action 2: intakeOff
        if(this.armAction==1){
            intakeOn();
        }else if(this.armAction==2){
            intakeOff(false,false);
        }

        localRobot.motorArmAngle1.setPower(arm1Power);
        localRobot.motorArmAngle2.setPower(arm2Power);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        localRobot.motorArmAngle2.setTargetPosition(angle2);
    }
//end of default running method

    //thread stop method
    public void stop(){
        isRunning=false;
    }

//thread run method
    @Override
    public void run() {
        while(isRunning){
            activeMechControl();
            try{
                sleep(sleepTime);
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
}
//end of thread run method