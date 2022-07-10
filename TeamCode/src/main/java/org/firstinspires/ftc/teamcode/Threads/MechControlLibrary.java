// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Threads;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class MechControlLibrary implements Runnable{
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();

    private int angle1=0;
    private int angle2=0;
    private double arm1Power=1;
    private double arm2Power=1;
    private int sleepTime;
    private boolean isRunning=true;

    //constructor
    public MechControlLibrary(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }

//deploy intake method
    public void intakeOn(boolean deployed, double currentTime){
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);
        while (runtime.time() - currentTime < 1.15) {
        }

        if(localRobot.motorArmAngle1.getCurrentPosition()<0) {
            angle1 = 0;
        }
        if(localRobot.motorArmAngle2.getCurrentPosition()>1000){
            angle2=localRobot.ARM_2_RESTING_INTAKE;
        }
        if(localRobot.motorArmAngle2.getCurrentPosition()>=65&&localRobot.motorArmAngle2.getCurrentPosition()<200){
            angle1=localRobot.ARM_1_INTAKE;
            if(localRobot.motorArmAngle1.getCurrentPosition()<750){
                arm1Power=0.75;
            }else{
                arm1Power=1;
            }
            if(localRobot.motorArmAngle1.getCurrentPosition()>300){
                angle2=localRobot.ARM_2_INTAKE;
            }
        }
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        localRobot.motorArmAngle2.setTargetPosition(angle2);
    }
//end of deploy intake method

    public void beaterOn(boolean deployed, int angle){
        if(deployed&&angle>900){
            localRobot.motorIntake.setPower(localRobot.INTAKE_POW);
        }
    }

//retract intake method
    public void intakeOff(boolean deployed, boolean TSEMode){
        if(localRobot.motorArmAngle1.getCurrentPosition()>800){
            localRobot.motorIntake.setPower(-1);
        }
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


    public void coopArmExtend(){

    }

//reset intake without moving arm (for retracting intake while scoring)
    public void resetIntake(){
        if(localRobot.motorArmAngle1.getCurrentPosition()<750){
            localRobot.motorIntake.setPower(0);
            localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
            localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
            localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
        }
    }
//end of rest intake without moving arm

//move to scoring positions methods
    //high platform scoring (default)
    public void scoringPos1(){
        arm1Power=1;
        arm2Power=1;
        angle1=-555;
        if(localRobot.motorArmAngle1.getCurrentPosition()<750){
            angle2=1755;
        }
    }
    //mid platform scoring
    public void scoringPos2(){
        arm1Power=1;
        arm2Power=1;
        angle1=-190;
        angle2=2320;
    }
    //low platform & shared shipping hub scoring
    public void scoringPos3(){
        arm1Power=0.75;
        arm2Power=0.75;
        angle1=-1250;
        angle2=-755;
    }
//end of scoring positions methods

//TSE positions
    public void TSEDown(){
        arm1Power=0.5;
        arm2Power=0.5;
        angle2=localRobot.ARM_2_RESTING_TSE;
        if(localRobot.motorArmAngle2.getCurrentPosition()<50) {
            angle1 = -969;
        }
    }
    public  void TSEresting(){
        angle1=195;
        angle2=-739;
    }

    public void TSEtop(){
        angle1=-456;
        angle2=-1116;
    }

    //TSE Bumper Controls, GP2, Bumpers
    public void TSEBumperUp(){angle2 = localRobot.motorArmAngle2.getCurrentPosition() + localRobot.ARM2_TSE_ADJ;}
    public void TSEBumperDown(){
        angle2 = localRobot.motorArmAngle2.getCurrentPosition() - localRobot.ARM2_TSE_ADJ;
    }

    //TSE Trigger Controls, GP2, triggers
    public void TSETriggerUp(){angle1 = localRobot.motorArmAngle1.getCurrentPosition() + localRobot.ARM1_TSE_ADJ;}
    public void TSETriggerDown(){angle1 = localRobot.motorArmAngle1.getCurrentPosition() - localRobot.ARM1_TSE_ADJ;}

//hard arm reset method (DO NOT USE IF POSSIBLE)
    public void resetArm(){
        angle1=0;
        angle2=2786; //moves arm 2 to one full rotation
        while(localRobot.motorArmAngle2.getCurrentPosition()!=angle2){

        }
        localRobot.motorArmAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localRobot.motorArmAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
//end of hard arm reset method

//soft arm reset method (returns to original zero)
    //moves arm to 0 (over the top)
    public void moveToZero(boolean TSEMode){
        angle1=0;
        if(TSEMode) {
            angle2 = localRobot.ARM_2_RESTING_TSE;
        }else{
            angle2 = localRobot.ARM_1_INTAKE;
        }
    }
//end of soft arm reset method
//arm 2 position manual increment methods
    public void incrementUp(){
        angle2+=5;
    }
    public void incrementDown(){
        angle2-=5;
    }
//end of arm 2 manual increment methods

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

//turret encoder centered check
    public boolean turretCentered(){
        return Math.abs(localRobot.turrentEncoder.getCurrentPosition())<6;
    }
//method that runs whenever thread is running
    public void activeMechControl(){
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