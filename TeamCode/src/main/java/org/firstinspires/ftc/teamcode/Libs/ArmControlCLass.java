// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Libs;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class ArmControlCLass {
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();

    private int angle1=0;
    private int angle2=0;
    private double arm1Power=1;
    private double arm2Power=1;
    private int sleepTime;
    private boolean isRunning=true;
    private double turretPower = 0.02;
    private int turretTargetPosition = 0;
    private int turretThreshold = 5;
    private int turretLimit = 330;
    private double timeElapsed;
    private int turretServoPower=0;

    //constructor
    public ArmControlCLass(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }

//initialize arms method
    public void initArms(){
        angle1 = 810;
        angle2=-364;
        setArmPosition();
    }

//deploy intake method
    public void intakeOn(){
        localRobot.motorArmAngle1.setPower(1);
        localRobot.motorArmAngle2.setPower(1);
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);
        angle1=localRobot.ARM_1_INTAKE;
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        while(localRobot.motorArmAngle1.getCurrentPosition()<900){

        }
        angle2=localRobot.ARM_2_INTAKE;
        localRobot.motorArmAngle2.setTargetPosition(angle2);
    }

    public void intakeOff(){
        angle1=0;
        localRobot.motorArmAngle1.setPower(1);
        localRobot.motorArmAngle2.setPower(1);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        while(localRobot.motorArmAngle1.getCurrentPosition()>300){

        }
        angle2=localRobot.ARM_2_RESTING_INTAKE;
        localRobot.motorArmAngle2.setTargetPosition(angle2);
        while(localRobot.motorArmAngle1.getCurrentPosition()<750){

        }
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
    }


    public void zintakeOn(){
/*
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);
        angle2=localRobot.ARM_2_INTAKE;
        localRobot.motorArmAngle2.setTargetPosition(angle2);
        while(localRobot.motorArmAngle2.getCurrentPosition()>1150){

        }
        angle1=localRobot.ARM_1_INTAKE;
        if(localRobot.motorArmAngle1.getCurrentPosition()>900) {
            localRobot.motorIntake.setPower(localRobot.INTAKE_POW);
        }
        localRobot.motorArmAngle1.setPower(arm1Power);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        localRobot.motorArmAngle2.setTargetPosition(angle2);

 */
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);
        if(localRobot.motorArmAngle1.getCurrentPosition()<0) {
            angle1 = 0;
        }
        angle1=localRobot.ARM_1_INTAKE;

        if(localRobot.motorArmAngle2.getCurrentPosition()>=65&&localRobot.motorArmAngle2.getCurrentPosition()<200){
            angle1=localRobot.ARM_1_INTAKE;
            localRobot.motorArmAngle1.setTargetPosition(angle1);
            while(localRobot.motorArmAngle1.getCurrentPosition()<300){

            }
            angle2=localRobot.ARM_2_INTAKE;
            localRobot.motorArmAngle2.setTargetPosition(angle2);
        }
    }       // end of intakeOn()

    public void beaterForward(){
        this.localRobot.motorIntake.setPower(localRobot.INTAKE_POW);
    }

    public void beaterReverse(){
        this.localRobot.motorIntake.setPower(-localRobot.INTAKE_POW);
    }
//end of deploy intake method

//retract intake method
    public void zintakeOff(){
        angle1=0;
        localRobot.motorArmAngle1.setPower(arm1Power);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        while(localRobot.motorArmAngle1.getCurrentPosition()<500) {
        }
        angle2=75;
        localRobot.motorArmAngle2.setPower(arm2Power);
        localRobot.motorArmAngle2.setTargetPosition(angle2);
        //waits for arm 1 to move up before moving intake to prevent collisions
        while(localRobot.motorArmAngle1.getCurrentPosition()<750) {
        }
        localRobot.motorIntake.setPower(0);
        localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
        localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
        localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
    }
//end of retract intake method

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
        angle1=-555;
        if(localRobot.motorArmAngle1.getCurrentPosition() < 750){
            angle2=1755;
        }
        setArmPosition();
    }
    //mid platform scoring
    public void scoringPos2(){
//        angle1=-230;
//        angle2=2411;
        angle1=-410;
        angle2=-364;
        setArmPosition();
    }
    //low platform & shared shipping hub scoring
    public void scoringPos3(){
        angle1=-717;
        angle2=-100;
        setArmPosition();
    }
//end of scoring positions methods

//hard arm reset method (DO NOT USE IF POSSIBLE)
    private void resetArm(){
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
    public void moveToZero(){
        angle1=0;
        angle2=0;
        setArmPosition();
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

// rotate turret
    public void rotateTurret(double power){
        // ensure turret within limit
        if (Math.abs(localRobot.turrentEncoder.getCurrentPosition())<turretLimit){
            localRobot.turretServoBlue.setPower(power);
            localRobot.turretServoPink.setPower(power);
        }
    }   // end of rotateTurret

    // reset turret - returns turret to 'home' position
    public void resetTurret(){
        while(Math.abs(localRobot.turrentEncoder.getCurrentPosition())>turretTargetPosition) {
            if(Math.abs(localRobot.turrentEncoder.getCurrentPosition())>50){
                if (localRobot.turrentEncoder.getCurrentPosition() > turretTargetPosition) {
                    turretPower = 0.3;
                } else {
                    turretPower = -0.3;
                }
            }else if(Math.abs(localRobot.turrentEncoder.getCurrentPosition())<=50){
                if (localRobot.turrentEncoder.getCurrentPosition() > turretTargetPosition) {
                    turretPower = 0.1;
                } else {
                    turretPower = -0.1;
                }
            }else if(Math.abs(localRobot.turrentEncoder.getCurrentPosition())<20){
                if (localRobot.turrentEncoder.getCurrentPosition() > turretTargetPosition) {
                    turretPower = 0.005;
                } else {
                    turretPower = -0.005;
                }
            }
            localRobot.turretServoBlue.setPower(turretPower);
            localRobot.turretServoPink.setPower(turretPower);
        }
    }   // end of resetTurret

//method that runs whenever thread is running
    public void setArmPosition(){
        localRobot.motorArmAngle1.setPower(arm1Power);
        localRobot.motorArmAngle2.setPower(arm2Power);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        localRobot.motorArmAngle2.setTargetPosition(angle2);
    } // end setArmPosition

}
//end of thread run method