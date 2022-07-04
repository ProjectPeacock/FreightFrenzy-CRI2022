// change comments

/*
12/7/21 added turret continuous servos: turret1 & turret2
        add turretEncoder - encoder added to inTakeMotor referneced via
        turretEncoder object
12/10/21 renamed turret servos
*/
package org.firstinspires.ftc.teamcode.HardwareProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareProfile022222 {

    /*
     * Constants
     */

    public final double DRIVE_TICKS_PER_INCH = 29.0;

    public final int ARM_THREAD_SLEEP=100; //arm thread sleepTime
//drive control constants
    public final double DRIVE_MULTIPLIER=0.5;
    public final double REVERSE_MULTIPLIER=0.75;
    public final double TURN_MULTIPLIER=0.75;

//intake control constants
    //intake servos
    public final double BLUE_ZERO = 0.50; //IntakeDeployBlue zero pos
    public final double PINK_ZERO = 0.50; //IntakeDeployPink zero pos
    public final double INTAKE_DEPLOY_BLUE = 0.275; //Distance to deploy intake
    public final double INTAKE_DEPLOY_PINK = INTAKE_DEPLOY_BLUE+0.145; // distance to deploy pink intake
    public final double INTAKE_OUTTAKE = 0.05; //Distance to deploy intake for outtaking
    public final double INTAKE_TILT_INPUT= 0.595;
    public final double INTAKE_STARTING_POS= 0.45;
    //intake motor
    public final double INTAKE_POW = 1; //intaking power
    public final double INTAKE_IDLE = 1; //intake idling power (for using intake as outtake)
    public final double INTAKE_REVERSE_POW = -0.8; //intake reverse power (for using intake as outtake)

//chainsaw control constants
    public final double CHAIN_POW = 0.75; //motorChainsaw power
    public final double CHAIN_POWER_BOOST = 0.05;
    public final int CHAIN_INCREMENTS=6; //increments for accelerating chainsaw to full speed

//arm control constants
    //arm intake positions
    public final int ARM_1_INTAKE=1050;
    public final int ARM_2_INTAKE=-25;

    //arm rest positions
    public final int ARM_2_RESTING_INTAKE=100;
    public final int ARM_2_RESTING_TSE=-150;

    //TSE angle adjustments
    public final int ARM2_TSE_ADJ=50;
    public final int ARM1_TSE_ADJ=50;

//turret constants
    public final int TURRET_MAX_POSITION = 275;
    public final int TURRET_INCREMENTS = 3;
    public final int TURRET_STEP=((275/TURRET_INCREMENTS)/10)*10;

//sweeper bar constants
//    public final double SWEEPER_INIT    = 0.5;
    public final double BLUE_SWEEPER_UP      = 0.8;
    public final double PINK_SWEEPER_UP      = .2;
    public final double BLUE_SWEEPER_DOWN    = 0.3;
    public final double PINK_SWEEPER_DOWN    = .7;

    /*
     * Hardware devices
     */

    //Motors
    public DcMotor motorR1 = null;  // Right Front Drive Motor
    public DcMotor motorL1 = null;  // Left Front Drive Motor
    public DcMotor motorL2 = null;  // Left Rear  Drive Motor
    public DcMotor motorR2 = null;  // Right Rear Drive Motor
    public DcMotorEx motorArmAngle2 = null; // Arm upper angle motor
    public DcMotorEx motorArmAngle1 = null; // Arm base angle motor
    public DcMotor motorChainsaw = null; // Chainsaw motor
    public DcMotor motorIntake = null; // Intake motor
    public DcMotor turrentEncoder = null;

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

    //Servos
    public Servo intakeDeployBlue = null; //Right intake deploy servo
    public Servo intakeDeployPink = null; //Left intake deploy servo
    public Servo intakeTilt = null; //Intake ramp position servo
    public Servo bucketDump = null; //Bucket turning servo
    public CRServo turretServoBlue = null;  // first turret servo
    public CRServo turretServoPink = null;  // second turret servo
    public Servo sweeperPink = null; //pink sweeper bar servo
    public Servo sweeperBlue = null; //blue sweeper bar servo

    //public distance sensors;
//    public DistanceSensor sensorDistPink=null;
//    public DistanceSensor sensorDistBlue=null;
    public DigitalChannel turretMagSensor = null;

    public DistanceSensor frontDistanceSensor=null;
  //  public RevBlinkinLedDriver LEDPort;

   // public WebcamName webcam = null;

    /* Constructor */
    public HardwareProfile022222() {

    }   // end of HardwareProfile method

    public void init(HardwareMap hwMap) {

//initialize DcMotor motors

        motorL1 = hwMap.dcMotor.get("motorL1");
        motorL1.setDirection(DcMotor.Direction.REVERSE);
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL1.setPower(0);

        motorL2 = hwMap.dcMotor.get("motorL2");
        motorL2.setDirection(DcMotor.Direction.REVERSE);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setPower(0);

        motorR1 = hwMap.dcMotor.get("motorR1");
        motorR1.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setPower(0);

        motorR2 = hwMap.dcMotor.get("motorR2");
        motorR2.setDirection(DcMotor.Direction.FORWARD);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setPower(0);

        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setPower(0);

        turrentEncoder = hwMap.dcMotor.get("motorIntake");

        motorChainsaw = hwMap.dcMotor.get("motorChainsaw");
        motorChainsaw.setDirection(DcMotor.Direction.FORWARD);
        motorChainsaw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorChainsaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChainsaw.setPower(0);

//initialize DcMotorEx motors (arm motors)

        motorArmAngle1 = hwMap.get(DcMotorEx.class,"motorArmAngle1");
        motorArmAngle1.setDirection(DcMotor.Direction.REVERSE);
        motorArmAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAngle1.setTargetPosition(0);
        motorArmAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle1.setPower(0);

        motorArmAngle2 = hwMap.get(DcMotorEx.class, "motorArmAngle2");
        motorArmAngle2.setDirection(DcMotor.Direction.REVERSE);
        motorArmAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAngle2.setTargetPosition(0);
        motorArmAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle2.setPower(0);

//initialize servos

        intakeDeployBlue = hwMap.servo.get("intakeDeployBlue");
        intakeDeployPink = hwMap.servo.get("intakeDeployPink");
        intakeTilt = hwMap.servo.get("intakeTilt");
        bucketDump = hwMap.servo.get("bucketDump");
        turretServoBlue = hwMap.crservo.get("turretServoBlue");
        turretServoPink = hwMap.crservo.get("turretServoPink");
        sweeperPink=hwMap.servo.get("sweeperPink");
        sweeperBlue=hwMap.servo.get("sweeperBlue");


/*
        motorOdometry = hwMap.dcMotor.get("motorOdometry");
        motorOdometry.setDirection(DcMotor.Direction.FORWARD);
        motorOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOdometry.setPower(0);

        /*
         * Initialize LED Controller
         */
  //      LEDPort = hwMap.get(RevBlinkinLedDriver.class, "LEDPort");
  //      LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

//initialize sensors
//        sensorDistBlue=hwMap.get(DistanceSensor.class, "sensorDistBlue");
//        sensorDistPink=hwMap.get(DistanceSensor.class, "sensorDistPink");
        frontDistanceSensor=hwMap.get(DistanceSensor.class,"frontDistanceSensor");
        turretMagSensor = hwMap.get(DigitalChannel.class, "turretMagSensor");
        turretMagSensor.setMode(DigitalChannel.Mode.INPUT);

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /* Webcam device will go here */
       // webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }   // end of init() method

}