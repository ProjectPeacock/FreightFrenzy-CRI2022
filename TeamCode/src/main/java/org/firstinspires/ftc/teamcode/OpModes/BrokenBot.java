package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Broken Bot TS", group = "Troubleshooting")
  //  @Disabled

    public class BrokenBot extends LinearOpMode {

        private final static HardwareProfile robot = new HardwareProfile();
        private LinearOpMode opMode = this;

        public BrokenBot(){


        }   // end of BrokenBotTS constructor

        public void runOpMode(){
            DriveClass drive = new DriveClass(robot, opMode);
            //MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
            //Thread mechController = new Thread(mechControl);

/*
            double currentTick, currentTime, currentRPM;
            boolean servoGrabFlag=false, servoKickFlag=false, servoTransferFlag=false, servoIntakeFlag=false, servoRingFlag=false;
            double armPosition = 0.5;


            boolean intake = false,             // tracks whether the user has enabled or disabled the intake system
                    intakeForward = true,       // tracks which direction the intake system should go
                    grabOpen = false,           // tracks whether the grabber should be open or closed
                    intakeDeployed = false;     // tracks whether the intake should be out or in
            ElapsedTime runTime = new ElapsedTime();
            boolean shooter = false;
            double shooterPower = 0.80;
            double buttonPress = 0;
            boolean readyToShoot = false;
            boolean motorIntakeFlag = false;
*/
            int turretAngle=0;
            telemetry.addData("Robot State = ", "NOT READY");
            telemetry.update();

            /*
             * Setup the initial state of the robot
             */
            robot.init(hardwareMap);
            ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);



            /*
             * Initialize the drive class
             */
            robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_UP);
            robot.sweeperPink.setPosition(robot.PINK_SWEEPER_UP);

            /*
             * Calibrate / initialize the gyro sensor
             */
            telemetry.addData("Robot state = ", "INITIALIZED");
            telemetry.update();


            robot.motorArmAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorArmAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double l1Start = robot.motorL1.getCurrentPosition();
            double l2Start = robot.motorL2.getCurrentPosition();
            double r1Start = robot.motorR1.getCurrentPosition();
            double r2Start = robot.motorR2.getCurrentPosition();

            waitForStart();
            //mechController.start();
            while(opModeIsActive()) {
                telemetry.addData("Pink DistanceUnit (CM)", robot.frontDistanceSensorPink.getDistance(DistanceUnit.CM));
                telemetry.addData("Bucket DistanceUnit (CM)", robot.bucketSensor.getDistance(DistanceUnit.MM));
                if (robot.bucketSensor.getDistance(DistanceUnit.MM) <150){
                    telemetry.addData("Element Detected = ", "true");
                } else {
                    telemetry.addData("Element Detected = ", "false");
                }
                telemetry.addData("Arm Angle 1 = ", robot.motorArmAngle1.getCurrentPosition());
                telemetry.addData("Arm Angle 2 = ", robot.motorArmAngle2.getCurrentPosition());
                telemetry.addData("Turret Angle = ",robot.turrentEncoder.getCurrentPosition());
                telemetry.addData("Distance Traveled = ", drive.calcDistance(r1Start, r2Start, l1Start, l2Start));

                if(gamepad2.dpad_left){
                    drive.driveTurn(-90, 2);
                }
                if(gamepad2.dpad_right) {
                    drive.driveTurn(90, 2);
                }
                if (gamepad2.dpad_down){
                    drive.driveTurn(0, 2);
                }
                if(gamepad2.dpad_up){
                    armControl.scoringPos3();
                }
                if(gamepad2.right_bumper){
                    armControl.scoringPos2();
                }
                if(gamepad2.left_bumper){
                    armControl.scoringPos1();
                }
                if (gamepad1.dpad_up) {
                    robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_UP);
                    robot.sweeperPink.setPosition(robot.PINK_SWEEPER_UP);
                    telemetry.addData("sweeper = ", "Up");
                }
                if (gamepad1.dpad_down) {
                    robot.sweeperBlue.setPosition(robot.BLUE_SWEEPER_DOWN);
                    robot.sweeperPink.setPosition(robot.PINK_SWEEPER_DOWN);
                    telemetry.addData("sweeper = ", "Down");
                }

            /*
                telemetry.addData("Drive Motor Encoders:",robot.motorR1.getCurrentPosition());if(robot.sensorDistPink.getDistance(DistanceUnit.CM)<60){
                    telemetry.addData("Pink shipping element present","");
                }
                if(robot.sensorDistBlue.getDistance(DistanceUnit.CM)<60){
                    telemetry.addData("Blue shipping element present","");
                }
                telemetry.addData("Pink distance:",robot.sensorDistPink.getDistance(DistanceUnit.CM));

                telemetry.addData("Blue distance:",robot.sensorDistBlue.getDistance(DistanceUnit.CM));

                telemetry.addData("Turret encoder:",robot.turrentEncoder.getCurrentPosition());
*/
                /*
                 * Mecanum Drive Control section
                double drive = -gamepad1.left_stick_y;
                double turn  =  gamepad1.right_stick_x;

                // Combine drive and turn for blended motion.
                double left  = drive + turn;
                double right = drive - turn;

                // Normalize the values so neither exceed +/- 1.0
                double max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0)
                {
                    left /= max;
                    right /= max;
                }

                // Output the safe vales to the motor drives.
                robot.motorL1.setPower(left);
                robot.motorL2.setPower(left);
                robot.motorR1.setPower(right);
                robot.motorR2.setPower(right);


                /**
                 * ##############################################################################
                 * ##############################################################################
                 * ################    GAMEPAD 1 NORMAL CONTROLS   #############################
                 * ##############################################################################
                 * ##############################################################################



                /**
                 * ##############################################################################
                 * ##############################################################################
                 * ################    GAMEPAD 2 TESTING CONTROLS   #############################
                 * ##############################################################################
                 * ##############################################################################


                if (gamepad2.dpad_down) {
                    robot.motorL2.setPower(1);
                    telemetry.addData("Motor = ", "MotorL2");
                }

                if (gamepad2.dpad_up) {
                    robot.motorL1.setPower(1);
                    telemetry.addData("Motor = ", "MotorL1");
                }

                if (gamepad2.dpad_right) {
                    robot.motorR2.setPower(1);
                    telemetry.addData("Motor = ", "MotorR2");
                }

                if(gamepad2.dpad_left) {
                    robot.motorR1.setPower(1);
                    telemetry.addData("Motor = ", "MotorR1");
                }

                //intake controls (GP1, A button and Y Button)
                if(gamepad1.a) {
                    robot.motorIntake.setPower(robot.INTAKE_POW);
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO-robot.INTAKE_DEPLOY_BLUE); //subtracting because it needs to rotate counterclockwise
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO+robot.INTAKE_DEPLOY_PINK); //adding because it needs to rotate clockwise
                }else if(gamepad1.y) {
                    robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO - robot.INTAKE_OUTTAKE); //counterclockwise
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO + robot.INTAKE_OUTTAKE); //clockwise
                }else if(gamepad1.x) {
                    robot.motorIntake.setPower(0);
                }else{
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
                }


                /*
                 * Toggle the intake forward and reverse based on user pressing gamepad1.y

                if (gamepad2.right_trigger > 0.2 && (currentTime - buttonPress) >= 0.3){
                    motorIntakeFlag = true;
                    intakeForward = true;      // returns opposite of flag setting
                    buttonPress = currentTime;      // update the time that the button was pressed
                }   else if (gamepad2.left_trigger > 0.2 && (currentTime - buttonPress) >= 0.3) {
                    motorIntakeFlag = true;
                    intakeForward = false;
                    buttonPress = currentTime;      // update the time that the button was pressed
                }   // end of if(gamepad2.right_trigger)

                if (gamepad2.left_bumper && (currentTime - buttonPress) >= 0.3){
                    shooter = toggleFlag(shooter);
                }


                if (gamepad2.right_stick_y > 0.2){
                    armPosition = armPosition + 0.05;
                } else if (gamepad2.right_stick_y < -0.2) {
                    armPosition = armPosition - 0.05;
                }
                */
                /*
                turretAngle=robot.turrentEncoder.getCurrentPosition();
                if (gamepad2.right_trigger > 0.05&&turretAngle>=-robot.TURRET_MAX_POSITION) {
                    robot.turretServoBlue.setPower(gamepad2.right_trigger/2);
                    robot.turretServoPink.setPower(gamepad2.right_trigger/2);
                } else if (gamepad2.left_trigger > 0.05&&turretAngle<=robot.TURRET_MAX_POSITION) {
                    robot.turretServoBlue.setPower(-gamepad2.left_trigger/2);
                    robot.turretServoPink.setPower(-gamepad2.left_trigger/2);
                }else{
                    robot.turretServoBlue.setPower(0);
                    robot.turretServoPink.setPower(0);
                }
                if(gamepad2.y){
                    mechControl.resetTurret();
                }
*/
/*
                if(robot.turretMagSensor.getState()==true){
                    telemetry.addData("Turret mag limit","not detected");
                }else{
                    telemetry.addData("Turret mag limit","detected");
                }
                telemetry.update();
                /**
                 * ##############################################################################
                 * ##############################################################################
                 * #####################    APPLY CONTROL ACTIONS   #############################
                 * ##############################################################################
                 * ##############################################################################
                 */
                telemetry.addData("IMU Angle: ",drive.getZAngle());

                telemetry.update();

                /**
                 * #################################################################################
                 * #################################################################################
                 * #################      PROVIDE USER FEEDBACK    #################################
                 * #################################################################################
                 * #################################################################################
                 */
            }   // end of while opModeIsActive()
            //mechControl.stop();
        }   // end of runOpMode method

        boolean toggleFlag(boolean flag){
            if (flag) return false;
            else return true;
        }   // end of toggleFlag function
    }   // end of BrokenBot.java class


