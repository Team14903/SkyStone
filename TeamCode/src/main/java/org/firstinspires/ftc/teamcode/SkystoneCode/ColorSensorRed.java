package org.firstinspires.ftc.teamcode.SkystoneCode;


import android.graphics.drawable.GradientDrawable;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name= "AutonomousRed")
//@Disabled
public class ColorSensorRed extends LinearOpMode {
    private double version = 2.3;

    private String Picture;

    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor linearSlideLeft;

    //Declare Servos
    private Servo armServo;
    private Servo TerrenceTheServo;
    private Servo NerrenceTheServo;
    private Servo LeftServoArm;
    private Servo RightServoArm;
    private Servo pullingArmServo;

    //Declare Sensors
    private ColorSensor SkystoneColorSensor;

    //Variables
    private double xLeftStick;
    private double yLeftStick;
    private double DistanceRobot;

    //Define Gyro
    BNO055IMU imu;

    public void runOpMode() throws InterruptedException {

        //Configure Sensors
        SkystoneColorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

        TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        LeftServoArm = hardwareMap.servo.get("Larm");
        RightServoArm = hardwareMap.servo.get("Rarm");
        pullingArmServo = hardwareMap.servo.get("pullingArmServo");

        //Configure motors to Expansion Hub
        linearSlideLeft = hardwareMap.dcMotor.get("linearSlideLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Configure Servos
        //armServo = hardwareMap.servo.get("motorArm");

        // Initialize Gyro
        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parametersGyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.loggingEnabled = false;
        parametersGyro.mode = BNO055IMU.SensorMode.IMU;
        parametersGyro.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu name");
        //Todo: find if initializing the code will be more accurate here or right before the code is begun
        imu.initialize(parametersGyro);
        float firstGyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Wait for Gyro Calibration", 0);
            telemetry.update();
            sleep(50);
            idle();
        }

        //vision init
        telemetry.addData("Version:", version);
        telemetry.addData("Ready for start", 0);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        //targetsSkyStone.activate();

        while (opModeIsActive()) {
            SkystoneColorSensor.enableLed(true);
            DriveInFWD(-.2,700);
            wait(500);
            if(SkystoneColorSensor.green()<100&&SkystoneColorSensor.red()<100&&SkystoneColorSensor.blue()<100){
                DriveInFWD(1,300);      //Drive Away from Skystone in position 1
                pullingArmServo.setPosition(0);       //Bring down arm

                DriveInFWD(-1,400);     //Drive towards skystone
                grabOrReleaseBlocks(true);            //grab skystone
                DriveInFWD(1,200);      //Drive away from skystone

                DriveSideways(1,800);   //Drive on other side of field
                linearSlideLeft.setTargetPosition(200);//lift linear slide
                DriveInFWD(1,100);       //drive towards foundation
                grabOrReleaseFoundation(true);         //grab foundation
                grabOrReleaseBlocks(false);            //release the block
                DriveInFWD(1,300);       //pull the foundation back
                TurnGyro(90,2,1,0);//turn foundation around
                grabOrReleaseFoundation(false);          //release foundation

                DriveInFWD(1,100);        //Drive away from foundation to bring down the arm
                linearSlideLeft.setTargetPosition(0);   //bring down linear slide
                DriveInFWD(1,200);        //cross the skybridge
                TurnGyro(-90,3,1,0);

                DriveInFWD(-1,400);       // push into the blocks
                grabOrReleaseBlocks(true);              //Grab skystone 2
                DriveInFWD(1,100);        //Drive away form skystone 2
                DriveSideways(1,300);     //Drive towards the foundation
                linearSlideLeft.setTargetPosition(200); //lift linear slide
                TurnGyro(90,5,1,0);//turn towards foundation
                grabOrReleaseBlocks(false);             //release skystone 2
                DriveInFWD(1,1000);       //push the foundation into the corner of the field
            }
            grabOrReleaseFoundation(false);
            Log.i("Debugger:", "Gyro Angle Before Moving:"+imu.getAngularOrientation());
            telemetry.addData("Status: Update Moving toward foundation", null);telemetry.update();
            DriveInFWD(.4, 880);
            Thread.sleep(500);
            Log.i("Debugger:", "Gyro Angle After Moving:"+imu.getAngularOrientation());
            //TurnGyro(10,2,-.2,0);
            grabOrReleaseFoundation(true);
            Thread.sleep(500);

            telemetry.addData("Status: Update Moving foundation", null);telemetry.update();
            DriveInFWD(-.4, 880);
            TurnGyro(270,1,-.5,0);
            grabOrReleaseFoundation(false);
            pullingArmServo.setPosition(0);
            DriveInFWD(-1,400);



            telemetry.update();
            requestOpModeStop();


        }
    }

    // This is a method to make code easier to read, see above
    /*public void DriveInStraightLine(double Power, double distance, double angle) throws InterruptedException {
        zeroMotorPower();//Makes sure that the motor isn't moving before the function begins

        xLeftStick = Math.cos(angle);
        yLeftStick = Math.sin(angle);
        DistanceRobot = 0;
        //double initialPosition = (Math.abs(motorBackLeft.getCurrentPosition())+ Math.abs(motorBackRight.getCurrentPosition())+Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition()))/4;
        double initialPosition = 0;
        while (statementFunction(DistanceRobot, initialPosition, distance)) {
            motorFrontLeft.setPower(Range.clip(-yLeftStick + xLeftStick, -1, 1) * Power);
            motorFrontRight.setPower(Range.clip(yLeftStick + xLeftStick, -1, 1) * Power);
            motorBackLeft.setPower(Range.clip(-yLeftStick - xLeftStick, -1, 1) * Power);
            motorBackRight.setPower(Range.clip(yLeftStick - xLeftStick, -1, 1) * Power);

            DistanceRobot = (Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition()) + Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition())) / 4;
            telemetry.addData("Robot Distance: ", DistanceRobot);
            telemetry.addData("Target Distance: ", distance);

            telemetry.update();

        }
        zeroMotorPower();
    }*/

    public void DriveInFWD(double Power, double distance) throws InterruptedException {
        zeroMotorPower();//makes sure the robot isn't moving before the function begins
        resetPosition();
        DistanceRobot = 0;
        //While the distance the robot traveled is less than the target distance...
        while (DistanceRobot < distance) {
            //Set the power to the motors to drive fowards or backwards
            motorFrontLeft.setPower(-Power);
            motorFrontRight.setPower(Power);
            motorBackLeft.setPower(-Power);
            motorBackRight.setPower(Power);
            //Store the current distance that the motor encoders traveled
            DistanceRobot = (Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition()) + Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition())) / 4;
            //Log and telemetry to debug any issues
            telemetry.addData("Robot Distance: ", DistanceRobot);
            telemetry.addData("Target Distance: ", distance);
            telemetry.update();
        }
        //Set the motor power to zero before ending the function
        zeroMotorPower();
    }

    public void DriveSideways(double Power, double distance) throws InterruptedException {
        zeroMotorPower(); // Set the motor power to zero before beginning the function
        resetPosition();
        DistanceRobot = 0;
        //Set the initial distance of the encoders before moving the robot
        //While the distance traveled by the robot is less than the target distance...
        while (DistanceRobot < distance) {
            //Set motor powers needed to move sideways
            motorFrontLeft.setPower(Power);
            motorFrontRight.setPower(Power);
            motorBackLeft.setPower(-Power);
            motorBackRight.setPower(-Power);
            //Measure the current distance traveled using the average absolute value of the encoders
            DistanceRobot = (Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition()) + Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition())) / 4;
            //Telemetry statements to debug code
            telemetry.addData("Robot Distance: ", DistanceRobot);
            telemetry.addData("Target Distance: ", distance);
            telemetry.update();
        }

        zeroMotorPower(); //Set the power of the motors to zero before starting the next function
    }
    public void TurnGyro(float degrees, float precision, double regurlarPower, float relativeToStart) {
        //Get the gyro reading from the IMU
        float gyroReading = imu.getAngularOrientation().firstAngle;
        //Adding previous value of the gyro to the target to find end position
        gyroReading = gyroReading < 0 ? (float)(2*Math.PI) + gyroReading : gyroReading;
        //Allows for a relative to start position
        float totalDegrees = relativeToStart != 0 ? degrees/180*(float)(Math.PI) + relativeToStart : degrees/180*(float)(Math.PI) + gyroReading;

        totalDegrees = totalDegrees % (float)(2*Math.PI);
        //Log information for Debugging
        Log.i("GyroDebug", "CurrentAngle: " + gyroReading + "   target: " + totalDegrees);


        //Set motors in opposite directions according to (+ or -)degrees
        double left = 1;
        double right = 1;
        //If degrees is negative = clockwise
        if (totalDegrees - gyroReading < 0) {
            left = -1;right = -1;
            //If degrees is Positive = anti-clockwise
        } else if (totalDegrees - gyroReading > 0) {
            left = 1;right = 1;
        }
        //While the gyro is not greater or less than the target ...
        // Math.Abs(totalDegrees - gyroDegrees) < precision
        //while(gyroReading>totalDegees+precision || gyroReading<totalDegees-precision){
        int counter = 0;
        while (Math.abs(totalDegrees - gyroReading) > precision*(float)(Math.PI)/180 && opModeIsActive()) {
            //turn the motors
            motorBackLeft.setPower(left * regurlarPower);
            motorFrontLeft.setPower(left * regurlarPower);
            motorBackRight.setPower(right * regurlarPower);
            motorFrontRight.setPower(right * regurlarPower);
            telemetry.addData("Gyro angle", (totalDegrees - gyroReading));
            telemetry.update();
            //Update Gyro position
            gyroReading = imu.getAngularOrientation().firstAngle;
            gyroReading = gyroReading < 0 ? (float)(2*Math.PI) + gyroReading : gyroReading;
            Log.i(counter++ + ". CurrentAngle", String.valueOf(gyroReading));
        }
        zeroMotorPower();//Set power of motors to zero before starting function
        Log.i("Final Angle", String.valueOf(gyroReading));
    }

    public void zeroMotorPower() {
        //motorFrontLeft.setPower(0);
        //motorFrontRight.setPower(0);
        //motorBackLeft.setPower(0);
        //motorBackRight.setPower(0);
    }
    public void resetPosition(){
        telemetry.addData("Currently: reseting encoders",null);
        telemetry.update();
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Done: reseting encoders",null);
        telemetry.update();
    }
    public void grabOrReleaseFoundation(boolean grab){
        if(grab){
            TerrenceTheServo.setPosition(0);
            NerrenceTheServo.setPosition(1);
        } else {
            TerrenceTheServo.setPosition(1);
            NerrenceTheServo.setPosition(0);
        }
    }
    public void grabOrReleaseBlocks(boolean grab){
        if(grab){
            RightServoArm.setPosition(0);
            LeftServoArm.setPosition(1);
        } else {
            RightServoArm.setPosition(1);
            LeftServoArm.setPosition(0);
        }
    }
}