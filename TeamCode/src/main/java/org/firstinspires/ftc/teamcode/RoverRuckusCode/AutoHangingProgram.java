package org.firstinspires.ftc.teamcode.RoverRuckusCode;

import android.os.Build;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous
//@Disabled
public class AutoHangingProgram extends LinearOpMode{
    private double version = 2.3;

    private String Picture;

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;

    //Declare Servos
    private Servo armServo;


    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double linearSlidePower = 1; //Power for linear slide motors
    private int secondsRun = 7;

    //Define Gyro
    BNO055IMU imu;

    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AecVx1T/////AAABmcI08yL6hk3Nt6hSJ1mD2XJhGX9s+cETNcXvwYRsviVOyFrvCxQW1uKq6eKCuXJcp6loA3K4kq5vdzjlG5oDest+XgLnl7uUO2EHXUsl63893eKJ8anaiFk1+ECVV8+Nx2UJWNLk1enRgDkMzhA38qnIfcw+D5ll48etLKc1aWkOS40ZHoIscDctrsSWxFI6O8fDMHF8M4OrqYUd1kHPOj+5hNA6C2+BKn72JY53ABrveqy2gYtyBTkBQsZbGU7tKnJb4xvNcqk2q1IINiKx/lkoPUuJWbtG7xMJta/DySf+xjcggD63vooQ6Lz9HYMNml+IhogXQQwgGwssPukwUg+snYX17rq4fHNPw26aLN8J";

        //Configure Sensors
        latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        latchingTouchSensorUp = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorUp");
        latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);
        latchingTouchSensorDown.setMode(DigitalChannel.Mode.INPUT);


        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLatching = hardwareMap.dcMotor.get("motorLatching");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLatching.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        // Initialize motors
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize Gyro
        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit       = BNO055IMU.AngleUnit.DEGREES;
        parametersGyro.accelUnit       = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.loggingEnabled  =false;
        parametersGyro.mode            =BNO055IMU.SensorMode.IMU;
        parametersGyro.loggingTag      ="IMU";
        imu                        =hardwareMap.get(BNO055IMU.class, "imu name");
        imu.initialize(parametersGyro);
        float firstGyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        //vision init
        telemetry.addData("Version:",version);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            //while upper limit isn't reached(robot hasn't lowed itself)...
            while (latchingTouchSensorUp.getState()) {
                motorLatching.setPower(linearSlidePower);
            }
            motorLatching.setPower(0.3);

            //Turn robot toward depot
            TurnGyro(175, 2, 0.8,0);
            Thread.sleep(50, 0);
            motorLatching.setPower(0);
            // Find the gold
            goldFinder();

            //Switch Case OLD CODE!!!!!!!!!!!!
            /* switch (goldPosition) {
                case LEFT:
                    Thread.sleep(20,0);
                    TurnGyro(31,2,0.8,0);
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,660,0.3,1);
                    Thread.sleep(20,0);
                    TurnGyro(-52,2,0.8,0);
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,860,0.3,1);
                    Thread.sleep(20,0);
                    TurnGyro(11,2,0.8,0);
                    Thread.sleep(20,0);

                    break;
                case RIGHT:
                    Thread.sleep(20,0);
                    TurnGyro(-31,2,0.8,0);
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,660,0.3,1);
                    Thread.sleep(20,0);
                    TurnGyro(52,2,0.8,0);
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,860,0.3,1);
                    Thread.sleep(20,0);
                    TurnGyro(-11,2,0.8,0);
                    Thread.sleep(20,0);
                    break;
                case CENTER:
                    Thread.sleep(20,0);
                    TurnGyro(-5,2,0.5,0);
                    //Move toward Depot while knocking out center game piece(Sampling Mission)
                    //Distance Total 1366 Counts or 52.8inches ~1400
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,1500,0.3,1);
                    break;
                case UNKNOWN:
                    Thread.sleep(20,0);
                    TurnGyro(-5,2,0.5,0);
                    //Move toward Depot while knocking out center game piece(Sampling Mission)
                    //Distance Total 1366 Counts or 52.8inches ~1400
                    Thread.sleep(20,0);
                    DriveAccFwdDcc(100,1500,0.3,1);
                    break;
            }
            vision.shutdown();*/
            //Place team marker
            Thread.sleep(20,0);
            armServo.setPosition(0);
            Thread.sleep(1000);
            armServo.setPosition(0.5);
            Thread.sleep(1000);
            armServo.setPosition(0);
            //Log.d("servo","back");
            //Turn robot towards the crater
            Thread.sleep(10,0);
            TurnGyro((360-145),1,0.8,firstGyroAngle);
            Thread.sleep(20,0);
            //Move until hits crater(sense reduction of speed)
            DriveFwdAccDcc(0.3,1,100);
            DriveFwdObstacle(1,0.9);
            //A bunch of telemetry data for quick debugging
            Log.i("Mission","Completed!");
            telemetry.addData("Wheel Encoder", (motorLeft.getCurrentPosition()+motorRight.getCurrentPosition())/2);
            telemetry.addData("Gyro angle",imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES));
            telemetry.addData("Right Wheel Encoder", motorRight.getCurrentPosition());
            telemetry.addData("Left Wheel Encoder", motorLeft.getCurrentPosition());
            telemetry.update();
            //Log.i("Vuforia",Picture);
            requestOpModeStop();

        }
    }
    // This is a method to make code easier to read, see above
    public void DriveFwdDistance(double Power,int distance,boolean stop) throws InterruptedException {
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Get current average motor position
        // Core hex motor with 90mm wheel is 0.03865148441 inches per count
        int motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;

        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition && !isStopRequested()) {
            //set power to drive motors
            motorLeft.setPower(-Power);
            motorRight.setPower(Power);
            //update the average position of the motors
            motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
            //Telemetry for debugging
            telemetry.addData("motorPosition",motorPosition);
            telemetry.addData("Distance",distance);
            telemetry.update();
        }
        //Stops motors at power 0 if requested using the stop boolean variable
        if(stop) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }
    public void DriveFwdAccDcc(double powerInitial, double powerFinal,int distance) throws InterruptedException {

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Get current average motor position
        // Core hex motor with 90mm wheel is 0.03865148441 inches per count
        int motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
        //Create a variable called powerCurrent for the current power
        double powerCurrent;
        //Use previous motor distance in calculation
        int distanceModified=distance+motorPosition;

        //Run while motor distance is less than target
        while (distanceModified>motorPosition&&opModeIsActive()) {
            //Set powerCurrent to the (changeInPower/distance)x(motorDistanceTraveled)
            //to make a linear increase in power.
            powerCurrent = ((powerFinal-powerInitial)/distance)*(distanceModified-motorPosition);
            //Set power to the motors
            motorLeft.setPower(-powerCurrent);
            motorRight.setPower(powerCurrent);
            //update the average position of the motors
            motorPosition = (-motorLeft.getCurrentPosition()+ motorRight.getCurrentPosition())/2;
            telemetry.addData("Accelerate/Deccelerate",powerCurrent);
        }
    }

    public void DriveFwdObstacle(double Power,double thresholdPower){
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Get average current motor speed and set it to motorEncoderSpeed Variable
        double motorEncoderSpeed= (-motorRight.getPower()+motorLeft.getPower())/2;
        //While motor hasn't sensed resistance
        while (motorEncoderSpeed<thresholdPower&&opModeIsActive()) {
            //Set power to the motors
            motorLeft.setPower(-Power);
            motorRight.setPower(Power);
            //update the average speed of the motors
            motorEncoderSpeed= (motorRight.getPower()+motorLeft.getPower())/2;
            //Telemetry for Debugging
            telemetry.addData("motorPower",motorEncoderSpeed);
            telemetry.addData("motorPowerThreshold",thresholdPower);
            telemetry.update();
        }

        while (motorEncoderSpeed>thresholdPower&&opModeIsActive()) {
            //Set power to the motors
            motorLeft.setPower(-Power);
            motorRight.setPower(Power);
            //update the average speed of the motors
            motorEncoderSpeed= (motorRight.getPower()+motorLeft.getPower())/2;
            //Telemetry for Debugging
            telemetry.addData("motorPower",motorEncoderSpeed);
            telemetry.addData("motorPowerThreshold",thresholdPower);
            telemetry.update();
            Log.i("motorPower",String.valueOf(motorEncoderSpeed));

        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void TurnGyro (float degrees, float precision, double regurlarPower, float relativeToStart) {
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Get the gyro reading from the IMU
        float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        //Adding previous value of the gyro to the target to find end position
        gyroReading = gyroReading < 0 ? 360 + gyroReading : gyroReading;
        //Allows for a relative to start position
        float totalDegrees= relativeToStart!= 0 ? degrees + relativeToStart :degrees + gyroReading;

        totalDegrees = totalDegrees % 360;
        //Log information for Debugging
        Log.i("GyroDebug", "CurrentAngle: " + gyroReading + "   target: " + totalDegrees);


        //Set motors in opposite directions according to (+ or -)degrees
        double left = 1;
        double right= 1;
        //If degrees is negative = clockwise
        if(totalDegrees-gyroReading<0){
            left =  -1;
            right=  -1;
            //If degrees is Positive = anti-clockwise
        }else if(totalDegrees-gyroReading>0){
            left = 1;
            right= 1;
        }
        //While the gyro is not greater or less than the target ...
        // Math.Abs(totalDegrees - gyroDegrees) < precision
        //while(gyroReading>totalDegees+precision || gyroReading<totalDegees-precision){
        int counter=0;
        while(Math.abs(totalDegrees - gyroReading) > precision&&opModeIsActive()) {
            //turn the motors
            motorLeft.setPower(left*regurlarPower);
            motorRight.setPower(right*regurlarPower);
            telemetry.addData("Gyro angle",(totalDegrees-gyroReading));
            telemetry.update();
            //Update Gyro position
            gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            gyroReading = gyroReading < 0 ? 360 + gyroReading:gyroReading;
            Log.i(counter++ + ". CurrentAngle",String.valueOf(gyroReading));
        }
        motorRight.setPower(0);
        motorLeft.setPower(0);
        Log.i("Final Angle",String.valueOf(gyroReading));
    }
    public void preciseTurn(double angle, double power){
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double wheelDiameterInch = 3.54;
        double axelLengthInch = 13;
        double differenceInDistance = (Math.abs(angle) * (axelLengthInch/wheelDiameterInch))*(288/360);
        int    motorLeftEncoder= motorLeft.getCurrentPosition();
        int    motorRightEncoder= motorRight.getCurrentPosition();
        double motorLeftTarget = motorLeftEncoder - differenceInDistance;
        double motorRightTarget = motorRightEncoder + differenceInDistance;
        double precision = 0.3;
        motorLeft.setTargetPosition((int)Math.round(motorLeftTarget));
        motorRight.setTargetPosition((int)Math.round(motorRightTarget));
        if(angle>=0){
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }else {
            motorLeft.setPower(-power);
            motorRight.setPower(-power);
        }


    }
    public void goldFinder(){
        //Senses Gold
        //vision.init();// enables the camera overlay. this will take a couple of seconds
        //vision.enable();// enables the tracking algorithms. this might also take a little time
        final long startTime = (long) (System.nanoTime() / Math.pow(10,9));
        long runningTime = (long) (System.nanoTime() / Math.pow(10,9));
        Log.i("System Time:startTime", String.valueOf(startTime));
        //while ((runningTime < startTime + secondsRun)&&((goldPosition!=SampleRandomizedPositions.CENTER||goldPosition!=SampleRandomizedPositions.LEFT)||goldPosition!=SampleRandomizedPositions.RIGHT)) {
            //runningTime = (long) (System.nanoTime() / Math.pow(10,9));
            //Log.i("System Time:runningTime", String.valueOf(startTime));
        //}
        //vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        //goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        if((runningTime > startTime + secondsRun)){
            Log.i("TensorFlow", "Ran Out of Time");
        }
        //Log.i("TensorFlow",String.valueOf(goldPosition));

        //telemetry.addData("goldPosition was", String.valueOf (goldPosition));// giving feedback
        telemetry.update();
    }

    public void DriveAccFwdDcc(int distanceAccDcc,int cruisingDistance, double initialAndFinalPower, double cruisingPower) throws InterruptedException {

        DriveFwdAccDcc(initialAndFinalPower, cruisingPower,distanceAccDcc);
        DriveFwdDistance(cruisingPower,cruisingDistance,true);
    }
}
