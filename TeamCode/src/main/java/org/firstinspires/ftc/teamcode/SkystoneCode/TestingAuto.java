package org.firstinspires.ftc.teamcode.SkystoneCode;


import android.hardware.Sensor;
import android.hardware.SensorDirectChannel;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous
//@Disabled
public class TestingAuto extends LinearOpMode {
    private double version = 2.3;

    private String Picture;

    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    //Declare Servos
    private Servo armServo;


    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit
    private Servo TerrenceTheServo;
    private Servo NerrenceTheServo;

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double linearSlidePower = 1; //Power for linear slide motors
    private int secondsRun = 7;
    private double accelerationRobot;
    private double xLeftStick;
    private double yLeftStick;
    private double DistanceRobot;

    //Define Gyro
    BNO055IMU imu;
    /*//Vuforia variables
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;
    public static final String VUFORIA_KEY =
            "AecVx1T/////AAABmcI08yL6hk3Nt6hSJ1mD2XJhGX9s+cETNcXvwYRsviVOyFrvCxQW1uKq6eKCuXJcp6loA3K4kq5vdzjlG5oDest+XgLnl7uUO2EHXUsl63893eKJ8anaiFk1+ECVV8+Nx2UJWNLk1enRgDkMzhA38qnIfcw+D5ll48etLKc1aWkOS40ZHoIscDctrsSWxFI6O8fDMHF8M4OrqYUd1kHPOj+5hNA6C2+BKn72JY53ABrveqy2gYtyBTkBQsZbGU7tKnJb4xvNcqk2q1IINiKx/lkoPUuJWbtG7xMJta/DySf+xjcggD63vooQ6Lz9HYMNml+IhogXQQwgGwssPukwUg+snYX17rq4fHNPw26aLN8J";
    public static final float mmPerInch = 25.4f;                    //Conversion factor
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    public static final float stoneZ = 2.00f * mmPerInch;                  //size of stone in z
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public boolean targetVisible = false;
    public float phoneXRotate = 0;
    public float phoneYRotate = 0;
    public float phoneZRotate = 90;*/

    public void runOpMode() throws InterruptedException {

        TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");


        //Configure motors to Expansion Hub
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        //Configure Servos
        //armServo = hardwareMap.servo.get("motorArm");

        // Initialize Gyro
        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersGyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.loggingEnabled = false;
        parametersGyro.mode = BNO055IMU.SensorMode.IMU;
        parametersGyro.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu name");
        //Todo: find if initializing the code will be more accurate here or right before the code is begun
        imu.initialize(parametersGyro);
        float firstGyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        //vision init
        telemetry.addData("Version:", version);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(!opModeIsActive()&&!isStopRequested()){
            telemetry.addData("status","waiting for start command...");
            telemetry.update();
        }
        //targetsSkyStone.activate();

        while (opModeIsActive()) {
            DriveInStraightLine(1, 100, 70);
            TerrenceTheServo.setPosition(1);
            NerrenceTheServo.setPosition(1);
            DriveInStraightLine(-1, 100, 0);
            TerrenceTheServo.setPosition(0);
            NerrenceTheServo.setPosition(0);
            DriveInStraightLine(1, 100, 270);


            Thread.sleep(500);
            telemetry.addData("Right Wheel Encoder", 0);
            telemetry.update();
            requestOpModeStop();
        }
    }

    // This is a method to make code easier to read, see above
    public void DriveInStraightLine(double Power, double distance, double angle) throws InterruptedException {
        yLeftStick = Math.sin(angle);
        DistanceRobot = 0;
        long overallTime;
        long initialTime = System.currentTimeMillis();
        distance += (Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition())+Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition()))/4;
        while (DistanceRobot < distance && !isStopRequested()) {
            motorFrontLeft.setPower(Range.clip(-yLeftStick + xLeftStick, -1, 1) * Power);
            motorFrontRight.setPower(Range.clip(yLeftStick + xLeftStick, -1, 1) * Power);
            motorBackLeft.setPower(Range.clip(-yLeftStick - xLeftStick, -1, 1) * Power);
            motorBackRight.setPower(Range.clip(yLeftStick - xLeftStick, -1, 1) * Power);
            DistanceRobot = (Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition())+Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition()))/4;
            telemetry.addData("Robot Distance", DistanceRobot);
            telemetry.addData("Encoders",(Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition())+Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition()))/4);

            telemetry.update();

        }
        telemetry.update();
        //Stops motors

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void TurnGyro(float degrees, float precision, double regurlarPower, float relativeToStart) {
        //Get the gyro reading from the IMU
        float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        //Adding previous value of the gyro to the target to find end position
        gyroReading = gyroReading < 0 ? 360 + gyroReading : gyroReading;
        //Allows for a relative to start position
        float totalDegrees = relativeToStart != 0 ? degrees + relativeToStart : degrees + gyroReading;

        totalDegrees = totalDegrees % 360;
        //Log information for Debugging
        Log.i("GyroDebug", "CurrentAngle: " + gyroReading + "   target: " + totalDegrees);


        //Set motors in opposite directions according to (+ or -)degrees
        double left = 1;
        double right = 1;
        //If degrees is negative = clockwise
        if (totalDegrees - gyroReading < 0) {
            left = -1;
            right = -1;
            //If degrees is Positive = anti-clockwise
        } else if (totalDegrees - gyroReading > 0) {
            left = 1;
            right = 1;
        }
        //While the gyro is not greater or less than the target ...
        // Math.Abs(totalDegrees - gyroDegrees) < precision
        //while(gyroReading>totalDegees+precision || gyroReading<totalDegees-precision){
        int counter = 0;
        while (Math.abs(totalDegrees - gyroReading) > precision && opModeIsActive()) {
            //turn the motors
            motorBackLeft.setPower(left * regurlarPower);
            motorFrontLeft.setPower(left * regurlarPower);
            motorBackRight.setPower(right * regurlarPower);
            motorFrontRight.setPower(right * regurlarPower);
            telemetry.addData("Gyro angle", (totalDegrees - gyroReading));
            telemetry.update();
            //Update Gyro position
            gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            gyroReading = gyroReading < 0 ? 360 + gyroReading : gyroReading;
            Log.i(counter++ + ". CurrentAngle", String.valueOf(gyroReading));
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        Log.i("Final Angle", String.valueOf(gyroReading));
    }
}