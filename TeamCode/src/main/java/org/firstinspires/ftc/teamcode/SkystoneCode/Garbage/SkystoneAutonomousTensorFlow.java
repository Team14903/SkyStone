package org.firstinspires.ftc.teamcode.SkystoneCode.Garbage;

import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.android.dex.SizeOf;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.ListIntSet;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name= "AutonomousSkystoneTensorFlow")
@Disabled
public class SkystoneAutonomousTensorFlow extends LinearOpMode {
    private double version = 3.0;

    private String Picture;

    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DigitalChannel linearSlideEncoder;

    //Declare Servos
    private Servo armServo;
    private Servo PullDownArmServo;


    //Declare Sensors
    private Servo TerrenceTheServo;
    private Servo NerrenceTheServo;
    private Servo LeftServoArm;
    private Servo RightServoArm;

    //Variables
    private double xLeftStick;
    private double yLeftStick;
    private double DistanceRobot;
    private double translations;
    private float BlockSkystone=0;
    private float BlockStone;
    private int Blocks;
    //Define Gyro
    BNO055IMU imu;
    //Vuforia variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final String VUFORIA_KEY =
            "AecVx1T/////AAABmcI08yL6hk3Nt6hSJ1mD2XJhGX9s+cETNcXvwYRsviVOyFrvCxQW1uKq6eKCuXJcp6loA3K4kq5vdzjlG5oDest+XgLnl7uUO2EHXUsl63893eKJ8anaiFk1+ECVV8+Nx2UJWNLk1enRgDkMzhA38qnIfcw+D5ll48etLKc1aWkOS40ZHoIscDctrsSWxFI6O8fDMHF8M4OrqYUd1kHPOj+5hNA6C2+BKn72JY53ABrveqy2gYtyBTkBQsZbGU7tKnJb4xvNcqk2q1IINiKx/lkoPUuJWbtG7xMJta/DySf+xjcggD63vooQ6Lz9HYMNml+IhogXQQwgGwssPukwUg+snYX17rq4fHNPw26aLN8J";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    public void runOpMode() throws InterruptedException {
        //Vuforia initialization
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }



        //Configure Sensors
        //latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        //latchingTouchSensorUp = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorUp");
        //latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);
        //latchingTouchSensorDown.setMode(DigitalChannel.Mode.INPUT);
        TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        LeftServoArm = hardwareMap.servo.get("Larm");
        RightServoArm = hardwareMap.servo.get("Rarm");
        //PullDownArmServo = hardwareMap.servo.get("PullDownArmServo");

        //Configure motors to Expansion Hub
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
        //while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Wait for Gyro Calibration", 0);
            telemetry.update();
            sleep(50);
            //idle();
        //}
        //vision init
        if (tfod != null) {
            tfod.activate();
        }

        /*while (!isStopRequested() && tfod!=null) {
            telemetry.addData("Wait for vision calibration", 0);
            telemetry.update();
            sleep(50);
            idle();
        }*/
        telemetry.addData("Version:", version);
        telemetry.addData("Ready for start", 0);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {

            DriveInFWD(-1,-450);
            TensorFlowDetection(5000);
            if(BlockSkystone==1){
                DriveInFWD(1,430);
                grabOrReleaseBlocks(true);
                DriveInFWD(-1,100);
                DriveSideways(1,100,0,0);
                grabOrReleaseBlocks(false);
                DriveSideways(-1,-100,0,0);
                DriveInFWD(1,100);
                grabOrReleaseBlocks(true);
                DriveInFWD(-1,100);
                DriveSideways(1,100,0,0);
                grabOrReleaseBlocks(false);
                DriveSideways(-1,100,0,0);
            }
            //int blockPositon = (BlockSkystone==0) ? 3 :
                    //((double)BlockSkystone<(double)BlockStone) ? 2 : 3;
            //Log.d("DEBUGGER", "Vuforia:Position" + translations);
            //telemetry.addData("TensorFlowPosition:",blockPositon);
            /*situationBlock(blockPositon);
            PullDownArmServo.setPosition(1);
            DriveInFWD(1, -440);
            DriveSideways(1, -1320, 0, 0);
            LeftServoArm.setPosition(0);
            RightServoArm.setPosition(0);
            DriveSideways(1, 1760, 0, 0);
            DriveInFWD(1, 440);
            LeftServoArm.setPosition(1);
            RightServoArm.setPosition(1);
            DriveInFWD(1, -440);
            DriveSideways(1, -1320, 0, 0);
            LeftServoArm.setPosition(0);
            RightServoArm.setPosition(0);
            DriveSideways(1, 220, 0, 0);
            */

            telemetry.update();
            sleep(5000);
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

    *        telemetry.update();

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

    public void DriveSideways(double Power, double distance, int slowDownDistance, double slowDownPower) throws InterruptedException {
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
        gyroReading = gyroReading < 0 ? (float) (2 * Math.PI) + gyroReading : gyroReading;
        //Allows for a relative to start position
        float totalDegrees = relativeToStart != 0 ? degrees / 180 * (float) (Math.PI) + relativeToStart : degrees / 180 * (float) (Math.PI) + gyroReading;

        totalDegrees = totalDegrees % (float) (2 * Math.PI);
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
        while (Math.abs(totalDegrees - gyroReading) > precision * (float) (Math.PI) / 180 && opModeIsActive()) {
            //turn the motors
            motorBackLeft.setPower(left * regurlarPower);
            motorFrontLeft.setPower(left * regurlarPower);
            motorBackRight.setPower(right * regurlarPower);
            motorFrontRight.setPower(right * regurlarPower);
            telemetry.addData("Gyro angle", (totalDegrees - gyroReading));
            telemetry.update();
            //Update Gyro position
            gyroReading = imu.getAngularOrientation().firstAngle;
            gyroReading = gyroReading < 0 ? (float) (2 * Math.PI) + gyroReading : gyroReading;
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

    public void resetPosition() {
        telemetry.addData("Currently: reseting encoders", null);
        telemetry.update();
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Done: reseting encoders", null);
        telemetry.update();
    }

    public void situationBlock(int x) throws InterruptedException {
        if (x == 1) {
            DriveInFWD(1, 880);
            LeftServoArm.setPosition(1);
            RightServoArm.setPosition(1);


        } else if (x == 2) {
            DriveSideways(1, 146, 0, 0);
            DriveInFWD(1, 880);

            LeftServoArm.setPosition(1);
            RightServoArm.setPosition(1);
        } else {
            DriveSideways(1, 292, 0, 0);
            DriveInFWD(1, 880);

            LeftServoArm.setPosition(1);
            RightServoArm.setPosition(1);

        }
    }

    public void grabOrReleaseFoundation(boolean grab) {
        if (grab) {
            TerrenceTheServo.setPosition(0);
            NerrenceTheServo.setPosition(1);
        } else {
            TerrenceTheServo.setPosition(1);
            NerrenceTheServo.setPosition(0);

        }
    }
    public void grabOrReleaseBlocks(boolean grab) {
        if (grab) {
            LeftServoArm.setPosition(0);
            RightServoArm.setPosition(1);
        } else {
            LeftServoArm.setPosition(1);
            RightServoArm.setPosition(0);

        }
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void TensorFlowDetection(int timeLimit){
        double time = System.currentTimeMillis()+ timeLimit;
        Blocks=0;
        while (!isStopRequested()&& Blocks>1&&System.currentTimeMillis()<time) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                Blocks =updatedRecognitions.size();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if(recognition.getLabel()=="Skystone"){BlockSkystone=1;}
                    //if(recognition.getLabel()=="Stone"){BlockStone= recognition.getRight();}
                }telemetry.update();
            }
        }
    }
}
