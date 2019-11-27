package org.firstinspires.ftc.teamcode.SkystoneCode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Date;

//@Disabled
@TeleOp(name= "ExperimentalSkystoneTeleOp")
public class ExperimentalSkystoneTeleOp extends LinearOpMode {

    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private CRServo linearslideRight;
    private CRServo linearslideLeft;





    //Declare Servos
    private Servo TerrenceTheServo;
    private Servo NerrenceTheServo;
    private Servo LeftServoArm;
    private Servo RightServoArm;

    //Declare Sensors
    //DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    //DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Variables
    private static boolean OldTerrence =false;
    private static boolean NewTerrence =false;
    private static boolean OldLarm =false;
    private static boolean NewLarm =false;
    private static boolean OldRarm =false;
    private static boolean NewRarm =false;
    private static double slopeForLeftJoystick = 1; //calculates slope of the line that goes through the origin and x and y value for the joystick
    private static double xLeftJoystick =0;         //xvalue for joystick
    private static double yLeftJoystick = 0;        //yvalue for joystick
    private static double referenceAngleLeftJoy=0;  //reference angle for the line^ in Quadrant 1 of unit circle
    private static double angleLeftJoy = 0;         //angle of the line^ for the joystick
    private static double robotAngle =0;            //angle of the robot
    private static double relativeRobotAngle =0;    //to reset angle of robot

    //Define Gyro
    BNO055IMU imu;
    //Time
    Date time;

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        //latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        //latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);


        //Configure motors to Expansion Hub
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        linearslideLeft = hardwareMap.crservo.get("linearslideLeft");
        linearslideRight = hardwareMap.crservo.get("linearslideRight");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

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

        //Configure Servos
       TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        LeftServoArm = hardwareMap.servo.get("Larm");
        RightServoArm = hardwareMap.servo.get("Rarm");

        waitForStart();

        //INSERT CODE HERE
        while(opModeIsActive()) {
            //Reset gyro angle if needed
            relativeRobotAngle = gamepad1.y ? imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle:relativeRobotAngle;
            //Move robot in rotations, one direction, or stop
            if((gamepad1.left_trigger+gamepad1.right_trigger>0.2)) {
                motorFrontLeft.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorBackRight.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorFrontRight.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorBackLeft.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }else if(gamepad1.left_bumper&&((Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y))>0)) {
                DriveInStraightLine(.5);
            } else if((Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0)) {
                DriveInStraightLine(1);
            } else {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            telemetry.addData("Right Motors", gamepad1.right_trigger-gamepad1.left_trigger);
            telemetry.addData("Left Motors", gamepad1.left_trigger-gamepad1.right_trigger);
            Log.d("Gamepad.1.y","Gamepady="+(gamepad1.left_stick_y));
            telemetry.addData("Gamepady=",(gamepad1.left_stick_y));
            //Mover robot in all directions in a straight line
            /*
            if(gamepad1.dpad_up){
             motorFrontLeft.setPower(.5);
             motorFrontRight.setPower(.5);
            }else if(gamepad1.dpad_down){
                motorBackLeft.setPower(.5);
                motorBackRight.setPower(.5);
            }else if(gamepad1.dpad_right){
                motorBackRight.setPower(.5);
                motorFrontRight.setPower(.5);
            }else if(gamepad1.dpad_left){
                motorBackLeft.setPower(.5);
                motorFrontLeft.setPower(.5);
            }
            */
            //Linear Slide Motors&&!latchingTouchSensorDown.getState()
            if(!(gamepad2.left_stick_y==0)) {
                linearslideLeft.setPower(gamepad2.left_bumper ? gamepad2.left_stick_y: gamepad2.left_stick_y/2);
            }else {
                linearslideLeft.setPower(0);
            }
            //Terrence servo control
            NewTerrence=gamepad2.a;
            if(!OldTerrence&&NewTerrence){
                if (TerrenceTheServo.getPosition()==0){
                    TerrenceTheServo.setPosition(1);
                    NerrenceTheServo.setPosition(1);
                } else {
                    TerrenceTheServo.setPosition(0);
                    NerrenceTheServo.setPosition(0);
                }
            }
            OldTerrence=NewTerrence;

            //Right And Left arm servo
            NewRarm=gamepad1.x;
            if(!OldRarm&&NewRarm){
                if (RightServoArm.getPosition()==1){
                    RightServoArm.setPosition(0);
                    LeftServoArm.setPosition(1);
                } else {
                    RightServoArm.setPosition(1);
                    LeftServoArm.setPosition(0);
                }
            }
            OldRarm=NewRarm;

            telemetry.update();
            Thread.sleep(100);
        }



        idle();

    }
    public void DriveInStraightLine(double powerMultiplier) throws InterruptedException {
        double gamepadx = gamepad1.left_stick_x;        //holds value of sensor
        double gamepady = gamepad1.left_stick_y;        //holds value of sensor
        double gamepadtrigger = 1-gamepad1.left_trigger;  //holds value of sensor

        //Calculations for the angle of the joystick
        slopeForLeftJoystick = gamepady/gamepadx;                                  //Calculates slope using m=y/x formula
        xLeftJoystick=Math.sqrt(1/(Math.pow(slopeForLeftJoystick,2)+1));           //Calculated the x value of the intersection between x^2 +y^2 =1 and y=mx
        yLeftJoystick=slopeForLeftJoystick*xLeftJoystick;                          //Calculated the y value from y=mx where x is from above
        xLeftJoystick=gamepadx<0 ? -xLeftJoystick:xLeftJoystick;                   //Negates x if x is negative on joystick
        yLeftJoystick=gamepady<0 ? -yLeftJoystick:yLeftJoystick;                   //Negates y if y is negative on joystick
        referenceAngleLeftJoy=(Math.abs(Math.acos(xLeftJoystick))+Math.abs(Math.asin(yLeftJoystick)))/2;
        angleLeftJoy= (xLeftJoystick<0&&yLeftJoystick>0)? Math.PI -referenceAngleLeftJoy:       //if in quadrant 2, make angle = π-reference angle
                ((xLeftJoystick<0&&yLeftJoystick<0)? Math.PI +referenceAngleLeftJoy:            //if in quadrant 3, make angle = π+reference angle
                        (((xLeftJoystick<0&&yLeftJoystick<0)? 2*Math.PI -referenceAngleLeftJoy: //if in quadrant 4, make angle = 2π-reference angle
                                referenceAngleLeftJoy)));                                       //if in quadrant 1,, make angle = reference angle

        //Calculations to find the desired angle relative to the robot
        double currentRobotAngle =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        robotAngle= angleLeftJoy-(currentRobotAngle-relativeRobotAngle);                        //Calculates the desired angle of movement relative to the robot

        double xRobot = Math.cos(robotAngle);                                                   //Calculates the necessary movement in the x
        double yRobot = Math.sin(robotAngle);                                                   //Calculates the necessary movement in the y

        //Following lines scale the powers to get the same proportion in the x and y to achieving maximum speed
        if(yRobot>xRobot){
            yRobot=1;
            xRobot=(1/yRobot)*(xRobot);
        }else{
            xRobot=1;
            yRobot=(1/xRobot)*(yRobot);
        }

        //set powers to the motors
        motorFrontLeft.setPower((-yRobot+xRobot)*powerMultiplier*gamepadtrigger);
        motorFrontRight.setPower((yRobot+xRobot)*powerMultiplier*gamepadtrigger);
        motorBackLeft.setPower((-yRobot-xRobot)*powerMultiplier*gamepadtrigger);
        motorBackRight.setPower((yRobot-xRobot)*powerMultiplier*gamepadtrigger);
        telemetry.update();
    }
}
