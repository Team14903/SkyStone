package org.firstinspires.ftc.teamcode.SkystoneCode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name= "TeleOp Left Program")
public class SkystoneTeleOpLeftStick extends LinearOpMode {

    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private AnalogInput linearslideEncoder;
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
    private static boolean OldDpadUp =false;
    private static boolean NewDpadUp =false;
    private static boolean OldDpadDown =false;
    private static boolean NewDpadDown =false;
    private static double linearSlidePower = 50; //Power for linear slide motors

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

        //Value positions for servos
        final double armRetractedPosition = 0.0;
        final double armExtendedPosition = 0.2;

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        //linearslideEncoder = hardwareMap.analogInput.get("linearSlideEncoder");


        //Configure Servos
       TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        LeftServoArm = hardwareMap.servo.get("Larm");
        RightServoArm = hardwareMap.servo.get("Rarm");

        waitForStart();

        //INSERT CODE HERE
        while(opModeIsActive()) {
            //Spin Robot Left or Right
            if((gamepad1.left_trigger+gamepad1.right_trigger>0.2)) {
                motorFrontLeft.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorBackRight.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorFrontRight.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
                motorBackLeft.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            }else if(gamepad1.left_bumper&&((Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y))>0)) {
                motorFrontLeft.setPower((-gamepad1.left_stick_y+gamepad1.left_stick_x)/2);
                motorFrontRight.setPower((gamepad1.left_stick_y+gamepad1.left_stick_x)/2);
                motorBackLeft.setPower((-gamepad1.left_stick_y-gamepad1.left_stick_x)/2);
                motorBackRight.setPower((gamepad1.left_stick_y-gamepad1.left_stick_x)/2);
            } else if((Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.left_stick_y)>0)) {
                motorFrontLeft.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x);
                motorFrontRight.setPower(gamepad1.left_stick_y+gamepad1.left_stick_x);
                motorBackLeft.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x);
                motorBackRight.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);
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
            //Log.d("DEBUGGER: ", "Linear Slide Encoder"+linearslideEncoder.getVoltage());

            //Terrence servo control
            NewTerrence=gamepad1.a;
            if(!OldTerrence&&NewTerrence){
                if (TerrenceTheServo.getPosition()==0){
                    TerrenceTheServo.setPosition(1);
                    NerrenceTheServo.setPosition(0);
                } else {
                    TerrenceTheServo.setPosition(0);
                    NerrenceTheServo.setPosition(1);
                }
            }
            OldTerrence=NewTerrence;

            //Right And Left arm servo
            NewRarm=gamepad2.x;
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
            NewDpadDown = gamepad2.dpad_down;
            NewDpadUp = gamepad2.dpad_up;
            if(NewDpadUp && !OldDpadUp) {
            }

            telemetry.update();
            Thread.sleep(100);
        }



        idle();

    }
}
