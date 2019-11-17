package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name= "TeleOp Basic Program")
public class SkystoneTeleOp extends LinearOpMode {

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
        linearslideRight = hardwareMap.crservo.get("linearslideRight");

        //Value positions for servos
        final double armRetractedPosition = 0.0;
        final double armExtendedPosition = 0.2;

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);


        //Configure Servos
       TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        LeftServoArm = hardwareMap.servo.get("Larm");
        RightServoArm = hardwareMap.servo.get("Rarm");

        waitForStart();

        //INSERT CODE HERE
        while(opModeIsActive()) {
            //Spin Robot Left or Right
            motorFrontLeft.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            motorBackRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorFrontRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorBackLeft.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            //Mover robot in all directions in a straight line
            motorFrontLeft.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x,-1, 1));
            motorFrontRight.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x,-1, 1));
            motorBackLeft.setPower(Range.clip(gamepad1.right_stick_y + gamepad1.left_stick_x,-1, 1));
            motorBackRight.setPower(Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -1, 1));

            //Linear Slide Motors&&!latchingTouchSensorDown.getState()
            if(gamepad2.left_stick_y<0) {
                linearslideLeft.setPower(gamepad2.left_bumper ? gamepad2.left_stick_y: gamepad1.left_stick_y/4);
            }else if (gamepad2.left_stick_y>0) {
                linearslideLeft.setPower(gamepad2.left_bumper ? gamepad2.left_stick_y: gamepad1.left_stick_y/4);
            }else {
                linearslideLeft.setPower(0);
            }
            //Terrence servo control
            NewTerrence=gamepad1.a;
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
        }



        idle();

    }
}
