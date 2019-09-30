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
    private DcMotor motorFront;
    private DcMotor motorBack;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    //private CRServo motorLatchingAssistant;

    //Declare Servos
    private Servo TerrenceTheServo;
    private Servo Larm;
    private Servo Rarm;

    //Declare Sensors
    //DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    //DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static boolean OldTerrence =false;
    private static boolean NewTerrence =false;
    private static boolean OldLarm =false;
    private static boolean NewLarm =false;
    private static boolean OldRarm =false;
    private static boolean NewRarm =false;

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        //latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        //latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);

        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorFront = hardwareMap.dcMotor.get("motorFront");
        motorBack = hardwareMap.dcMotor.get("motorBack");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotor.Direction.REVERSE);
        motorBack.setDirection(DcMotor.Direction.FORWARD);
        //extendingArmMotor.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        Larm = hardwareMap.servo.get("Larm");
        Rarm = hardwareMap.servo.get("Rarm");

        waitForStart();

        //INSERT CODE HERE
        while(opModeIsActive()) {

            if(gamepad1.left_trigger==1){
                motorLeft.setPower(1);
                motorRight.setPower(-1);
                motorFront.setPower(-1);
                motorBack.setPower(1);
            }else if(gamepad1.right_trigger==1){
                motorLeft.setPower(-1);
                motorRight.setPower(1);
                motorFront.setPower(1);
                motorBack.setPower(-1);
            }
            motorLeft.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            motorRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorFront.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorBack.setPower(gamepad1.left_trigger-gamepad1.right_trigger);


            motorFront.setPower(-gamepad1.right_stick_x);
            motorBack.setPower(-gamepad1.right_stick_x);
            motorLeft.setPower(-gamepad1.right_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //Terrence servo control
            NewTerrence=gamepad1.a;
            if(!OldTerrence&&NewTerrence){
                if (TerrenceTheServo.getPosition()==0){
                    TerrenceTheServo.setPosition(1);
                } else {
                    TerrenceTheServo.setPosition(0);
                }
            }
            OldTerrence=NewTerrence;

            //Right And Left arm servo
            NewRarm=gamepad1.x;
            if(!OldRarm&&NewRarm){
                if (Rarm.getPosition()==1){
                    Rarm.setPosition(0);
                    Larm.setPosition(1);

                } else {
                    Rarm.setPosition(1);
                    Larm.setPosition(0);

                }
            }
            OldRarm=NewRarm;

            telemetry.update();
        }
        idle();

    }
}
