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
    private DcMotor linearslideRight;
    private DcMotor linearslideLeft;
    private CRServo motorLatchingAssistant;
    private CRServo extendingArmMotor;
    Float LFspeed;
    Float RFspeed;
    Float LBspeed;
    Float RBspeed;



    //Declare Servos
    private Servo TerrenceTheServo;
    private Servo NerrenceTheServo;
    private Servo Larm;
    private Servo Rarm;

    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static boolean OldTerrence =false;
    private static boolean NewTerrence =false;
    private static boolean OldLarm =false;
    private static boolean NewLarm =false;
    private static boolean OldRarm =false;
    private static boolean NewRarm =false;
    private static double xPosition;
    private static double yPosition;
    private static double linearSlidePower = 50; //Power for linear slide motors

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);


        //Configure motors to Expansion Hub
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        linearslideLeft = hardwareMap.dcMotor.get("linearslideLeft");
        linearslideRight = hardwareMap.dcMotor.get("linearslideRight");

        //Value positions for servos
        final double armRetractedPosition = 0.0;
        final double armExtendedPosition = 0.2;

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        //extendingArmMotor.setDirection(DcMotor.Direction.FORWARD);

        //configuring Linear Slide Motors
        extendingArmMotor = hardwareMap.crservo.get("extendingArmMotor");


        //Configure Servos
       TerrenceTheServo = hardwareMap.servo.get("TerrenceTheServo");
        NerrenceTheServo = hardwareMap.servo.get("NerrenceTheServo");
        Larm = hardwareMap.servo.get("Larm");
        Rarm = hardwareMap.servo.get("Rarm");

        waitForStart();

        //INSERT CODE HERE
        while(opModeIsActive()) {
            //Spin Robot Left or Right
            motorFrontLeft.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            motorBackRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorFrontRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            motorBackLeft.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            //Linear Slide Motors
            linearslideLeft.setPower(gamepad2.left_stick_y);
            linearslideRight.setPower(-gamepad2.left_stick_y);




            xPosition = gamepad1.right_stick_x;
            yPosition = gamepad1.right_stick_y;

            motorFrontLeft.setPower(-gamepad1.right_stick_y-gamepad1.right_stick_x);
            motorFrontRight.setPower(-gamepad1.right_stick_y+gamepad1.right_stick_x);
            motorBackLeft.setPower(-gamepad1.right_stick_y-gamepad1.right_stick_x);
            motorBackRight.setPower(-gamepad1.right_stick_y+gamepad1.right_stick_x);


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
                if (Rarm.getPosition()==1){
                    Rarm.setPosition(0);
                    Larm.setPosition(1);

                } else {
                    Rarm.setPosition(1);
                    Larm.setPosition(0);

                }
            }
            OldRarm=NewRarm;

            if(gamepad2.left_bumper){
                extendingArmMotor.setPower(-linearSlidePower);

            } else if(gamepad2.right_bumper){
                extendingArmMotor.setPower(linearSlidePower);
            } else {
                extendingArmMotor.setPower(0);
            }
            if(gamepad1.dpad_up){
                motorLatchingAssistant.setPower(linearSlidePower);

            } else if(gamepad1.dpad_down){
                motorLatchingAssistant.setPower(-linearSlidePower);

            } else {
                motorLatchingAssistant.setPower(0);
            }

            telemetry.update();
        }
        LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        RFspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        LBspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;
        (Range.clipgamepad1.left_stick_y - gamepad1.left_stick_x;
        (Range.clip gamepad1.left_stick_y + gamepad1.left_stick_x;
        (Range.clipgamepad1.right_stick_y + gamepad1.left_stick_x;
        (Range.clipgamepad1.right_stick_y - gamepad1.left_stick_x;
        //driveMotorPower = -gamepad1.right_stick_y;

            //LFspeed.setPower(Range.clip((driveMotorPower + turningPower), -1, 1));
            //RFspeed.setPower(Range.clip(-(driveMotorPower - turningPower), -1, 1));
            //LBspeed.setPower(Range.clip((driveMotorPower + turningPower), -1, 1));
            //RBspeed.setPower(Range.clip(-(driveMotorPower - turningPower), -1, 1));
        //} else {
            //LFspeed.setPower(Range.clip((driveMotorPower - turningPower), -1, 1));
            //RFspeed.setPower(Range.clip(-(driveMotorPower + turningPower), -1, 1));
            //LBspeed.setPower(Range.clip((driveMotorPower - turningPower), -1, 1));
            //RBspeed.setPower(Range.clip(-(driveMotorPower + turningPower), -1, 1));
        //}

        idle();

    }
}
