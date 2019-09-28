package org.firstinspires.ftc.teamcode.SkystoneCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name= "TeleOp Basic Program")
public class SkystoneTeleOp extends LinearOpMode {

    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorLatching;
    private DcMotor rotatingArmMotor;
    private CRServo extendingArmMotor;
    private CRServo motorLatchingAssistant;

    //Declare Servos
    private Servo armServo;

    //Declare Sensors
    DigitalChannel latchingTouchSensorDown;//Sensor to to test if motor has reached lower limit
    DigitalChannel latchingTouchSensorUp; //Sensor to test if motor has reached upper limit

    //Value positions for servos
    private static final double armRetractedPosition = 0.0;
    private static final double armExtendedPosition = 0.2;

    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double linearSlidePower = 50; //Power for linear slide motors
    private static double turningPower; //Power difference between drive motors
    private static double xCordForCheese; //Cheesy x coordinates for game pad 1 right stick
    private static double yCordForCheese; //Cheesy y coordinates for game pad 1 right stick
    private static double theataAngleDegrees; // Angle in degrees for Cheesy control system

    @Override
    public void runOpMode() throws InterruptedException{

        //Configure Sensors
        latchingTouchSensorDown = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorDown");
        latchingTouchSensorUp = hardwareMap.get(DigitalChannel.class, "latchingTouchSensorUp");
        latchingTouchSensorUp.setMode(DigitalChannel.Mode.INPUT);
        latchingTouchSensorDown.setMode(DigitalChannel.Mode.INPUT);

        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLatching = hardwareMap.dcMotor.get("motorLatching");
        rotatingArmMotor = hardwareMap.dcMotor.get("rotatingArmMotor");
        extendingArmMotor = hardwareMap.crservo.get("extendingArmMotor");
        motorLatchingAssistant = hardwareMap.crservo.get("motorLatchingAssistant");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLatching.setDirection(DcMotor.Direction.FORWARD);
        rotatingArmMotor.setDirection(DcMotor.Direction.FORWARD);
        //extendingArmMotor.setDirection(DcMotor.Direction.FORWARD);

        //Configure Servos
        armServo = hardwareMap.servo.get("motorArm");

        //INSERT CODE HERE
            telemetry.update();
            idle();
    }
}
