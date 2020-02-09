
    private DcMotor motorLatching;
    private DcMotor rotatingArmMotor;
    private CRServo extendingArmMotor;
package org.firstinspires.ftc.teamcode.RoverRuckusCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

    @Disabled
    @TeleOp(name= "TeleOp Basic Program")
    public class RoverRuckusTeleOp extends LinearOpMode {

        //Declare motors
        private DcMotor motorLeft;
        private DcMotor motorRight;


    //Variables
    private static double driveMotorPower; // Power for drive motors
    private static double turningPower; //Power difference between drive motors
    private static double xCordForCheese; //Cheesy x coordinates for game pad 1 right stick
    private static double yCordForCheese; //Cheesy y coordinates for game pad 1 right stick
    private static double theataAngleDegrees; // Angle in degrees for Cheesy control system

    @Override
    public void runOpMode() throws InterruptedException{


        //Configure motors to Expansion Hub
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        //Set drive motors to opposite directions(is reversable if needed) and set latching motor to forward
        //Update 10.1.18: Setting right motor direction to reverse to enable 1 joystick driving
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

  int gameMode = 0;
        String[] listOfGameModes = {"1 Joystick Arcade", "2 Joystick Arcade", "Tank Drive"};
        /*boolean oldUpDPadValue = false;
        boolean oldDownDPadValue = false;

        while(!gamepad1.dpad_right||!gamepad1.dpad_left){
            boolean newUpDPadValue = gamepad1.dpad_up;
            boolean newDownDPadValue = gamepad1.dpad_down;
            if(newUpDPadValue&&!oldUpDPadValue&&!(gameMode==4)){
                gameMode = gameMode+1;
            }else if(newDownDPadValue&&!oldDownDPadValue&&!(gameMode==1)){
                gameMode = gameMode-1;
            }
            oldDownDPadValue=newDownDPadValue;
            oldUpDPadValue=newUpDPadValue;
            telemetry.addData("Controller 1 Game Mode is: ",listOfGameModes[gameMode]);
            telemetry.update();
        }*/
        //Waits for person to press Play button
        waitForStart();


        while(opModeIsActive()) {
            if (gameMode == 0) {
                //Arcade style with one joystick(simple y is power,x is the differential of the x and y values)
                // Motor goes forward with joystick and turns with same joystick
                driveMotorPower = -gamepad1.right_stick_y;
                turningPower = -gamepad1.right_stick_x;
                if(!gamepad1.x) {
                    motorLeft.setPower(Range.clip((driveMotorPower + turningPower), -1, 1));
                    motorRight.setPower(Range.clip(-(driveMotorPower - turningPower), -1, 1));
                } else {
                    motorLeft.setPower(Range.clip(-(driveMotorPower - turningPower), -1, 1));
                    motorRight.setPower(Range.clip((driveMotorPower + turningPower), -1, 1));
                }

            }else if(gameMode == 1) {
                // Arcade style with 2 joysticks(simple s(simple y is power,x is the differential of
                // the x and y values)
                // Motor goes forward with joystick and turns with different joysticks
                driveMotorPower = -gamepad1.right_stick_y;
                turningPower = -gamepad1.left_stick_x;
                motorLeft.setPower(Range.clip(-(driveMotorPower - turningPower), -1, 1));
                motorRight.setPower(Range.clip((driveMotorPower + turningPower), -1, 1));
                telemetry.addData("MotorRight",motorRight.getCurrentPosition());
                telemetry.addData("MotorLeft",motorLeft.getCurrentPosition());

            }else if(gameMode == 2) {
                //y value is set to negative in the stick control to make up mean positive
                //Motor goes forward as joystick is pushed forward in a tank drive situation
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);
                telemetry.addData("MotorRight",motorRight.getCurrentPosition());
                telemetry.addData("MotorLeft",motorLeft.getCurrentPosition());

            }
            //Button for retracted position
            //If a button is pressed and lower limit isn't reached then ...
            //Set motor to negative linear slide power
            if(gamepad1.b && latchingTouchSensorDown.getState()){
                motorLatching.setPower(-linearSlidePower);
            }
            if((latchingTouchSensorDown.getState() || latchingTouchSensorUp.getState())||(!gamepad1.a&&!gamepad1.b)){
                motorLatching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLatching.setPower(0);
            }
            if(gamepad1.left_bumper) {
                armServo.setPosition(0);
            } else if(gamepad1.right_bumper){
                armServo.setPosition(1);
            }
    }
}