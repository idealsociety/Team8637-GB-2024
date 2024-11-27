package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;






@TeleOp
public class GoBilda_Drive_Base_SINGLEPLAYER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DigitalChannel ExtendLimit;
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        
        DcMotor motorLiftLeft = hardwareMap.dcMotor.get("motorLiftLeft");
        DcMotor motorLiftRight = hardwareMap.dcMotor.get("motorLiftRight");
        
        DcMotor motorExtendLeft = hardwareMap.dcMotor.get("motorExtendLeft");
        DcMotor motorExtendRight = hardwareMap.dcMotor.get("motorExtendRight");
        
        DigitalChannel LiftDown = hardwareMap.get(DigitalChannel.class,"LiftHome");
        
        ColorSensor sensorColor;
        DistanceSensor sensorDistance;
        
        Servo flipper = hardwareMap.servo.get("flipper");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        
        //Color and Distance Sensor are one in the same!
        sensorColor = hardwareMap.get(ColorSensor.class, "ColorSense");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "ColorSense");
        
        //hsv Values is an array that will hold the hue, saturation and value information!
        float hsvValues[] = {0F, 0F, 0F};
        
        //values is a reference to the hsv values array!
        final float values [] = hsvValues;
        
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        
         // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtendLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtendRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtendRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Lift motor reversed for correct encoder counts!
        motorLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtendRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        
        
        //telemetry.addData("Status", "he ready");
        //telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        
        
        //telemetry.
        //telemetry.update();
        
        


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // This is currently inverted!!
            double x = -gamepad1.left_stick_x * 10; // Counteract imperfect strafing this is front to back weight bias
            double rx = gamepad1.right_stick_x * 8; // Invert this if needed!!

            ///////////////////////// Color Detection Control Section ////////////////////////////

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);


            ///////////////////////// Overdrive Control Section ////////////////////////////


            float Overdrive; // makes robot go fast!!!
            if (gamepad1.right_bumper == false) {
                Overdrive = 0.35F;
            } else {
                Overdrive = 1F;
            }
            // ^^^ Gamepad 1

            ///////////////////////// Lifter Section ////////////////////////////
            
            float Lifta; // lift up
            if (gamepad1.dpad_up == true && motorLiftLeft.getCurrentPosition() <= 2600) 
            {
                Lifta = 1F; // Goes up!
            } 
            else if (gamepad1.dpad_down == true && (LiftDown.getState() == false))
            {
                Lifta = -1F; //Goes down until limit switch met!
            }
            else if (gamepad1.dpad_down == false && gamepad1.dpad_up == false && LiftDown.getState()==true)
            {
                Lifta = 0F; // Stops holding power if arm is down!
            }
            else
            {
                Lifta = 0.001F; //Holds in place if arm is up!
            }
            
            
            ///////////////////////// Extender Section ////////////////////////////
            
            float Extenda; // Extends the arm!
            if (gamepad1.dpad_right == true && (motorExtendLeft.getCurrentPosition() <= 1400)) 
            {
                Extenda = 1F; // Goes up!
            } 
            else if (gamepad1.dpad_left == true && motorExtendLeft.getCurrentPosition() >= 5) 
            {
                Extenda = -1F; //goes down
            }
            else if (gamepad1.dpad_right == false && gamepad1.dpad_left == false && LiftDown.getState() == true)
            {
                Extenda = 0F; //No need to hold when arm is down
            }
            else 
            {
                Extenda = 0.001F; // Stops and holds in place!
            }
            
            ///////////////////////// Flipper Section ////////////////////////////


            float flippera = 0;
            if (gamepad1.a) {  // flips the claw down larger value = towards floor default = 0.81
                flipper.setPosition(0.42);
            } if (gamepad1.y) {  // flips the claw up lower value = towards robot default = 0.50
                flipper.setPosition(0);
            }

            ///////////////////////// Claw Section ////////////////////////////


            float clawa = 0;
            if (gamepad1.right_trigger > 0) {  // closes the claw
                claw.setPosition(0);
            } if (gamepad1.left_trigger > 0) {  // opens the claw
                claw.setPosition(0.3);
            }


            ///////////////////////// Wrist Section ////////////////////////////

            float wrista = 0;
            if (gamepad1.b) { //rotates the wrist to face the front
                wrist.setPosition(-0.10);
            } if (gamepad1.x) { //rotates the wrist to face the rear
                wrist.setPosition(0.625);
            }

            ///////////////////////// Servo Variable Output Section ////////////////////////////


            double k;
            {
                k = claw.getPosition(); 
            }
            
            double k2;
            {
                k2 = flipper.getPosition();
            }

            double k3;
            {
                k3 = wrist.getPosition();
            }

            ///////////////////////// Display on Station Section ////////////////////////////

            //feedback data below for on driver station!
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Claw", k);
            telemetry.addData("Flipper", k2);
            telemetry.addData("Lift Command", Lifta);
            telemetry.addData("Lifter Left", motorLiftLeft.getCurrentPosition());
            telemetry.addData("Lifter Right", motorLiftRight.getCurrentPosition());
            telemetry.addData("Extender Left", motorExtendLeft.getCurrentPosition());
            telemetry.addData("Extender Right", motorExtendRight.getCurrentPosition());
            telemetry.addData("Lift Down", LiftDown.getState());
            
             // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            
            telemetry.update();

            ///////////////////////// DC motor driving Section ////////////////////////////


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator;
            {
                denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
            }
            double frontLeftPower = ((y + x + rx) * Overdrive) / denominator;
            double RearLeftPower = ((y - x + rx) * Overdrive) / denominator;
            double frontRightPower = ((y - x - rx) * Overdrive) / denominator;
            double RearRightPower = ((y + x - rx) * Overdrive) / denominator;
            double LiftMotorPower = Lifta;
            double ExtendMotorPower = Extenda;
            
            ///////////////////////// DC Motor Power Set Section ////////////////////////////

            motorFrontLeft.setPower(frontLeftPower);
            motorRearLeft.setPower(RearLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorRearRight.setPower(RearRightPower);
            motorLiftLeft.setPower(LiftMotorPower);
            motorLiftRight.setPower(LiftMotorPower);
            motorExtendLeft.setPower(ExtendMotorPower);
            motorExtendRight.setPower(ExtendMotorPower);

            ///////////////////////// For Fun Section ////////////////////////////

            telemetry.addData("Status", "RUN BOB RUN!");

            //telemetry.update();
                }
        });
    }
}
}
