package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(preselectTeleOp="Gobilda_Drive_Base")
public class Auto_Start_Unfold extends LinearOpMode {



    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    
    DcMotor lifter;
    DcMotor Extender;
    
    
    Servo claw;
    Servo flipper;
    Servo wrist;


    BHI260IMU imu;
    
    ElapsedTime timer = new ElapsedTime();
    
    double robotHeading  = 0;
    double headingOffset = 0;
    double headingError  = 0;

    
    double  targetHeading = 0;
    double  driveSpeed    = 0;
    double  turnSpeed     = 0;
    double  leftSpeed     = 0;
    double  rightSpeed    = 0;
    double  liftSpeed     = 0;

    int     frontLeftTarget    = 0;
    int     frontRightTarget   = 0;
    int     backLeftTarget     = 0;
    int     backRightTarget    = 0;
    int     liftTarget         = 0;
    
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.25;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 3.0 ;    // How close must the heading get to the target before moving to next step.
                                                              // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.025;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.015;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {


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


        
        Servo flipper = hardwareMap.servo.get("flipper");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");



        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, 
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        // Technically this is the default, however specifying it is clearer
        // parameters.angleUnit = BHI260IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(new BHI260IMU.Parameters(RevOrientation));
        

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
        

        // Reverse the Left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Lift motor reversed for correct encoder counts!
        motorLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtendRight.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLiftLeft.setTargetPosition(0);
        motorLiftRight.setTargetPosition(0);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        

        waitForStart();
        resetHeading();

        
        
        
        
        
        
        
        /****************************************


                 Robot Starts Moving


         ****************************************/

        
        claw.setPosition(0);
        sleep(1000);
        flipper.setPosition(0.42);
        wrist.setPosition(0.625);
        sleep(3000);
        claw.setPosition(0.3);
        sleep(3000);    
        telemetry.update();
        









        /****************************************


                        End of Auto


         ****************************************/












    }
    
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    
  public void driveStrafeRight(double speed, double time) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            timer.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && timer.seconds() < time) {
                 frontLeft.setPower(speed);
                 frontRight.setPower(-speed);
                 backLeft.setPower(-speed);
                 backRight.setPower(speed);
                
            }

            // Stop all motion & Turn off RUN_TO_POSITION
             frontLeft.setPower(0);
             frontRight.setPower(0);
             backLeft.setPower(0);
             backRight.setPower(0);

             frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void driveStrafeLeft(double speed, double time) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            timer.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && timer.seconds() < time) {
                 frontLeft.setPower(-speed);
                 frontRight.setPower(speed);
                 backLeft.setPower(speed);
                 backRight.setPower(-speed);
                
            }

            // Stop all motion & Turn off RUN_TO_POSITION
             frontLeft.setPower(0);
             frontRight.setPower(0);
             backLeft.setPower(0);
             backRight.setPower(0);

             frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget =  frontLeft.getCurrentPosition()+ moveCounts;
            frontRightTarget =  frontRight.getCurrentPosition() + moveCounts;
            backLeftTarget =  backLeft.getCurrentPosition() + moveCounts;
            backRightTarget =  backRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
             frontLeft.setTargetPosition(frontLeftTarget);
             frontRight.setTargetPosition(frontRightTarget);
             backRight.setTargetPosition(backRightTarget);
             backLeft.setTargetPosition(backLeftTarget);
            
            

             frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                  ( frontLeft.isBusy() &&  frontRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
             frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

         frontLeft.setPower(leftSpeed);
         frontRight.setPower(rightSpeed);
         backRight.setPower(rightSpeed);
         backLeft.setPower(leftSpeed);
        
        
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      frontLeftTarget,  frontRightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",       frontLeft.getCurrentPosition(),
                     frontRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   =  imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }
}
