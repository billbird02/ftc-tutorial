/*  SimpleAutonomous - example autonomous program for Into The Deep
    (based off of AutoPixelFront.java https://firstroboticsbc.org/ftc/ftc-team-resources/centerstage-java-autonomous-programs/)
    This program moves the robot for a specified amount of time. The code assumes a the motor directions must be set so a positive power
    goes forward on all wheels. Speed and turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.

    Starting position - red alliance, audience side of field

    Program flow:
    - move forward 6 inches
    - examine each spike more in turn
    - if not found on the left side or centre, assume pixel on right side
    - drive toward indicated spike mark,
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Simple Autonomous", group = "Tutorial")
public class SimpleAutonomous extends LinearOpMode {

    DcMotor frontLeft = null;
    DcMotor backLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;

    @Override
    public void runOpMode() {

        double drive = 0;   // Desired forward power/speed (-1 to +1)
        double strafe = 0;  // Desired strafe power/speed (-1 to +1)
        double turn = 0;    // Desired turning power/speed (-1 to +1)

        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        // set the GAIN constants to control the relationship between the measured position error, and how much power is
        // applied to the drive motors to correct the error.
        // Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN = 0.02;     // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error (0.50 / 25.0)
        final double STRAFE_GAIN = 0.015;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error (0.25 / 25.0)
        final double TURN_GAIN = 0.01;      // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;  //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.5; //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        // initialize the hardware variables. Note that the deviceNames used here as parameters
        // to 'get' must match the names assigned during the robot configuration
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // to drive forward, most robots need the motor(s) on one side to be reversed since the axles are mirrored
        // when run, this OpMode should start both motors driving forward, so adjust these two lines based on your first test drive
        // note: the settings here assume direct drive on left and right wheels; gear Reduction or 90 Deg drives may require direction reversal
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // initialize IMU on REV Control Hub
        IMU imu = hardwareMap.get(IMU.class, "imu");
        double targetYaw = 0;

        // the next two lines define Hub orientation with the default orientation (shown) being when a hub is mounted horizontally
        // with the printed logo pointing UP and the USB port pointing FORWARD
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // initialize the IMU with the mounting orientation
        // note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // send telemetry message to signify robot is ready and waiting
        // This telemetry line is especially important when using the IMU,
        // as the IMU can take a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        runtime.reset();  // start timer for step 1
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            // STEP 1 move forward
            if (currentStep == 1) {
                if (runtime.milliseconds() < 400) {
                    moveRobot(1, 0, 0); // mimics gamepad controls
                } else {
                    moveRobot(0, 0, 0);
                    currentStep = 2;
                    runtime.reset();  // reset timer for next step
                }
            }

            // STEP 2 slight rotation counter-clockwise - look at left spike mark
            if (currentStep == 2) {
                if (runtime.milliseconds() < 100) {
                    moveRobot(0, 0, -1);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 3;
                    runtime.reset();  // reset timer for next step
                }
            }

            // STEP 3 back off, leaving purple pixel on mark will end up facing the red April Tags on the front wall
            if (currentStep == 3) {
                if (runtime.milliseconds() < 650) {
                    moveRobot(-1, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 10; // prepare to go backstage
                    targetYaw = 90;
                    runtime.reset();  // reset timer for next step
                }
            }

            // STEP 10 - do IMU turn to line up for run to backstage
            if (currentStep == 10) {
                if ((orientation.getYaw(AngleUnit.DEGREES) > targetYaw)
                        && (runtime.milliseconds() < 1000)) {
                    moveRobot(0, 0, 0.5); // clockwise
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 11;   // go to backdrop
                    runtime.reset();    // reset timer for next step
                }
            }

            // STEP 11 - drive by IMU Yaw heading to go backstage
            if (currentStep == 11) {
                if (runtime.milliseconds() < 2000) {
                    double headingError = targetYaw - orientation.getYaw(AngleUnit.DEGREES);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    moveRobot(-1, 0, turn); // go backwards, adjusting for yaw
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 0;  // last step completed
                    runtime.reset();  // reset timer for next step
                }
            }

            telemetry.addData("current step", currentStep);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("target yaw", targetYaw);
            telemetry.update();

        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // function to move robot along desired axis, mimicking gamepad control
        // positive values of x move forward, positive values of y strafe to the right, positive values of yaw rotate clockwise

        // calculate wheel powers
        double frontLeftPower    =  x -y -yaw;
        double frontRightPower   =  x +y +yaw;
        double backLeftPower     =  x +y -yaw;
        double backRightPower    =  x -y +yaw;

        // normalize wheel powers to not exceed +/- 1.0 range
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // send powers to the wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

}