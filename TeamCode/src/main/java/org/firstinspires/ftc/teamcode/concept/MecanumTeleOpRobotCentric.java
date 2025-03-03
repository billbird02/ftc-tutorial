/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mecanum TeleOp (Robot Centric)", group="Concept")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOpRobotCentric extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor rearLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor rearRightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings specified here as parameters
        // must match the names assigned in the robot controller configuration.
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // TODO: Make sure all motors are facing the correct direction.
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoder for constant power resulting in increased accuracy
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot is ready.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run until the driver presses stop
        while (opModeIsActive()) {

            // FORWARD/BACKWARD or LONGITUDINAL = left joystick y axis (robot centric)
            double forward = -gamepad1.left_stick_y;  // set as negative so pushing joystick forward is a positive value
            // STRAFE = left joystick x axis
            double strafe = gamepad1.left_stick_x * 1.1;    // multiplier to counteract imperfect strafing, adjustable based on driver preference
            // TURN = right joystick x axis
            double turn = gamepad1.right_stick_x;

            moveRobot(forward, strafe, turn, 0.6);  // speedLimiter reduces movement speed to a specified % of maximum (1.0).
        }
    }

    public void moveRobot(double y, double x, double rx, double speedLimiter) {
        // Function to move robot.
        // Positive values of y move forward, positive values of x strafe to the right, positive values of rx rotate clockwise.
    
        // Determine the largest motor power (absolute value) or 1. This ensures all the powers are scaled proportionally,
        // but only if at least one is out of the range [-1.0, 1.0].
        double kScaleFactor = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // The left Y component (y or LONGITUDINAL) is added to all wheels, the left X component (x or STRAFE) is added
        // to diagonal motors pairs (i.e., left front and right rear) and subtracted from the opposite diagonal pair
        // (i.e., right front and left rear), and the right X (rx or TURN) is added to the left wheels and subtracted from the right.
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double frontLeftPower = squarePower((y + x + rx) / kScaleFactor);
        double rearLeftPower = squarePower((y - x + rx) / kScaleFactor);
        double frontRightPower = squarePower((y - x - rx) / kScaleFactor);
        double rearRightPower = squarePower((y + x - rx) / kScaleFactor);
    
        // Set scaled motor powers (with limiter).
        frontLeftMotor.setPower(frontLeftPower * speedLimiter);
        rearLeftMotor.setPower(rearLeftPower * speedLimiter);
        frontRightMotor.setPower(frontRightPower * speedLimiter);
        rearRightMotor.setPower(rearRightPower * speedLimiter);
    }
    
    public static double squarePower(double power) {
        // Function to square motor power to allow for better micro control.  Returns a double.
    
        if (power < 0) {    // check for negative power input
            return -Math.pow(power, 2);  // return negative result if negative power input
        } else {
            return Math.pow(power, 2); // return positive result if positive power input
        }
    }
}