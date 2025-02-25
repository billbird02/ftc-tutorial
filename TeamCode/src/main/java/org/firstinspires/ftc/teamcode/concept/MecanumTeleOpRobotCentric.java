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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum TeleOp (Robot Centric)", group="Concept")
//@Disabled   // comment this out to add to the OpMode list on the Driver Hub
public class MecanumTeleOpRobotCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings specified here as parameters
        // must match the names assigned in the robot controller configuration.
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        DcMotor rearLeftMotor = hardwareMap.dcMotor.get("rear_left");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("front_right");
        DcMotor rearRightMotor = hardwareMap.dcMotor.get("rear_right");

        // Set drive speed limiter (full power = 1.0)
        final double DRIVE_SPEED_LIMITER = 0.6; // reduce drive rate to 60% of maximum

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

            // DRIVE = left joystick y axis (robot centric)
            double drive = -gamepad1.left_stick_y;  // set as negative so pushing joystick forward is a positive value
            // STRAFE = left joystick x axis
            double strafe = gamepad1.left_stick_x * 1.1;    // multiplier to counteract imperfect strafing, adjustable based on driver preference
            // TURN = right joystick x axis
            double turn = gamepad1.right_stick_x;

            // Determine the largest motor power (absolute value) or 1. This ensures all the powers are scaled proportionally,
            // but only if at least one is out of the range [-1.0, 1.0].
            double kScaleFactor = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

            // The left Y component (y or DRIVE) is added to all wheels, the right X (rx or TURN) is added to the left wheels and subtracted from
            // the right, and the left X component (x or STRAFE) is added to diagonal motors pairs (i.e., left front and right rear) and subtracted
            // from the opposite diagonal pair (i.e., right front and left rear).
            // see mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
            double frontLeftPower = squarePower((drive + turn + strafe) / kScaleFactor);
            double rearLeftPower = squarePower((drive + turn - strafe) / kScaleFactor);
            double frontRightPower = squarePower((drive - turn - strafe) / kScaleFactor);
            double rearRightPower = squarePower((drive - turn + strafe) / kScaleFactor);

            // Set scaled motor powers (with limiter).
            frontLeftMotor.setPower(frontLeftPower * DRIVE_SPEED_LIMITER);
            rearLeftMotor.setPower(rearLeftPower * DRIVE_SPEED_LIMITER);
            frontRightMotor.setPower(frontRightPower * DRIVE_SPEED_LIMITER);
            rearRightMotor.setPower(rearRightPower * DRIVE_SPEED_LIMITER);

            // Display power settings on Driver Hub
            telemetry.addData("kScaleFactor", kScaleFactor);
            telemetry.addLine()
                    .addData("FrontLeft Power", "%4.2f", ((drive + turn + strafe) / kScaleFactor))
                    .addData("FrontLeft Power²", "%4.2f", frontLeftPower);
            telemetry.addLine()
                    .addData("RearLeft Power", "%4.2f", ((drive + turn - strafe) / kScaleFactor))
                    .addData("RearLeft Power²", "%s", rearLeftPower);
            telemetry.addLine()
                    .addData("FrontRight Power", "%4.2f", ((drive - turn - strafe) / kScaleFactor))
                    .addData("FrontRight Power²", "%s", frontRightPower);
            telemetry.addLine()
                    .addData("RearRight Power", "%4.2f", ((drive - turn + strafe) / kScaleFactor))
                    .addData("RearRight Power²", "%s", rearRightPower);
            telemetry.update();
        }
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