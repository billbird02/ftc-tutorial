/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive (robot centric)
                         FTC Programming Episode 9: Scaling Drive Powers Proportionally

    and https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#robot-centric-final-sample-code
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum TeleOp (Robot Centric)", group="Test")
public class MecanumTeleOpRobotCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor frontLeftDrive = hardwareMap.dcMotor.get("front_left");
        DcMotor backLeftDrive = hardwareMap.dcMotor.get("back_left");
        DcMotor frontRightDrive = hardwareMap.dcMotor.get("front_right");
        DcMotor backRightDrive = hardwareMap.dcMotor.get("back_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // TODO: Make sure all motors are facing the correct direction. Go one at a time.
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // using encoder for constant power resulting in increased accuracy
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        // Send telemetry message to signify robot waiting.
        // This telemetry line is especially important when using the IMU, as the IMU can take
        // a couple of seconds to initialize and this line executes when IMU initialization is complete.
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run until the driver presses stop
        while (opModeIsActive()) {

            // DRIVE = left joystick y axis (robot centric)
            double drive = -gamepad1.left_stick_y; // set as negative so pushing joystick forward is a positive value
            // TURN = right joystick x axis
            double turn = gamepad1.right_stick_x;
            // STRAFE = left joystick x axis
            double strafe = gamepad1.left_stick_x * 1.1;    // multiplier to counteract imperfect strafing, adjustable based on driver preference

            // the left Y component (y or DRIVE) is added to all wheels, the right X (rx or TURN) is added to the left wheels and subtracted from
            // the right, and the left X component (x or STRAFE) is added to diagonal motors pairs (i.e., left front and right rear) and subtracted
            // from the opposite diagonal pair (i.e., right front and left rear).
            // see mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
            double fLeftPower = drive + turn + strafe;
            double fRightPower = drive - turn - strafe;
            double rLeftPower = drive + turn - strafe;
            double rRightPower = drive - turn + strafe;

            // determine the largest motor power (absolute value), then scale power to ensure all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double[] adjPower = scalePower(fLeftPower, fRightPower, rLeftPower, rRightPower);
            frontLeftDrive.setPower(adjPower[0]);
            frontRightDrive.setPower(adjPower[1]);
            backLeftDrive.setPower(adjPower[2]);
            backRightDrive.setPower(adjPower[3]);
        }
    }

    public double[] scalePower(double fLeftPower, double fRightPower, double rLeftPower, double rRightPower) {
    // function to determine max power and scale proportionally.  returns an array.

        // determine maximum power value of the four motors
        double max = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(fRightPower), Math.max(Math.abs(rLeftPower), Math.abs(rRightPower))));

        if (max > 1) {  // proportionally scale motor power levels so values do not exceed +/- 1.0
            fLeftPower /= max;
            fRightPower /= max;
            rLeftPower /= max;
            rRightPower /= max;
        }

        return new double[] { fLeftPower, fRightPower, rLeftPower, rRightPower };
    }
}