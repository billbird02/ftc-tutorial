/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode shows how to use the new universal IMU interface. This
 * interface may be used with the BNO055 IMU or the BHI260 IMU. It assumes that an IMU is configured
 * on the robot with the name "imu".
 *
 * The sample will display the current Yaw, Pitch and Roll of the robot.<br>
 * With the correct orientation parameters selected, pitch/roll/yaw should act as follows:
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
 *
 * The yaw can be reset (to zero) by pressing the X (PS4: □) button on the gamepad.
 *
 * This specific sample assumes that the Hub is mounted on one of the three orthogonal planes
 * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
 *
 * Note: if your Hub is mounted on a surface angled at some non-90 Degree multiple (like 30) look at
 *       the alternative SensorIMUNonOrthogonal sample in this folder.
 *
 * This "Orthogonal" requirement means that:
 *
 * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 *
 * 2) The USB ports can only be pointing in one of the same six directions:<br>
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 *
 * So, To fully define how your Hub is mounted to the robot, you must simply specify:<br>
 *    logoFacingDirection<br>
 *    usbFacingDirection
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * Finally, choose the two correct parameters to define how your Hub is mounted and edit this OpMode
 * to use those parameters.
 */
@TeleOp(name = "IMU (Gyro) Test", group = "Test")
//@Disabled   // comment this out to add to the OpMode list on the Drive Hub
public class TestSensorIMU extends LinearOpMode
{
    private DcMotor frontLeftMotor = null;
    private DcMotor rearLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor rearRightMotor = null;

    // The IMU sensor object
    IMU imu;

    @Override public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings specified here as parameters
        // must match the names assigned in the robot controller configuration.
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right");

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
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve and initialize the IMU.
        // This expects the IMU to be configured in the REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         *
         * If you are using a REV 9-Axis IMU, you can use the Rev9AxisImuOrientationOnRobot class instead of the
         * RevHubOrientationOnRobot class, which has an I2cPortFacingDirection instead of a UsbFacingDirection.
         */

        // The next two lines define Hub orientation.
        // The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
        // TODO:  EDIT these two lines to match YOUR mounting configuration.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with the specified mounting orientation.
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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

            moveRobot(forward, strafe, turn, 1.0);  // speedLimiter reduces movement speed to a specified % of maximum (1.0).

            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.x) {   // PS4: □
                telemetry.addData("Yaw", "Resetting...\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press X (PS4: □) on Gamepad to reset.\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
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
        // (i.e., left front and right rear), and the right X (rx or TURN) is added to the left wheels and subtracted from the right.
        // See mecanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
        double frontLeftPower = (y + x + rx) / kScaleFactor;
        double rearLeftPower = (y - x + rx) / kScaleFactor;
        double frontRightPower = (y - x - rx) / kScaleFactor;
        double rearRightPower = (y + x - rx) / kScaleFactor;

        // Set scaled motor powers (with limiter).
        frontLeftMotor.setPower(frontLeftPower * speedLimiter);
        rearLeftMotor.setPower(rearLeftPower * speedLimiter);
        frontRightMotor.setPower(frontRightPower * speedLimiter);
        rearRightMotor.setPower(rearRightPower * speedLimiter);
    }
}
