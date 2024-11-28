/*
    From FTC Team 7477 - FTC Programming Episode 8: Mechanum Drive
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mechanum Drive", group="FTC Tutorial")
//@Disabled
public class MechanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization code
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rLeft");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "rRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // using encoder for increased accuracy
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double drive, turn, strafe; // DRIVE forward and reverse, STRAFE is left and right
        double fLeftPower, fRightPower, rLeftPower, rRightPower;

        waitForStart();

        while (opModeIsActive()) {

            // DRIVE = left joystick y axis
            drive = -gamepad1.left_stick_y; // set as negative so pushing joystick forward is a positive value
            // TURN = right joystick x axis
            turn = gamepad1.right_stick_x;
            // STRAFE = left joystick x axis
            strafe = gamepad1.left_stick_x;

            // mechanum drive reference: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2234/13280/3209-0001-0007-Product-Insight-2__06708__33265.1725633323.png?c=1
            fLeftPower = drive + turn + strafe;
            fRightPower = drive - turn - strafe;
            rLeftPower = drive + turn - strafe;
            rRightPower = drive - turn + strafe;

        }
    }
}

