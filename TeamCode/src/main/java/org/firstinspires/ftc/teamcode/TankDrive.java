/*
    From FTC Team 7477 - FTC Programming Episode 7: Programming a Tank Drive Train

    Tank Drive using 4 drive motors each driving a wheel.  Can simplify to 2 drive motors with each driving
    a pair of wheels connected with a chain system.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.io.CharArrayWriter;

@TeleOp(name="Tank Drive", group="FTC Tutorial")
//@Disabled
public class TankDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization code
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rLeft");
        DcMotor rearRight = hardwareMap.get(DcMotor.class, "rRight");

        double drive;
        double turn;
        double leftDrivePower, rightDrivePower;

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            // tank drive has 2 components: DRIVE (i.e., forward and reverse) by setting the same power to each motor pair and TURN
            // DRIVE forward and reverse using gamepad left joystick y axis
            drive = -gamepad1.left_stick_y; // set as negative so pushing joystick forward is a positive value

            // TURN right and left using gamepad right joystick x axis
            turn = gamepad1.right_stick_x;

            leftDrivePower = Range.clip(drive - turn, -1, 1);   // ensure power stays in appropriate range
            rightDrivePower = Range.clip(drive + turn, -1, 1);

            frontLeft.setPower(leftDrivePower);
            rearLeft.setPower(leftDrivePower);
            frontRight.setPower(rightDrivePower);
            rearRight.setPower(rightDrivePower);
        }
    }
}
