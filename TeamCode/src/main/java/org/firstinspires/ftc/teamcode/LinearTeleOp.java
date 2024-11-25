package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "First_TeleOp") // imports TeleOp code and sets unique program name on control hub
public class LinearTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // MOTORS
        DcMotor motor1;
        motor1 = hardwareMap.get(DcMotor.class, "motor_one"); // deviceName must match the Control Hub config file

        motor1.setDirection(DcMotorSimple.Direction.FORWARD); // set motor direction (FORWARD | REVERSE)
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set behavior on motor stop (BRAKE | FLOAT)
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    // run motor at target velocity, requires encoder
        // other RunMode options RUN_WITHOUT_ENCODER | STOP_AND_RESET_ENCODER)

        DcMotorEx motor2;
        motor2 = hardwareMap.get(DcMotorEx.class, "motor_two"); // map variable to configuration file name

        // SERVOS
        Servo servo1;
        servo1 = hardwareMap.get(Servo.class, "servo_one");
        servo1.scaleRange(0.2, 0.8);    // scales servo movement range, i.e. defines movement range subset

        CRServo servo2; // must use servo programmer to switch servo mode to continuous
        servo2 = hardwareMap.get(CRServo.class, "servo_two");

        waitForStart(); // waits until start is pressed

        while (opModeIsActive()) {
            motor1.setPower(0.5);    // values from -1 to 1 inclusive
            motor2.setVelocity(150);    // set velocity based on ticks per second; more accurate than power

            servo1.setPosition(0.4);    // values from 0 to 1 inclusive.  sets initial servo position (e.g. 0.5 is halfway in range
            servo2.setPower(0.85);  // values from 0 to 1 inclusive
        }

    }
}
