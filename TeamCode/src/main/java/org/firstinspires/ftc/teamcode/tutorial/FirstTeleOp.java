/*
From FTC Team 7477 Youtube video series: https://youtu.be/ig9YUI4wu6c?si=GKPDrYQiAySjAnxz
 */
package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "First TeleOp") // adds TeleOp library import and sets program name on control hub
@Disabled
public class FirstTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // MOTORS
        DcMotor motor1;
        motor1 = hardwareMap.get(DcMotor.class, "motor_one"); // deviceName must match the Control Hub config file

        motor1.setDirection(DcMotorSimple.Direction.FORWARD); // set motor direction (FORWARD | REVERSE)
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set behavior on motor stop (BRAKE | FLOAT)
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    // run motor at target velocity, requires encoder
        // other RunMode options RUN_WITHOUT_ENCODER | STOP_AND_RESET_ENCODER)

        DcMotorEx liftArm = hardwareMap.get(DcMotorEx.class, "lift_arm");   // enhanced motor functionality
        liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int degrees = 1000/4; // motor has 1000 ticks per rotation. desired position is 90 degrees, which is a 1/4 rotation
        int position = 0;

        DcMotor intake = hardwareMap.dcMotor.get("intake");    // shorthand motor declaration and initiation statement
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx motor2;
        motor2 = hardwareMap.get(DcMotorEx.class, "motor_two"); // map variable to configuration file name

        // SERVOS
        Servo servo1;
        servo1 = hardwareMap.get(Servo.class, "servo_one");
        servo1.scaleRange(0.2, 0.8);    // scales servo movement range, i.e. defines movement range subset

        CRServo servo2; // must use servo programmer to switch servo mode to continuous
        servo2 = hardwareMap.get(CRServo.class, "servo_two");

        // GAME CONTROLLER (Logitech F310)
        double joystick;
        boolean isPressed;
        boolean isDown = true;
        boolean previousCycle = false, currentCycle = false;

        waitForStart(); // waits until start is pressed

        while (opModeIsActive()) {
            motor1.setPower(0.5);    // values from -1 to 1 inclusive
            motor2.setVelocity(150);    // set velocity based on ticks per second; more accurate than power


            servo1.setPosition(0.4);    // values from 0 to 1 inclusive.  sets initial servo position (e.g. 0.5 is halfway in range
            servo2.setPower(0.85);  // values from 0 to 1 inclusive to define power for continuous servo movement; similar to motors


            if (gamepad2.a) {
                servo1.setPosition(1);
            }
            if (gamepad2.b) {
                servo1.setPosition(0);
            }
            joystick = gamepad1.left_stick_x;   // moving joystick LEFT is a negative value and RIGHT is a positive value
            joystick = gamepad1.left_stick_y;   // moving joystick UP is a negative value and DOWN is a positive value
                                                // use -gamepad1.left_stick_y (equivalent to gamepad1.left_stick_y * -1) for
                                                // more intuitive operation where UP is positive and DOWN is negative.
            motor1.setPower(gamepad1.left_stick_x); // for controlling motor power level

            isPressed = gamepad1.right_trigger > 0.05;
            if (isPressed) {
                servo1.setPosition(0.6);
            }

            /*
            This code segment will not work due to how quickly the program loops through the while block; it will run
            through several iterations while the button is pressed and will quickly cycle through the UP and DOWN toggles.

            if (gamepad1.a) {
                isDown = !isDown;   // toggles state of button 'a' (e.g., false -> true or true -> false)
                if (isDown) {
                    servo1.setPosition(0);  // move DOWN
                } else {
                    servo1.setPosition(1);  // move UP
                }
            }

            The following code segment is what works; need to track current and previous values.
            */

            previousCycle = currentCycle;
            currentCycle = gamepad1.a;
            if (currentCycle && !previousCycle) {   // set leading edge detector; code is only run once when button is pressed
                                                    // (https://gm0.org/en/latest/docs/software/tutorials/gamepad.html#rising-edge-detector)
                isDown = !isDown;   // toggles state of button 'a' (e.g., false -> true or true -> false)
                if (isDown) {
                    servo1.setPosition(0);  // move DOWN
                } else {
                    servo1.setPosition(1);  // move UP
                }
            }


            liftArm.setPower(0.5);  // run motor at 50% power
            liftArm.setTargetPosition(degrees);
            liftArm.setTargetPositionTolerance(3);  // allows for a position range to account for movement jitter, in this case 247 - 25 tpr
            liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // move liftarm to desired position (90 degrees)

            position = liftArm.getCurrentPosition();    // returns the current reading of the motor encoder


            intake.setPower(0.5);   // by using encoder, the motor will attempt to regulate a constant power level, independent of battery level
                                    // otherwise, there would be variation in power level base on battery level.


        }
    }
}