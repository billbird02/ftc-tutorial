/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * Ref: goBilda https://github.com/goBILDA-Official/FTC-Servo-Helper-Examples
 *
 * This file contains example code designed to help your team find the signal to send
 * your servos so that they move to your desired location.
 * It is a TeleOp mode, and uses the gamepad to increment or decrement the servo position
 * while displaying that position as Telemetry on your Driver Station.
 *
 * To use this program, start with the servo mounted to your robot and the servo attachment
 * not installed yet. Then hit "init" on your drivers station to move the servo to a known "halfway"
 * position. This should be a desired state of your mechanism, like a wrist pointed straight out
 * ready to interact with game elements. On init, this code moves the servo to the "0.5" position.
 * This is halfway through the throw of the servo. So if your servo has 180° of range, this starts
 * it at 90°.
 *
 * Once your servo attachment is installed, hit the START button. This will allow you to drive the
 * servo using your gamepad. Pressing the Y(△) button will increase the signal, while pressing
 * A(X) will decrease the position.
 * If the servo is incrementing too far, or not far enough. You can adjust the rate by pressing
 * D-pad up, or d-pad down.
 */

@TeleOp(name="Servo Position Helper", group="Test")
//@Disabled
public class ServoPositionHelper extends LinearOpMode {

    // Declare OpMode member.
    private Servo servo = null;

    /*
     * Create a variable which we will modify with our code. Eventually we will instruct
     * the servo to run to the position captured by this variable.
     */
    private double servoPosition = 0.5;

    // Create a variable for size of each "step" that we will increment or decrement our servo position by.
    private double positionAdjustment = 0.05;

    // This variable captures how much we need to increment or decrement the step size by
    private final double STEP_ADJUSTMENT = 0.01;

    /*
     * This variable is the maximum position we want to send to the servo.
     * Some servos do not operate well went sent a signal too large, or too small.
     * Most Hitec Linear servos for example only respond to signals within a 1050-1950µsec range.
     * Converted to 0-1, that means we should not send a Hitec Linear Servo less than 0.25, or more than 0.75.
     */
    private final double MIN_POSITION = 0;
    private final double MAX_POSITION = 1;

    // These booleans are used in the "rising edge detection"
    private boolean previousGamepadY = false;
    private boolean previousGamepadA = false;
    private boolean previousGamepadUp = false;
    private boolean previousGamepadDown = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        /*
         * Initialize the hardware variables, string here must exactly match the name of a configured
         * servo in the Robot Configuration on your Driver Station.
         */
        servo = hardwareMap.get(Servo.class, "servo");

        /*
         * Set the servo to an initial position of 0.5, we do this before the while (opModeIsActive())
         * loop starts so that we can install the servo attachment in a "known" position.
         * There isn't anything special about doing this at 0.5. We could instead choose 0, or 1!
         */
        servoPosition = 0.5;
        servo.setPosition(servoPosition);


        telemetry.addData("Servo Set Position: ",servo.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * when the user presses a button. We have a few ways too design how our code responds.
             * The simplest is to preform an action whenever the button is pressed, in practice,
             * this means that for every cycle (fraction of a second) that the button is held, it
             * will run the code in your if statement.
             * Here we want the code in our statement to only execute once every time that the button
             * is pressed. To do this, we need to implement "rising edge detection" this works by
             * checking both the current state of the button (which we store in a boolean called
             * currentGamepadY) and the state of the button in the previous loop (which we store in
             * previousGamepadY). If previousGamepadY is false and currentGamepadY is true, then
             * the button went from not being pressed, to being pressed. Once it is held down, both
             * booleans will be true. So we can ignore those and just look for the change from one
             * cycle to the next.
             */
            boolean currentGamepadY = gamepad1.y;
            boolean currentGamepadA = gamepad1.a;
            boolean currentGamepadUp = gamepad1.dpad_up;
            boolean currentGamepadDown = gamepad1.dpad_down;

            // Check to see if the user is clicking the Y(△) button on the gamepad.
            if (currentGamepadY && !previousGamepadY){
                // += is an operator that lets us add the step variable without overwriting the servoPosition variable.
                servoPosition += positionAdjustment;
            } else if (currentGamepadA && !previousGamepadA){
                // We use an else if statement here so that we only check if A(x) is pressed after we know
                // that the Y(△) button is not pressed.
                servoPosition -= positionAdjustment;
            } else if (gamepad1.b) {    // B = PS4: Circle; open claw
                //setClawOpen();
                servoPosition = 0.55;
            } else if (gamepad1.x) {    // X = PS4: Square; close claw
                //setClawClose();
                servoPosition = 0.13;
            }

            // Modify the step size if the user clicks D-pad up or D-pad down.
            if (currentGamepadUp && !previousGamepadUp){
                positionAdjustment += STEP_ADJUSTMENT;
            } else if (currentGamepadDown && !previousGamepadDown){
                positionAdjustment -= STEP_ADJUSTMENT;
            }

            // Check to see if we're setting the servoPosition to less than the min, or more than the max.
            if (servoPosition > MAX_POSITION){
                servoPosition = MAX_POSITION;
            } else if (servoPosition < MIN_POSITION){
                servoPosition = MIN_POSITION;
            }

            /*
             * Finally, set the servo to the servoPosition variable. We do this only once per loop
             * so that we can be sure not to write conflicting positions to the servo.
             */
            servo.setPosition(servoPosition);

            // Because our logic has finished, we set our "previousGamepad" booleans to the current ones.
            previousGamepadY = currentGamepadY;
            previousGamepadA = currentGamepadA;
            previousGamepadUp = currentGamepadUp;
            previousGamepadDown = currentGamepadDown;

            // Show the servo position
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Step Size", positionAdjustment);
            telemetry.update();
        }
    }

    public void  setClawClose() {
        servo.setPosition(0.15);    // claw shouldn't close beyond servo position 0.15 to allow for gap for scoring element.
    }
    public void setClawOpen() {
        servo.setPosition(0.55);    // claw can't extend beyond servo position 0.6 due to contact with claw back plate.
    }

}