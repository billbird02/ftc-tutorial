package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Toggle Motor State", group="Tutorial")
public class ToggleMotorState extends OpMode {
    // define state variables
    boolean bAlreadyPressed;
    boolean motorOn;

    // declare motor
    private DcMotor testMotor = null;

    @Override
    public void init() {    // runs ONCE after INIT is pressed on the driver hub.
        testMotor  = hardwareMap.get(DcMotor.class, "front_left");  // defined in mecanum_servo config
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot is initialized and waiting.
        telemetry.addLine("Robot ready.");
        telemetry.update();
    }

    @Override
    public void loop() {    // runs REPEATEDLY when START is pressed until STOP is pressed.
        if (gamepad1.circle && !bAlreadyPressed) {
            motorOn = !motorOn; // invert boolean value
            telemetry.addData("motorOn variable: ", motorOn);

            if (motorOn) {
                testMotor.setPower(0.5);    // motor on
            } else {
                testMotor.setPower(0);      // motor off
            }
        }
        bAlreadyPressed = gamepad1.circle;   // set boolean variable to value of gamepad.b (PS4 circle)
    }
}
