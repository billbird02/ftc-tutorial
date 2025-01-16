/*
MoveSlideTest.java | Created 06 January 2025 | Updated 06 January 2025

Java class to test goBilda Viper slide movement (extension/retraction) using a specified distance in millimeters (mm).
Note: RUN_USING_ENCODER - attempts to run the motor at a targeted velocity
      RUN_WITHOUT_ENCODERS - runs the motor at whatever velocity is achieved by applying a particular power level to the motor.
      Regardless of either setting, the motor encoder cn be used.

public class ProgrammingBoard4  // Learn Java for FTC pg 54
  private DcMotor motor;
  private double ticksPerRotation;

  @Override
  public void init() {
    motor = hardwareMap.dcMotor.get("motor0");
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ticksPerRotation = motor.getMotorType().getTicksPerRev();
  }

  public double getMotorRotations() {
    return motor.getCurrentPosition() / ticksPerRotation;
  }

  @Override
  public void loop() {
    motor.setMotorSpeed(0.5);
    telemetry.addData("Motor rotations: ", getMotoRotations());
  }

 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Move Slide Test", group="Test")
@Disabled
public class TestMoveSlide extends LinearOpMode {

    public DcMotor armMotor = null;     // arm lft motor
    public DcMotor slideMotor = null;   // Viper slide extension/retraction motor

    /* This constant is the number of encoder ticks for each degree of rotation of the arm. To find this, we first
    need to consider the total gear reduction powering our arm. First, we have an external 20t:100t (5:1) reduction
    created by two spur gears. But we also have an internal gear reduction in our motor. The motor we use for
    this arm is a 117RPM Yellow Jacket, which has an internal gear reduction of ~50.9:1 (more precisely it is 250047/4913:1).
    We can multiply these two ratios together to get our final reduction of ~254.47:1. The motor's encoder
    counts 28 times per rotation. So in total you should see about 7125.16 counts per rotation of the arm.
    We divide that by 360 to get the counts per degree. */
    final double ARM_MOTOR_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox = (((1+(46/17))) * (1+(46/17))) * (1+(46/17)))
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    /* Constants to hold the position that the arm is commanded to run to in degrees. These are relative to where the arm was
    located when you start the OpMode. So make sure the arm is reset to collapsed inside the robot before you start the program. */
    final double ARM_DOWN  = 0;     // R13D uses ARM_COLLAPSED_INTO_ROBOT
    // TODO: adjust degrees to reach a desired target arm angle relative to the start position.
    final double ARM_UP = 45 * ARM_MOTOR_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_DOWN; // set initial arm position on floor

    /* Convert slide motor ticks to millimeters (mm). The 2 Stage Viper Slide uses a 60 tooth GT2 pulley to move
    the slide 120mm per rotation. With a maximum travel distance of 488mm (19.2"), full extension is reach at ~4.1 rotations
    of the motor shaft.

    From the 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX™ Shaft, 312 RPM, 3.3 - 5V Encoder) specs:
       TICKS_PER_REVOLUTION = 28    // at encoder shaft
       INTERNAL_GEAR_RATIO = (((1+(46/17))) * (1+(46/11)))  // 19.2:1 gear ratio formula
    and
       EXTERNAL_GEAR_RATIO = n/a
       PULLEY_CIRCUMFERENCE_MM = 120
    thus,
       MOTOR_TICKS_PER_MM = (TICKS_PER_REVOLUTION * INTERNAL_GEAR_RATIO * EXTERNAL_GEAR_RATIO) / PULLEY_CIRCUMFERENCE_MM */
    final double SLIDE_MOTOR_TICKS_PER_MM =
            28 // Number of encoder ticks per rotation of the bare motor
                    * 3591.0 / 187.0 // Exact gear ratio of the 19.2:1 Yellow Jacket gearbox.  19.2:1 = (((1+(46/17))) * (1+(46/11))):1
                    * 1 / 120.0; // Ticks per mm; viper slide moves 120mm per rotation
    // ((((1+(46/17))) * (1+(46/11))) * 28) / 120.0 = (100548.0 / 187.0) / 120.0
    // R13D had (111132.0 / 289.0) / 120.0 for the 435 RPM version (SKU: 5203-2402-0014) of the motor.
    // Note, 111132.0/289.0 = 384.5397 PPR (which is approximately the Encoder Resolution on the goBilda spec sheet)

    final double SLIDE_RETRACTED = 0 * SLIDE_MOTOR_TICKS_PER_MM;
    final double SLIDE_EXTENDED_MAX = 480 * SLIDE_MOTOR_TICKS_PER_MM;
    double slidePosition = (int)SLIDE_RETRACTED;

    double cycletime = 0;
    double loopStart = 0;
    double priorStart = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        // Set the maximum current that the control hub will apply to the arm before throwing a flag.
        ((DcMotorEx) slideMotor).setCurrentAlert(5,CurrentUnit.AMPS);

        // Define and initialize the lift arm motor. Before starting the armMotor, make sure the TargetPosition is set to 0.
        armMotor = hardwareMap.dcMotor.get("armMotor");    // lift arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER per gm0 (https://gm0.org/en/latest/docs/software/tutorials/encoders.html)
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize the Viper side motor.  Motor may need to be reversed based on installation position.
        slideMotor = hardwareMap.dcMotor.get("slideMotor");    // slide motor
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get number of encoder ticks per rotation for the motor type specified in the Control Hub configuration
        double ticksPerRotation = slideMotor.getMotorType().getTicksPerRev();

        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER per gmo (https://gm0.org/en/latest/docs/software/tutorials/encoders.html)
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting.
        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){

            // Define a set of if else statements to set our arm to different arm and slide positions.
            if (gamepad1.dpad_up){   // raise arm
                armPosition = ARM_UP;
            }

            else if (gamepad1.dpad_down){   // lower arm
                armPosition = ARM_DOWN;
                slidePosition = SLIDE_RETRACTED;
            }

            /* Set the target position of our arm to match the variable that was selected by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it. */
            moveArm((int) armPosition, 1500);

            /* Set the slide position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The slide is run with encoders. So it knows exactly where it is,
            and there's a limit to how far in and out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm, where we click a button and it goes to a position, then stops. But the drivers wanted more
            "open loop" controls. So we want the slide to keep extending for as long as we hold the bumpers, and when we let go of the bumper,
            stop where it is at. This allows the driver to manually set the position. But it also lets us enforce the end stops for the slide in software.
            So that the motor can't run past its endstops and stall. We have our slidePosition variable, which we increment or decrement for every cycle
            (every time our main robot code runs) that we're holding the button. Now since every cycle can take a different amount of time to complete, and
            we want the slide to move at a constant speed, we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the slide to run at (in mm/sec) by the cycletime variable. There's no way that our slide can move 2800mm in one cycle, but
            since each cycle is only a fraction of a second, we are only incrementing it a small amount each cycle. */
            if (gamepad1.right_bumper){       // extend slide
                slidePosition += 2800 * cycletime;
            }
            else if (gamepad1.left_bumper){   // retract slide
                slidePosition -= 2800 * cycletime;
            }
            else if (gamepad1.b) {       // extend slide halfway (b = PS4: Circle)
                slidePosition = 0.5 * SLIDE_EXTENDED_MAX;
            }
            else if (gamepad1.y) {  // fully extend slide (y = PS4: Triangle)
                slidePosition = SLIDE_EXTENDED_MAX;
            }
            else if (gamepad1.x) {   // retract slide (x = PS4: Square)
                slidePosition = SLIDE_RETRACTED;
            }
            else if (gamepad1.a) {  // stop slide motor (a = PS4: Cross)
                slideMotor.setPower(0);
            }

            if (slidePosition > SLIDE_EXTENDED_MAX){    // enforce maximum slide extension limit.
                slidePosition = SLIDE_EXTENDED_MAX;
            }

            if (slidePosition < 0) {    // enforce slide retraction limit; ensure slide does not go past 0.
                slidePosition = 0;
            }

            moveSlide((int) slidePosition, 1000);

            // Check to see if our arm is over the current (amp) limit, and report via telemetry.
            if (((DcMotorEx) slideMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT (AMP) LIMIT!");
            }

            /* This is how we check our loop time. We create three variables:
                loopStart = the current time when we hit this part of the code
                cycletime = the amount of time in seconds our current loop took
                priorStart = the time in seconds that the previous loop started at
            and find cycletime by subtracting the old time from the current time. */
            loopStart = getRuntime();
            cycletime = loopStart - priorStart;
            priorStart = loopStart;

            // Send telemetry to the driver of the arm's current position and target position.
            telemetry.addLine("ARM MOTOR");
            telemetry.addData("Arm target position: ", armMotor.getTargetPosition());
            telemetry.addData("Arm encoder position: ", armMotor.getCurrentPosition());
            telemetry.addData("Arm motor ticks per degree: ", ARM_MOTOR_TICKS_PER_DEGREE);
            telemetry.addLine();
            telemetry.addLine("SLIDE MOTOR");
            telemetry.addData("slidePosition variable: ", slidePosition);
            telemetry.addData("Slide target position: ", slideMotor.getTargetPosition());
            telemetry.addData("Slide encoder position: ", slideMotor.getCurrentPosition());
            telemetry.addData("Slide motor ticks per rotation: ", ticksPerRotation);
            telemetry.addData("Slide motor ticks per mm:", SLIDE_MOTOR_TICKS_PER_MM);
            telemetry.addData("Slide motor # of rotations: ", slideMotor.getCurrentPosition() / ticksPerRotation);
            telemetry.addData("slide Motor Amps: ", ((DcMotorEx) slideMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    public void moveArm(int position, int velocity) {
        armMotor.setTargetPosition(position);

        ((DcMotorEx) armMotor).setVelocity(velocity);   // Ri3D had setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveSlide(int position, int velocity) {
        slideMotor.setTargetPosition(position);

        ((DcMotorEx) slideMotor).setVelocity(velocity); // Ri3D had setVelocity(2100);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}