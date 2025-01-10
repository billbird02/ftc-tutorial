/*
MoveSlideTest.java | Created 06 January 2025 | Updated 09 January 2025

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

//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Move Slide Test", group="Test")
//@Disabled
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

    /* Number of degrees that can be manually adjusted by the gamepad triggers to position the arm. */
    final double ARM_MANUAL_ADJUST = 15 * ARM_MOTOR_TICKS_PER_DEGREE; // R13D used FUDGE_FACTOR

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_DOWN; // set initial arm position on floor
    double armPositionManualAdjust;

    /* Convert slide motor ticks to millimeters (mm). The 2 Stage Viper Slide uses a 60 tooth GT2 pulley to move
    the slide 120mm per rotation. With a maximum travel distance of 488mm (19.2"), full extension is reach at ~4.1 rotations
    of the motor shaft.

    From the 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX™ Shaft, 312 RPM, 3.3 - 5V Encoder) specs:
       TICKS_PER_ROTATION = 28    // at encoder shaft
       INTERNAL_GEAR_RATIO = (((1+(46/17))) * (1+(46/11)))  // 19.2:1 gear ratio formula
    and
       EXTERNAL_GEAR_RATIO = n/a
       PULLEY_CIRCUMFERENCE_MM = 120
    thus,
       MOTOR_TICKS_PER_MM = (TICKS_PER_ROTATION * INTERNAL_GEAR_RATIO * EXTERNAL_GEAR_RATIO) / PULLEY_CIRCUMFERENCE_MM */

    final double SLIDE_MOTOR_TICKS_PER_MM =
            28 // Number of encoder ticks per rotation of the bare motor
                    * 3591.0 / 187.0 // Exact gear ratio of the 19.2:1 Yellow Jacket gearbox.  19.2:1 = (((1+(46/17))) * (1+(46/11))):1
                    * 1 / 120.0; // Ticks per mm; viper slide moves 120mm per rotation
    // ((((1+(46/17))) * (1+(46/11))) * 28) / 120.0 = (100548.0 / 187.0) / 120.0
    // R13D had (111132.0 / 289.0) / 120.0 for the 435 RPM version (SKU: 5203-2402-0014) of the motor.
    // Note, 111132.0/289.0 = 384.5397 PPR (which is approximately the Encoder Resolution on the goBilda spec sheet)

    final double SLIDE_RETRACTED = 0;
    final double SLIDE_EXTENDED_MAX = 480 * SLIDE_MOTOR_TICKS_PER_MM;
    double slidePosition = (int) SLIDE_RETRACTED;


    @Override
    public void runOpMode() throws InterruptedException {

        // Set the maximum current that the control hub will apply to the arm before throwing a flag.
        //((DcMotorEx)armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        // Define and initialize the lift arm motor. Before starting the armMotor, make sure the TargetPosition is set to 0.
        armMotor = hardwareMap.dcMotor.get("armMotor");    // lift arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setTargetPosition((int) armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize the Viper side motor.  Motor may need to be reversed based on installation position.
        slideMotor = hardwareMap.dcMotor.get("slideMotor");    // slide motor
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get number of encoder ticks per rotation for the motor type specified in the Control Hub configuration
        double ticksPerRotation = slideMotor.getMotorType().getTicksPerRev();

        slideMotor.setTargetPosition((int) slidePosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        //slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting.
        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* Manual adjustment to the arm position.
            This allows the driver to adjust the arm position slightly with the gamepad triggers. We want the RIGHT trigger to move
            the arm up, and LEFT trigger to move the arm down, so we add the LEFT trigger's variable to the inverse of the RIGHT trigger.
            If you pull both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our ARM_MANUAL_ADJUST factor. */
            armPositionManualAdjust = ARM_MANUAL_ADJUST * (gamepad1.left_trigger + (-gamepad1.right_trigger));

            /* Define fixed arm and corresponding slide positions. */
            if (gamepad1.dpad_up) {         // raise arm
                armPosition = ARM_UP;
            }

            else if (gamepad1.dpad_down) {  // lower arm
                armPosition = ARM_DOWN;
                //slidePosition = SLIDE_RETRACTED;
            }

            /* Set the target position of the arm by adding the armPosition variable to our armPositionFudgeFactor.
            We also set the target velocity (speed) at which to run the motor. */
            armMotor.setTargetPosition((int) (armPosition + armPositionManualAdjust));
            ((DcMotorEx)armMotor).setVelocity(1200);    // Ri3D had setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Define fixed slide positions. */
            if (gamepad1.b) {       // extend slide (b = PS4: Circle)
                slidePosition = 0.2 * SLIDE_EXTENDED_MAX;
            }
            else if (gamepad1.y) {  // fully extend slide (y = PS4: Triangle)
                slidePosition = SLIDE_EXTENDED_MAX;
            }
            else if (gamepad1.x) {   // retract slide (x = PS4: Square)
                slidePosition = SLIDE_RETRACTED;
            }
            else if (gamepad1.a) {  // stop slide motor
                slidePosition = SLIDE_RETRACTED;
            }
       
            if (slidePosition > SLIDE_EXTENDED_MAX) {   // enforce maximum slide extension limit
                slidePosition = SLIDE_EXTENDED_MAX;
            }

            if (slidePosition < SLIDE_RETRACTED) {      // enforce slide retraction limit
                slidePosition = SLIDE_RETRACTED;
            }

            slideMotor.setTargetPosition((int) slidePosition);
            ((DcMotorEx)slideMotor).setVelocity(1200);   // Ri3D had setVelocity(2100);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Check to see if our arm is over the current (amp) limit, and report via telemetry.
            //if (((DcMotorEx) armMotor).isOverCurrent()) {
            //    telemetry.addLine("MOTOR EXCEEDED CURRENT (AMP) LIMIT!");
            //}


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
            //telemetry.addData("slide Motor Amps: ", ((DcMotorEx) slideMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

}