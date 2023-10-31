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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class MinibotHardware {
    private  Telemetry telemetry;  //HardwareMinibot robot = new HardwareMinibot(); // uses Minibot Hardware

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftDrive   = null; //LEFT FRONT MOTOR
    private DcMotor rightDrive  = null;// RIGHT FRONT MOTOR
    private Servo   oneHand = null; // servo restrain

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
   // TouchSensor touchSensor;  // Touch sensor Object
    //NormalizedColorSensor colorSensor;
    //View relativeLayout;
    //HardwareMap hwMap = null;
    private ElapsedTime runtime;
      // Define a constructor that allows the OpMode to pass a reference to itself.
    public MinibotHardware(LinearOpMode opMode, Telemetry telemetry) {
        myOpMode= opMode;
        this.telemetry = telemetry;
        runtime = new ElapsedTime();
    }
       /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() { //sandra


        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        // set all motors to zero power  maybe commented if not needed Sandra
        leftDrive.setPower(0);
        rightDrive.setPower(0);



        //setDrivePowers(0); sandra

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // may need to switch to without Sandra
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);// may need to switch to without Sandra


        // Define and initialize ALL installed servos.
        oneHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        oneHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

      /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */

    public void  setHandPositions(double offset) {
            offset = Range.clip(offset, -0.5, 0.5);
            oneHand.setPosition(MID_SERVO + offset);

    }

     public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

         int newLeftTarget;
         int newRightTarget;

         // Ensure that the OpMode is still active
         if (myOpMode.opModeIsActive()) {

             // Determine new target position, and pass to motor controller
             newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
             newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
             leftDrive.setTargetPosition(newLeftTarget);
             rightDrive.setTargetPosition(newRightTarget);

             // Turn On RUN_TO_POSITION
             leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             // reset the timeout time and start motion.
             runtime.reset();
             leftDrive.setPower(Math.abs(speed));
             rightDrive.setPower(Math.abs(speed));

             // keep looping while we are still active, and there is time left, and both motors are running.
             // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             // its target position, the motion will stop.  This is "safer" in the event that the robot will
             // always end the motion as soon as possible.
             // However, if you require that BOTH motors have finished their moves before the robot continues
             // onto the next step, use (isBusy() || isBusy()) in the loop test.
             while ((myOpMode.opModeIsActive() &&
                     (runtime.seconds() < timeoutS) &&
                     (leftDrive.isBusy() && rightDrive.isBusy()))) {

                 // Display it for the driver.
                 myOpMode.telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                 myOpMode.telemetry.addData("Currently at", " at %7d :%7d",
                         leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                 myOpMode.telemetry.update();
             }

             // Stop all motion;
             leftDrive.setPower(0);
             rightDrive.setPower(0);

             // Turn off RUN_TO_POSITION
             leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


             myOpMode.telemetry.addData("Path", "Complete");
             myOpMode.telemetry.update();

             //sleep(250); // optional pause after each move.*/
         }
     }
}

    /*int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity)hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
        runSample(); // actually execute the sample
    } finally {
        // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
        // as pure white, but it's too much work to dig out what actually was used, and this is good
        // enough to at least make the screen reasonable again.
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}

    protected void runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed.
       // waitForStart();

        // Loop until we are asked to stop
        while (myOpMode.opModeIsActive()) {
            // Explain basic gain information via telemetry
            myOpMode.telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            myOpMode.telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

           /* // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }

            // Show the gain value via telemetry
            myOpMode.telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.setGain(gain);
            double drive = -gamepad1.left_stick_y; //
            double turn  = -gamepad1.right_stick_x;
            // Check the status of the X button on the gamepad
            xButtonCurrentlyPressed = gamepad1.right_stick_x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight) colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color.

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

            myOpMode.telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            myOpMode.telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            myOpMode.telemetry.addData("Alpha", "%.3f", colors.alpha);

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity.
            if (colorSensor instanceof DistanceSensor) {
                myOpMode.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            myOpMode.telemetry.update();

            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues)); */





