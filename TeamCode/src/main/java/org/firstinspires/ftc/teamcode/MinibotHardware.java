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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

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
 // tensor flow to detect object start
 private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal; // end tensor flow declare
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


