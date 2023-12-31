/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="minibotEncoder", group="Sandra")
//@Disabled
public class MinibotEncoderAut extends LinearOpMode {
    private LinearOpMode runOpMode;
    // MinibotHardware robotAuto = null;  // uses Minibot Hardware
    MinibotHardware robotAuto = new MinibotHardware(runOpMode, telemetry);
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // private  telemetry = null;
    static final double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.5;
    private DcMotor leftDrive = null; //LEFT FRONT MOTOR
    private DcMotor rightDrive = null;// RIGHT FRONT MOTOR
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;

    //private TfodProcessor tfod;
    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robotAuto = new MinibotHardware(this, telemetry);

        robotAuto.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        int position = 1; // 0 is left, 1 is center, 2 is right, will be given from the camera code

        double offset = 0.0;
        double distance = 24; //inches


        while (opModeIsActive()) {
            if (position == 0) { // left
                robotAuto.encoderDrive(DRIVE_SPEED, 20, 20, 2.5);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(1000);
                robotAuto.encoderDrive(DRIVE_SPEED, -33, -33, 3.5);
                sleep(1000);
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.35);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(1000);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(1000);
                robotAuto.encoderDrive(DRIVE_SPEED, 3, 3, 2.0);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(1000);

            } else if (position == 1) { //center
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                robotAuto.encoderDrive(DRIVE_SPEED, 20, 20, 2.5);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, -19, -19, 2.5);  // S3: Reverse 24 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Right 12 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 21, 21, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.15);  // S2: Turn Right 12 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 30, 30, 2.0);
                sleep(500);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Right 12 Inches with 4 Sec timeout
                sleep(500);
                robotAuto.encoderDrive(DRIVE_SPEED, 4, 4, 1.05);
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, -4, -4, 1.05);
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 26, 26, 2.0);
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);
                robotAuto.encoderDrive(DRIVE_SPEED, -4, -4, 1.05);
                sleep(250);// park
            } else if (position == 2) { //right*/
                robotAuto.encoderDrive(DRIVE_SPEED, 20, 20, 2.5);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(150);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Right 12 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 2, 2, .75);
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.10);  // S2: Turn Right 12 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 2, 2, 1.0);
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.10);
                sleep(200);
                robotAuto.encoderDrive(DRIVE_SPEED, 30, 30, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(150);
                robotAuto.encoderDrive(TURN_SPEED, 12, -12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(DRIVE_SPEED, 24, 24, 2.0);  // S1: Forward 47 Inches with 5 Sec timeout
                sleep(250);
                robotAuto.encoderDrive(TURN_SPEED, -12, 12, 1.15);  // S2: Turn Left 12 Inches with 4 Sec timeout
                sleep(1000);// park
            }
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(30000);  // pause to display final telemetry message.

 /* while (opModeIsActive() &&
            (runtime.seconds() < timeoutS) &&
            (leftDrive.isBusy() && rightDrive.isBusy())) {

      // Display it for the driver.
      telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
      telemetry.addData("Currently at", " at %7d :%7d",
              leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
      telemetry.update();
  }
        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.*/
    }
}





