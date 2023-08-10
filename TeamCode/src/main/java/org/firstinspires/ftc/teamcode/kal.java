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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="test", group="Iterative Opmode")
public class kal extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor handRight = null;
    private DcMotor handLeft = null;
    private DcMotor mazafak1 = null;
    private DcMotor mazafak2 = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftfrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftbackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        handLeft = hardwareMap.get(DcMotor.class, "handLeft");
        handRight = hardwareMap.get(DcMotor.class,"handRight");
        handLeft.setPower(0.);
        handRight.setPower(0.);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        handRight.setDirection(DcMotor.Direction.REVERSE);
        handLeft.setDirection(DcMotor.Direction.FORWARD);
        mazafak1.setDirection(DcMotor.Direction.FORWARD);
        mazafak2.setDirection(DcMotor.Direction.REVERSE);

        handLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        handRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        handRight.setTargetPosition(0);
        handLeft.setTargetPosition(0);

        handRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        handLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        mazafak1.setPower(gamepad1.right_trigger);
        mazafak2.setPower(gamepad1.right_trigger);

        if (gamepad1.y){
            handRight.setTargetPosition(700);
            handLeft.setTargetPosition(700);
            handLeft.setPower(0.7);
            handRight.setPower(0.7);
        }
        if (gamepad1.a){
            handRight.setTargetPosition(100);
            handLeft.setTargetPosition(100);
            handRight.setPower(0.3);
            handLeft.setPower(0.3);
        }

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
//        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
         leftPower  = -gamepad1.left_stick_y ;
         rightPower = -gamepad1.right_stick_y ;


        // Send calculated power to wheels
        leftfrontDrive.setPower(leftPower);
        rightfrontDrive.setPower(rightPower);
        leftbackDrive.setPower(leftPower);
        rightbackDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
