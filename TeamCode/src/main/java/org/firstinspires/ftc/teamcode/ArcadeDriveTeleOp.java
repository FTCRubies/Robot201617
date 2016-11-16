/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Arcade Drive", group="TeleOp")
public class ArcadeDriveTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareHopper robot       = new HardwareHopper(); // use the class created to define a Hardware Hopper


    private ElapsedTime runtime = new ElapsedTime();

    double    INCREASE_VALUE  = 2.0 ;                  // Increase from base speed when "turbo" button pushed
    double    DECREASE_VALUE = 0.5;                 // Decrease from base speed when "slow" button pushed

    private boolean calibration_complete = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Initializing");
        updateTelemetry(telemetry);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double directionHorizontal;
        double directionVertical;
        double speedMultiplier;
        double slowValue = 0.1;
        double changeValue;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        directionHorizontal = -gamepad1.right_stick_y;
        directionVertical = -gamepad1.right_stick_x;
        speedMultiplier = ((1 - slowValue)/2) * (-gamepad1.left_stick_y-1) + 1;

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.left_trigger > 0){
            changeValue = 2;
            telemetry.addData("Turbo Button", "Pressed");
        } else if (gamepad1.right_trigger > 0) {
            changeValue = 0.5;
            telemetry.addData("Slowing Button", "Pressed");
        } else {
            changeValue = 1;
            telemetry.addData ("Buttons Pressed", "None");
        }




        double leftPower = ((directionHorizontal + directionVertical) * speedMultiplier * changeValue);
        double rightPower = ((directionHorizontal - directionVertical) * speedMultiplier * changeValue);

        leftPower = Range.clip(Math.abs(leftPower), 0.1, 1); // absolute value





       robot.setLeftPower(leftPower);
       robot.setRightPower(rightPower);

        // Send telemetry message to signify robot running;
        telemetry.addData("Left Power",  "Power = %f", leftPower);
        telemetry.addData("Right Power", "Power = %f", rightPower);
        telemetry.addData("Speed Multiplier", "Multiplier = %f", speedMultiplier);
        telemetry.addData("Slowing button", "Value = %f", gamepad1.right_trigger);
        telemetry.addData("Turbo button", "Value = %f", gamepad1.left_trigger);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
