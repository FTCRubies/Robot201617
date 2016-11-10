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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous", group="Linear Autonomous")
@Disabled
public class Autonomous_v2 extends LinearOpMode {

    HardwareHopper robot = new HardwareHopper();

    private enum State { //List all of the states that will be used in this program (as seen in our state diagram)
        STATE_GYRO_DRIVE,
        STATE_TURN,
        STATE_WALL_FOLLOW,
        STATE_READ_BEACON,
        STATE_FAR_BUTTON,
        STATE_CLOSE_BUTTON,
        STATE_PUSHER_IN,
        STATE_STOP
    }

    final double BASE_SPEED = 0.15; // Name variables, such as the base turn and base speed
    final int BASE_TURN = 45;
    final double CORRECTION = 1;

    private State currentState; //Name terms we will be using involving states
    private double heading;
    private double distance;


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime(); //Name what we will be using for run time and state time
    private ElapsedTime stateTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!"); //Calibrate the gyro and display telemetry
        telemetry.update();
          //robot.gyro.calibrate();

        // make sure the gyro is calibrated.
//        while (robot.gyro.isCalibrating() || robot.navx_device.isCalibrating())  {
//            Thread.sleep(50);
//            idle();
//        }

        robot.navx_device.zeroYaw();

        telemetry.addData(">", "Gyros Calibrated.  Press Start."); //Update telemetry involving the gyro sensor
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // eg: Set the drive motor directions:
        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        robot.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        heading = 45 * CORRECTION;
        distance = 25;

        robot.ultrasonicServo.setPosition(.75);
        robot.pusherLeft.setPower(0);
        robot.pusherRight.setPower(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        newState(State.STATE_GYRO_DRIVE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString()); //Telemetry that will be displayed throughout the match on the drivers station phone
            telemetry.addData("Status", "State Time: " + stateTime.toString());
            telemetry.addData("Current State", "State: " + currentState);
            telemetry.addData("ODS", "Line Sensor: " + robot.lineSensor.getLightDetected());
            telemetry.addData ("Ultrasonic", "Reading: " + robot.wallUltrasonic.getVoltage()/5.0);
            telemetry.addData("Ultrasonic", "Distance Difference: " + Double.toString(distance/100 - (robot.wallUltrasonic.getVoltage()/5)));
            //telemetry.addData("MR Gyro", "Current Heading: " + robot.gyro.getIntegratedZValue());
            telemetry.addData("MR Gyro", "Heading goal: " + heading);
            telemetry.addData("NavXGyro", "Current Heading" + robot.navx_device.getFusedHeading());
            telemetry.addData("Power", "Left Power: " + robot.leftFrontMotor.getPower());
            telemetry.addData("Power", "Right Power: " + robot.rightFrontMotor.getPower());
            telemetry.addData("Position", "Servo Position: " + robot.ultrasonicServo.getPosition());
            telemetry.update();


            switch (currentState){
                case STATE_GYRO_DRIVE:
                    if (robot.wallUltrasonic.getVoltage()/5.0 <= 0.25){
                        setDrivePower(BASE_SPEED, 0);
                        heading = 0 * CORRECTION;
                        newState(State.STATE_WALL_FOLLOW);
                    } else {
                        setTargetAngle(-45.0);


//                        robot.ultrasonicServo.setPosition(0.51);

//                        telemetry.addData("Gyro", "Current Heading: " + robot.gyro.getIntegratedZValue());
//                        telemetry.addData("Gyro", "Heading goal: " + heading);
//                        telemetry.addData("Power", "Left Power: " + robot.leftFrontMotor.getPower());
//                        telemetry.addData("Power", "Right Power: " + robot.rightFrontMotor.getPower());
//                        telemetry.update();
                    }
                    break;

                case STATE_TURN:
                    if(robot.navx_device.getFusedHeading() <= heading){
                        setDrivePower(0,0);
                        robot.ultrasonicServo.setPosition(0.5);
                        newState(State.STATE_WALL_FOLLOW);

                        //robot.ultrasonicServo.setPosition(115);
                        //newState(State.STATE_WALL_FOLLOW);
                } else {
                        setDrivePower(BASE_SPEED,0);
                        //telemetry.addData("Gyro", "Current Heading: ", + robot.gyro.getIntegratedZValue());
                        telemetry.addData("Gyro", "Heading goal: ", + heading);
                        telemetry.addData("Power", "Left Power: ", robot.leftFrontMotor.getPower());
                        telemetry.addData("Power", "Right Power: ", robot.rightFrontMotor.getPower());
                    }
                    break;

                case STATE_WALL_FOLLOW:
                    if( robot.lineSensor.getLightDetected()>= 0.02){
                        setDrivePower(0,0);
                        newState(State.STATE_STOP);



                } else {
                        setTargetAngle(0);
//                        double distanceDifference = distance/100 - (robot.wallUltrasonic.getVoltage()/5);
//                        setDrivePower(BASE_SPEED + distanceDifference/2, BASE_SPEED - distanceDifference/2);
                    }
                    break;
                case STATE_STOP:
                    setDrivePower(0,0);

                    }



            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        }

    private void newState(State newState){
        stateTime.reset();      //resets State Time
        currentState = newState;        //changes the state
    }
    void setDrivePower(double rightPower, double leftPower) {
        robot.setLeftPower(leftPower);
        robot.setRightPower(rightPower);
    }
    void setTargetAngle(double inputAngle){
        double gyroDifference = inputAngle - robot.navx_device.getFusedHeading();
        while (gyroDifference >= 180){
            gyroDifference = gyroDifference - 360;
        }
        while (gyroDifference <= -180){
            gyroDifference = gyroDifference + 360;
        }
        setDrivePower(BASE_SPEED + gyroDifference/150, BASE_SPEED - gyroDifference/150);
//        if (inputAngle >= 0){
//           double targetAngle = inputAngle;
//            gyroDifference = targetAngle - robot.navx_device.getFusedHeading();
//
//            setDrivePower(BASE_SPEED - gyroDifference/100, BASE_SPEED + gyroDifference/100);
//        } else {
//            double targetAngle = 360 + inputAngle;
//            gyroDifference = targetAngle - (360 + robot.navx_device.getFusedHeading());
//
//        }
    }
}

