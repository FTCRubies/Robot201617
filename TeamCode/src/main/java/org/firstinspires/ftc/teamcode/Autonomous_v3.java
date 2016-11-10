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
import com.qualcomm.robotcore.util.Range;



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

public class Autonomous_v3 extends LinearOpMode {

    HardwareHopper robot = new HardwareHopper();

    private enum State { //List all of the states that will be used in this program (as seen in our state diagram)
        STATE_TEST,
        STATE_GYRO_DRIVE,
        STATE_TURN,
        STATE_WALL_FOLLOW,
        STATE_WALL_FOLLOW_ONE,
        STATE_BACKUP,
        STATE_READ_BEACON,
        STATE_FAR_BUTTON,
        STATE_CLOSE_BUTTON,
        STATE_PUSHER_IN,
        STATE_STOP
    }

    // Names constants
    final double BASE_SPEED = 0.2;     // Speed used in the majority of autonomous
    final double CORRECTION_SENSITIVITY = 20.0;
    double TARGET_DISTANCE = 5.0;      //Distance that we want to be from the wall
    double RADIUS = 150.0;       //Turning radius for wall-following corrections

    private State currentState;     //State in state machine that is currently running
//    private double heading;
//    private double distance;

    //Name what we will be using for run time and state time
    private ElapsedTime runtime = new ElapsedTime();        //Overall time in program
    private ElapsedTime stateTime = new ElapsedTime();      //Time that the program has been in a given state

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);        //Connect and initialize all of the hardware in the hardware map

        telemetry.addData(">", "Gyro Calibrating. Do Not move!"); //Calibrate the gyro and display telemetry
        telemetry.update();

        // make sure the gyro is calibrated.
        while (robot.navx_device.isCalibrating())  {      //while either gyro sensor is calibrating, wait
           // Thread.sleep(50);
            idle();
        }

        robot.navx_device.zeroYaw();        //zero the Yaw on the navX gyro
        boolean firstBeaconPushed = false;

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");      //Update telemetry signalling that the gyro has calibrated
        telemetry.update();


        //Set the drive motor directions:

        //Set servo positions:
        robot.ultrasonicServo.setPosition(.75);
        robot.pusherLeft.setPower(0);
        robot.pusherRight.setPower(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Set initial state
        newState(State.STATE_GYRO_DRIVE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Telemetry that will be displayed throughout the match on the drivers station phone:
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "State Time: " + stateTime.toString());
            telemetry.addData("Power", "Left Power: " + robot.leftFrontMotor.getPower());
            telemetry.addData("Power", "Right Power: " + robot.rightFrontMotor.getPower());
            telemetry.addData("Current State", "State: " + currentState);
            telemetry.addData("Position", "Servo Position: " + robot.ultrasonicServo.getPosition());
            telemetry.addData ("Ultrasonic", "Reading: " + robot.wallUltrasonic.getVoltage()/5.0);
            telemetry.addData("NavXGyro", "Current Heading" + robot.navx_device.getFusedHeading());
            telemetry.addData("ODS", "Line Sensor: " + robot.lineSensor.getLightDetected());
            telemetry.addData("Ultrasonic", "Target Distance: " + (TARGET_DISTANCE / 100) + ((RADIUS*(1-(1/Math.sqrt(2))))/100));
            telemetry.update();


            switch (currentState){
                case STATE_GYRO_DRIVE:
                    if (robot.wallUltrasonic.getVoltage()/5 <= 0.7){    //Correct distance to begin curve
                        newState(State.STATE_WALL_FOLLOW);     //Start next state
                    } else {
                        //setDrivePower(BASE_SPEED, BASE_SPEED);
                        setTargetAngle(-45);    //Drive using the navX gyro 45 degrees left
                    }
                    break;

//                case STATE_WALL_FOLLOW_ONE:
//                    double currentAngle;
//                    if(robot.lineSensor.getLightDetected()>= 0.02) { setDrivePower(-BASE_SPEED, -BASE_SPEED);
//                        telemetry.addData("Weird", "Weird,");
//                        setDrivePower(0,0);     //Stop the robot
//                        robot.ultrasonicServo.setPosition(0.5);     //Point servo directly out
//                        newState(State.STATE_BACKUP);     //Start next state
//                    }
//                    else {
//                        double distance = robot.wallUltrasonic.getVoltage() / 5.0;    //Distance in meters
//                        double distanceError = distance - TARGET_DISTANCE / 100.0;
//                        telemetry.addData("Distance", "Distance Error" + distanceError);
//                        if (distanceError >= 0) {
//                            currentAngle = Math.toDegrees(Math.acos(((RADIUS / 100.0) - distanceError) / (RADIUS / 100.0)));
//                            setTargetAngle(currentAngle);
//                            setDrivePower(-BASE_SPEED, -BASE_SPEED);
//                        } else{
//                            currentAngle = Math.toDegrees(Math.acos(((RADIUS / 100.0) + distanceError) / (RADIUS / 100.0)));     //Calculate the angle that the robot needs to be pointed at
//                        setTargetAngle(+currentAngle);      //Sets that value as our target Angle
//                    }
//                        followWall();
//
//                    }
//                    break;

                case STATE_WALL_FOLLOW:
                    double currentAngle;    //Angle that the robot needs to be pointed at to complete the curve
                    if(robot.lineSensor.getLightDetected()>= 0.02){     //White line is detected
                        telemetry.addData("Weird", "Weird,");
//                        setDrivePower(0,0);     //Stop the robot
//                        robot.ultrasonicServo.setPosition(0.5);     //Point servo directly out
//                        newState(State.STATE_BACKUP);     //Start next state
                } else {
                        double distance = robot.wallUltrasonic.getVoltage()/5.0;    //Distance in meters
                        double distanceError = distance - TARGET_DISTANCE/100.0;    //Error between where we are and where we need to be
                        telemetry.addData("Distance", "Distance Error" + distanceError);    //Display telemetry for the distanceError

                        if (distanceError >= 0) {   //If we are too far from the wall
                            currentAngle = Math.toDegrees(Math.acos(((RADIUS/100.0) - distanceError) / (RADIUS/100.0)));     //Calculate the angle that the robot needs to be pointed at
                            setTargetAngle(-currentAngle);      //Sets that values as our Target Angle
                        } else {    //otherwise we must be too close to wall
                            currentAngle = Math.toDegrees(Math.acos(((RADIUS/100.0) + distanceError) / (RADIUS/100.0)));     //Calculate the angle that the robot needs to be pointed at
                            setTargetAngle(+currentAngle/2.0);      //Sets that value as our target Angle
                        }
                        followWall();


                       // telemetry.addData("Angle", "Current Angle" + currentAngle);     //Displays that angle as telemetry
                    }
                    break;

                case STATE_BACKUP:
                    if(robot.lineSensor.getLightDetected()>= 0.5){
                        setDrivePower(0,0);
                        newState(State.STATE_STOP);
                } else {
                        setDrivePower(-0.12, -0.12);  //Back up slowly
                    }
                    break;

//                case STATE_READ_BEACON:
//                    if (robot.colorLeft.red() >= robot.colorLeft.blue()){
//                        robot.pusherLeft.setPower(BASE_SPEED);
//                        newState(State.STATE_PUSHER_IN);
//                    } else if (robot.colorRight.red() >= robot.colorRight.blue()){
//                        robot.pusherRight.setPower(BASE_SPEED);
//                        newState(State.STATE_PUSHER_IN);
//                    }
//                        break;
//
//                case STATE_PUSHER_IN:
//                    if ((robot.colorLeft.red() >= robot.colorLeft.blue() && robot.colorLeft.red() >= robot.colorLeft.blue())){
//                        robot.pusherLeft.setPower(-BASE_SPEED);
//                        if (firstBeaconPushed == false){
//                            firstBeaconPushed = !firstBeaconPushed;
//                            newState(State.STATE_WALL_FOLLOW);
//                        } else {
//                            firstBeaconPushed = !firstBeaconPushed;
//                            newState(State.STATE_STOP);
//                        }
//                    } else {
//                        sleep(50);
//                    }
//                    break;

                case STATE_STOP:
                    setDrivePower(0,0);     //Keeps the robot stopped
                    break;
                    }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        }

    private void newState(State newState){
        stateTime.reset();      //Resets State Time
        currentState = newState;        //Changes the state
    }
    void setDrivePower(double leftPower, double rightPower) {       //Sets the power of both sides of the robot
        robot.setLeftPower(leftPower);      //Sets the motors on the left side to an inputted power
        robot.setRightPower(rightPower);        //Sets the motors on the right side to an inputted power
    }
    void setTargetAngle(double inputAngle){    //Uses the navX sensor and given information to calculate the wheel power necessary to be at a certain angle
        double gyroDifference = inputAngle - robot.navx_device.getFusedHeading();       //Calculates the difference between the inputted angle and the current angle

        //Calculates the shortest way to get to the angle
        while (gyroDifference >= 180){
            gyroDifference = gyroDifference - 360;
        }
        while (gyroDifference <= -180){
            gyroDifference = gyroDifference + 360;
        }
        double finalGyroDifference;

        //Limits the gyro from -45 to +45
        if (gyroDifference < 0) {
            finalGyroDifference = Math.max(-45.0, gyroDifference);
        } else {
           finalGyroDifference = Math.min(45.0, gyroDifference);
        }

        setDrivePower(BASE_SPEED *(1 + finalGyroDifference/CORRECTION_SENSITIVITY), BASE_SPEED * (1 - finalGyroDifference/CORRECTION_SENSITIVITY));      //Sets power to change the angle accordingly

        telemetry.addData("Left Motor", "Setting to" + Double.toString(BASE_SPEED *(1 + finalGyroDifference/45.0)));
        telemetry.addData("Right Motor", "Setting to" + Double.toString(BASE_SPEED * (1 - finalGyroDifference/45.0)));
        telemetry.addData("Angle", "Going to" + finalGyroDifference);
    }
    void followWall(){
        double integratedHeading = robot.navx_device.getFusedHeading();


        while (integratedHeading > 180){
           integratedHeading = integratedHeading - 360;
        }
        while (integratedHeading < -180){
           integratedHeading = integratedHeading + 360;
        }

        robot.ultrasonicServo.setPosition((-integratedHeading / 180.0) + 0.5);     //Sets servo position to point towards wall
    }
}

