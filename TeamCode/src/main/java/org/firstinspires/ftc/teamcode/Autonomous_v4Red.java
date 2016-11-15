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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Red", group="Linear Autonomous")

public class Autonomous_v4Red extends LinearOpMode {

    HardwareHopper robot = new HardwareHopper();

    private enum State { //List all of the states that will be used in this program (as seen in our state diagram)
        STATE_TEST,
        STATE_GYRO_DRIVE,
        STATE_WALL_FOLLOW,
        STATE_BACKUP,
        STATE_READ_BEACON,
        STATE_PUSHER_IN,
        STATE_WALL_FOLLOW_BACK,
        STATE_FORWARD,
        STATE_FAR_BUTTON,
        STATE_CLOSE_BUTTON,
        STATE_STOP
    }
    private enum AllianceColor {
        RED,
        BLUE
    }

    // Names constants
    final double BASE_SPEED = 0.3;    // Speed used in the majority of autonomous
    final double CORRECTION_SENSITIVITY = 20.0;
    double TARGET_DISTANCE = 6.0;      //Distance that we want to be from the wall
    double LINE_THRESHOLD = 0.15;
    double SAFETY_DISTANCE = 35.0;

    private State currentState;     //State in state machine that is currently running

    //Name what we will be using for run time and state time
    private ElapsedTime runtime = new ElapsedTime();        //Overall time in program
    private ElapsedTime stateTime = new ElapsedTime();      //Time that the program has been in a given state
    boolean pastFirstLine = false;
    AllianceColor currentAlliance = AllianceColor.RED;

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
        if (currentAlliance == AllianceColor.BLUE){
            robot.ultrasonicServo.setPosition(.75);
        } else {
            robot.ultrasonicServo.setPosition(.25);
        }
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
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Status", "State Time: " + stateTime.toString());
            telemetry.addData("Power", "Left Power: " + robot.leftFrontMotor.getPower());
            telemetry.addData("Power", "Right Power: " + robot.rightFrontMotor.getPower());
            telemetry.addData("Current State", "State: " + currentState);
            telemetry.addData("Color Sensor", "Red on right: " + robot.colorRight.red());
            telemetry.addData("Color Sensor", "Blue on right: " + robot.colorRight.blue());
            telemetry.addData("Color Sensor", "Red on left: " + robot.colorLeft.red());
            telemetry.addData("Color Sensor", "Blue on left: " + robot.colorLeft.blue());
//            telemetry.addData("Position", "Servo Position: " + robot.ultrasonicServo.getPosition());
//            telemetry.addData ("Ultrasonic", "Reading: " + robot.wallUltrasonic.getVoltage()/5.0);
            telemetry.addData("NavXGyro", "Current Heading" + robot.navx_device.getFusedHeading());
            telemetry.addData("ODS", "Line Sensor: " + robot.lineSensor.getLightDetected());
//            telemetry.addData("Ultrasonic", "Target Distance: " + (TARGET_DISTANCE / 100) + ((RADIUS*(1-(1/Math.sqrt(2))))/100));
            telemetry.update();


            switch (currentState){
                case STATE_GYRO_DRIVE:
                    if (robot.wallUltrasonic.getVoltage()/5 <= 0.5){    //Correct distance to begin curve
                        //setDrivePower(BASE_SPEED,-0.1);
                        newState(State.STATE_WALL_FOLLOW);     //Start next state
                    } else {
                        //setDrivePower(BASE_SPEED, BASE_SPEED);
                        setTargetAngle(currentAlliance == AllianceColor.RED ? -45 : 45);    //Drive using the navX gyro 45 degrees left
                    }
                    break;

                case STATE_WALL_FOLLOW:
                    double currentAngle1;    //Angle that the robot needs to be pointed at to complete the curve
                    if(robot.lineSensor.getLightDetected()>= 0.1){     //White line is detected
                        if (pastFirstLine == true){
                            if (currentAlliance == AllianceColor.RED) {
                                sleep(100);
                                setDrivePower(-0.15,-0.15);
                            } else {
                                sleep(100);
                                setDrivePower(0.15, 0.15);
                            }
                            newState(State.STATE_BACKUP);

                        } else {

                            while (robot.lineSensor.getLightDetected() >= 0.1){
                                idle();

                            }
                            pastFirstLine = true;

                        }
//                        setDrivePower(0,0);     //Stop the robot
//                        robot.ultrasonicServo.setPosition(0.5);     //Point servo directly out
//                        newState(State.STATE_BACKUP);     //Start next state
                } else {
                        double distance = robot.wallUltrasonic.getVoltage()/5.0;    //Distance in meters
                        double distanceError = distance - TARGET_DISTANCE/100.0;    //Error between where we are and where we need to be
//                        double angleTo = Range.clip(distanceError/(0.2159 + SAFETY_DISTANCE/100), -1, 1 );
//                        telemetry.addData("Distance", "Distance Error" + distanceError);    //Display telemetry for the distanceError

                        double sideRatio = Range.clip(distanceError/(0.2159 + SAFETY_DISTANCE/100), -1, 1);
                        currentAngle1 = Math.toDegrees(Math.asin(sideRatio));

                        setTargetAngle(Range.clip(currentAlliance == AllianceColor.RED ? -currentAngle1 : currentAngle1, -45, 45));
                        pointServoToWall();

                       // telemetry.addData("Angle", "Current Angle" + currentAngle);     //Displays that angle as telemetry
                    }
                    break;

                case STATE_BACKUP:
                    if(robot.lineSensor.getLightDetected()>= LINE_THRESHOLD) {
                        setDrivePower(0, 0);
                        newState(State.STATE_READ_BEACON);
                    }
                    break;

                case STATE_READ_BEACON:
                    if (currentAlliance == AllianceColor.RED) {
                        if (robot.colorLeft.red() > robot.colorLeft.blue()) {
                            telemetry.addData("Pushing button", "Left");
                            robot.pusherLeft.setPower(-1);
                            newState(State.STATE_PUSHER_IN);
                        } else if (robot.colorRight.red() > robot.colorRight.blue()) {
                            telemetry.addData("Pushing button", "Right");
                            robot.pusherRight.setPower(1);
                            newState(State.STATE_PUSHER_IN);
                        }
                    } else {
                        if (robot.colorLeft.blue() > robot.colorLeft.red()) {
                            telemetry.addData("Pushing button", "Right");
                            robot.pusherLeft.setPower(-1);
                            newState(State.STATE_PUSHER_IN);
                        } else if (robot.colorRight.blue() > robot.colorRight.red()) {
                            telemetry.addData("Pushing button", "Left");
                            robot.pusherRight.setPower(1);
                            newState(State.STATE_PUSHER_IN);
                        }
                    }
                        break;

                case STATE_PUSHER_IN:
                    if (currentAlliance == AllianceColor.RED && (robot.colorLeft.red() >= robot.colorLeft.blue() && robot.colorRight.red() >= robot.colorRight.blue())
                    || currentAlliance == AllianceColor.BLUE && (robot.colorLeft.blue() >= robot.colorLeft.red() && robot.colorRight.blue() >= robot.colorRight.red())
                            || stateTime.seconds() >= 2){
                        if (robot.pusherRight.getPower() != 0){
                            robot.pusherRight.setPower(-1);
                        } else {
                            robot.pusherLeft.setPower(1);
                        }
                        sleep(500);

                        if (firstBeaconPushed == false){
                            firstBeaconPushed = true;
                            newState(State.STATE_WALL_FOLLOW_BACK);
                        } else {
                            newState(State.STATE_STOP);
                        }
                    }
                    break;
                case STATE_WALL_FOLLOW_BACK:
                    double currentAngle2;    //Angle that the robot needs to be pointed at to complete the curve
                    if(robot.lineSensor.getLightDetected()>= 0.1 && stateTime.seconds() >= 0.5){     //White line is detected
                        sleep(100);
                        if (currentAlliance == AllianceColor.RED) {
                            setDrivePower(0.15, 0.15);
                        } else {
                            setDrivePower(-0.15, -0.15);
                        }
                        newState(State.STATE_FORWARD);

                    } else {
                        double distance = robot.wallUltrasonic.getVoltage()/5.0;    //Distance in meters
                        double distanceError = distance - TARGET_DISTANCE/100.0;    //Error between where we are and where we need to be
//                        telemetry.addData("Distance", "Distance Error" + distanceError);    //Display telemetry for the distanceError
                        currentAngle2 = Math.toDegrees(Math.asin(distanceError/(0.2159 + SAFETY_DISTANCE/100)));
                        setTargetAngleBackwards(Range.clip(currentAlliance == AllianceColor.RED ? currentAngle2 : -currentAngle2, -45, 45));
                        pointServoToWall();
                    }
                    break;

                case STATE_FORWARD:
                    if(robot.lineSensor.getLightDetected()>= LINE_THRESHOLD) {
                        setDrivePower(0, 0);
                        newState(State.STATE_READ_BEACON);
                    }
                    break;

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
        double setLeft = Range.clip(leftPower, -1, 1);
        double setRight = Range.clip(rightPower, -1, 1);
        robot.setLeftPower(setLeft);      //Sets the motors on the left side to an inputted power
        robot.setRightPower(setRight);       //Sets the motors on the right side to an inputted power
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

        if (currentAlliance == AllianceColor.RED){
            setDrivePower(BASE_SPEED *(1 + finalGyroDifference/CORRECTION_SENSITIVITY), BASE_SPEED * (1 - finalGyroDifference/CORRECTION_SENSITIVITY));      //Sets power to change the angle accordingly
        } else {
            setDrivePower(-BASE_SPEED *(1 - finalGyroDifference/CORRECTION_SENSITIVITY), -BASE_SPEED * (1 + finalGyroDifference/CORRECTION_SENSITIVITY));      //Sets power to change the angle accordingly
        }

        telemetry.addData("Left Motor", "Setting to" + Double.toString(BASE_SPEED *(1 + finalGyroDifference/45.0)));
        telemetry.addData("Right Motor", "Setting to" + Double.toString(BASE_SPEED * (1 - finalGyroDifference/45.0)));
        telemetry.addData("Angle", "Going to" + finalGyroDifference);
    }
    void setTargetAngleBackwards(double inputAngle){    //Uses the navX sensor and given information to calculate the wheel power necessary to be at a certain angle
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

        if (currentAlliance == AllianceColor.RED) {
            setDrivePower(-BASE_SPEED * (1 - finalGyroDifference / CORRECTION_SENSITIVITY), -BASE_SPEED * (1 + finalGyroDifference / CORRECTION_SENSITIVITY));      //Sets power to change the angle accordingly
        } else {
            setDrivePower(BASE_SPEED * (1 + finalGyroDifference / CORRECTION_SENSITIVITY), BASE_SPEED * (1 - finalGyroDifference / CORRECTION_SENSITIVITY));      //Sets power to change the angle accordingly
        }

        telemetry.addData("Left Motor", "Setting to" + Double.toString(BASE_SPEED *(1 + finalGyroDifference/45.0)));
        telemetry.addData("Right Motor", "Setting to" + Double.toString(BASE_SPEED * (1 - finalGyroDifference/45.0)));
        telemetry.addData("Angle", "Going to" + finalGyroDifference);
    }
    void pointServoToWall(){
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

