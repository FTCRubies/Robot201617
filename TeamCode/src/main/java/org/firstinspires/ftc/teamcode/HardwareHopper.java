package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is Hopper (RUBIES robot 2016-2017)
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareHopper
{
    /* Public OpMode members. */
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  leftBackMotor = null;
    public DcMotor  rightFrontMotor   = null;
    public DcMotor  rightBackMotor = null;

    public Servo ultrasonicServo = null;
    public CRServo pusherLeft = null;
    public CRServo pusherRight = null;

    public AnalogInput wallUltrasonic = null;
    public OpticalDistanceSensor lineSensor = null;
    public AHRS navx_device;
    public ModernRoboticsI2cColorSensor colorLeft= null;
    public ModernRoboticsI2cColorSensor colorRight= null;

    public final int NAVX_DIM_I2C_PORT = 5;


    public void setLeftPower(double leftPower){
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower (leftPower);
    }
    public void setRightPower(double rightPower){
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(rightPower);
    }
//    public void getLeftPower(){
//        leftFrontMotor.getPower();
//        leftBackMotor.getPower ();
//    }
//    public void getRightPower(){
//        rightFrontMotor.getPower();
//        rightBackMotor.getPower();
//    }


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareHopper(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = ahwMap.dcMotor.get("left_front"); //Name all of the motors the same as used as configuration on the phone
        leftBackMotor = ahwMap.dcMotor.get("left_back");
        rightFrontMotor = ahwMap.dcMotor.get("right_front");
        rightBackMotor = ahwMap.dcMotor.get("right_back");

        wallUltrasonic = ahwMap.analogInput.get("wall_ultrasonic");
        navx_device = AHRS.getInstance(ahwMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        lineSensor = ahwMap.opticalDistanceSensor.get("line_ODS");
        colorLeft = (ModernRoboticsI2cColorSensor) ahwMap.colorSensor.get("left_color");
        colorRight = (ModernRoboticsI2cColorSensor) ahwMap.colorSensor.get("right_color");


        ultrasonicServo = ahwMap.servo.get("ultrasonic_servo");
        pusherLeft = ahwMap.crservo.get("pusher_left");
        pusherRight = ahwMap.crservo.get("pusher_right");


//        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

//        colorLeft.setI2cAddress(new I2cAddr(0x3c));
//        colorRight.setI2cAddress(new I2cAddr(0x42));

        setLeftPower(0);
        setRightPower(0);
        ultrasonicServo.setPosition(0.5);
        pusherRight.setPower(0);
        pusherLeft.setPower(0);
        colorLeft.enableLed(true);
        colorRight.enableLed(true);
        lineSensor.enableLed(true);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

