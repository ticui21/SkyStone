package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Vicki - Foundation Autonomous", group = "Vicki")
public class VickiFoundationAutonomous extends LinearOpMode {

    //Define DcMotors variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //Define servo variables (used to pick up foundation)
    Servo leftServo;
    Servo rightServo;

    //a time variable that I will use when the robot goes and comes back from its original spot to where the foundation is
    //this variable will change depending on how far or close the foundation is to the robot:
    int _timeToFoundation = 3000;

    /*
        NOTE:
        * I set the default time to 3 seconds (3000 milliseconds)
        * That is not accurate because I do not know how far the foundation is.
        * time will vary depending how far/close it is to the robot
        * this variable will need to be changed to fit the situation.
        * encoders will help create a more accurate code in terms of how long the motors will be on
    */

    @Override
    public void runOpMode() {
        //pair our DcMotor variables with the corresponding parts using hardwareMap
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");

        //same with the servos
        leftServo = hardwareMap.get(Servo.class, "left");
        rightServo = hardwareMap.get(Servo.class, "right");


        //Set left motors on reverse:
        //"Most robots need the motor on one side to be reversed to drive forward" (FTC BasicOpMode_Linear)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //wait for start:
        waitForStart();


        //First, ensure servos are in their original position:
        leftServo.setPosition(0);
        rightServo.setPosition(0);

        //will drive forward (towards foundation) at a power of 25%
        frontLeft.setPower(.25);
        frontRight.setPower(.25);
        backLeft.setPower(.25);
        backRight.setPower(.25);
        sleep(_timeToFoundation);

        //stops motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //wait 1 sec for next command
        sleep(1000);

        //Servos will turn to grab the foundation
        leftServo.setPosition(1);
        rightServo.setPosition(1);

        //Set the motors to reverse to pull back the foundation into its original position:
        frontLeft.setPower(-.25);
        frontRight.setPower(-.25);
        backLeft.setPower(-.25);
        backRight.setPower(-.25);
        sleep(_timeToFoundation);

        //stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //End of program

    }
}