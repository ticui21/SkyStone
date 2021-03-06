/* Copyright (c) 2020, All Rights Reserved
 *
 * This file is intended for FTC Team #8696 Trobotix only. Redistribution, duplication, or use in
 * source and binary forms, with or without modification, is not permitted without explicit
 * permission from the creator or authorized moderator.
 *
 * Written by Timothy (Tikki) Cui
 */

package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * 'Drivetrain' controls the motion motors of 'Trobot'. It contains the drive motor variables as
 * well as containing many utility methods.
 *
 * Key features include:
 * - driving with power inputs
 * - strafing
 * - auto-drive using encoders
 * - speed reduction
 * - checking motor statuses
 *
 * 'Drivetrain' remains relatively constant through each season since a different game theme doesn't
 * affect a robot's basic movement abilities.
 *
 * @author Tikki
 * @version 4.8.3
 * @since release
 *
 * Version Log: https://docs.google.com/document/d/1grqlfPZ2qIrb3CIF2aWJTiKoygQXw1z6A0UvgkgxKjM/edit?usp=sharing
 */

public class Drivetrain {
    private HardwareMap hardwareMap;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    // start with full speed
    private boolean isSpeedReduced = false;
    private String speedStatus = "Normal";

    // imu variables
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation lastAngles;
    private double globalAngle;
    private double correction;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");

        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        lastAngles = new Orientation();

        while (!imu.isGyroCalibrated()) {}
    }
    
    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getFrontLeftDrive() {return frontLeftDrive;}
    public DcMotor getFrontRightDrive() {return frontRightDrive;}
    public DcMotor getRearLeftDrive() {return rearLeftDrive;}
    public DcMotor getRearRightDrive() {return rearRightDrive;}
    public boolean isSpeedReduced() {return isSpeedReduced;}
    public String getSpeedStatus() {return speedStatus;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setFrontLeftDrive(DcMotor frontLeftDrive) {this.frontLeftDrive = frontLeftDrive;}
    public void setFrontRightDrive(DcMotor frontRightDrive) {this.frontRightDrive = frontRightDrive;}
    public void setRearLeftDrive(DcMotor rearLeftDrive) {this.rearLeftDrive = rearLeftDrive;}
    public void setRearRightDrive(DcMotor rearRightDrive) {this.rearRightDrive = rearRightDrive;}
    public void setSpeedReduced(boolean speedReduced) {isSpeedReduced = speedReduced;}
    public void setSpeedStatus(String speedStatus) {this.speedStatus = speedStatus;}

    // Utility
    public void drive(double power) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            rearLeftDrive.setPower(power);
            rearRightDrive.setPower(power);
        } else {
            frontLeftDrive.setPower(power * 0.65);
            frontRightDrive.setPower(power * 0.65);
            rearLeftDrive.setPower(power * 0.65);
            rearRightDrive.setPower(power * 0.65);
        }
    }

    public void drive(double leftPower, double rightPower) {
        if (!isSpeedReduced) {
            frontLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearLeftDrive.setPower(leftPower);
            rearRightDrive.setPower(rightPower);
        } else {
            frontLeftDrive.setPower(leftPower * 0.65);
            frontRightDrive.setPower(rightPower * 0.65);
            rearLeftDrive.setPower(leftPower * 0.65);
            rearRightDrive.setPower(rightPower * 0.65);
        }
    }

    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void switchSpeed() {
        isSpeedReduced = !isSpeedReduced;

        if (isSpeedReduced) {
            speedStatus = "Reduced";
        } else {
            speedStatus = "Normal";
        }
    }

    public void strafeRight(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        rearLeftDrive.setPower(-power);
        rearRightDrive.setPower(power);
    }

    public void strafeLeft(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(-power);
    }

    public void autoDriveTime(double power, double milliseconds) {
        drive(power);

        sleep((long)(milliseconds));

        stop();
    }

//    public void autoDriveDistance(double power, double distance) {
//        if (distance > 0) {
//            frontLeftDrive.setPower(power);
//            frontRightDrive.setPower(power);
//            rearLeftDrive.setPower(power);
//            rearRightDrive.setPower(power);
//        } else if (distance < 0) {
//            frontLeftDrive.setPower(-power);
//            frontRightDrive.setPower(-power);
//            rearLeftDrive.setPower(-power);
//            rearRightDrive.setPower(-power);
//        }
//
//        // Source code for 'sleep'
//        sleep((int)((distance / (72.5 * power)) * 1000));
//    }

    public void autoDriveDistance(double power, double distance) {
        // TODO: Confirm measurements
        double threadsPerCentimeter = ((1120 * 2) / (10 * Math.PI));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        frontRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        rearLeftDrive.setTargetPosition((int)(distance * threadsPerCentimeter));
        rearRightDrive.setTargetPosition((int)(distance * threadsPerCentimeter));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power);

        while (frontLeftDrive.isBusy() || rearRightDrive.isBusy()) {}

        stop();
    }

    public double getPosition() {
        return (frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition()
                + rearLeftDrive.getCurrentPosition() + rearRightDrive.getCurrentPosition()) / 4.0;
    }

    public void resetEncoder() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return frontLeftDrive.isBusy() || frontRightDrive.isBusy() || rearLeftDrive.isBusy() || rearRightDrive.isBusy();
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // IMU
    private void driveIMU(double power) {
        // Use gyro to drive in a straight line.
        correction = checkDirection();

        drive(power - correction, power + correction);
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0) {
            correction = 0;             // no adjustment.
        } else {
            correction = -angle;        // reverse sign of angle for correction.
        }

        correction = correction * gain;

        return correction;
    }

    private void rotate(int degrees, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        drive(leftPower, rightPower);

        // rotate until turn is completed.
        if (degrees < 0) { // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        } else { // left turn.
            while (getAngle() < degrees) {}
        }

        // turn the motors off.
        stop();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}