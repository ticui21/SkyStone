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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 'Component' controls the robot's features, mostly mechanisms designed to score points. It
 * controls robot features such as latches, claws, elevators, intake motors, etc.
 *
 * Key features include:
 * - intake motors
 * - foundation latches
 * - elevator to raise block
 *
 * 'Component' needs a lot of rework each season. A different game theme will have a major impact on
 * a robot's components since each component is usually designed to score points
 *
 * @author Tikki
 * @version 2.3.1
 * @since release
 */

public class Component {
    private HardwareMap hardwareMap;

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor elevator;

    private Servo leftLatch;
    private Servo rightLatch;

    private boolean isLatched = false;
    private String latchStatus = "Pending";

    public Component(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");

        leftLatch = hardwareMap.servo.get("left latch");
        rightLatch = hardwareMap.servo.get("right latch");

        elevator = hardwareMap.dcMotor.get("elevator");
    }

    // Accessor/Mutator
    public HardwareMap getHardwareMap() {return hardwareMap;}
    public DcMotor getLeftIntake() {return leftIntake;}
    public DcMotor getRightIntake() {return rightIntake;}
    public DcMotor getElevator() {return elevator;}
    public Servo getLeftLatch() {return leftLatch;}
    public Servo getRightLatch() {return rightLatch;}
    public boolean isLatched() {return isLatched;}
    public String getLatchStatus() {return latchStatus;}

    public void setHardwareMap(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}
    public void setLeftIntake(DcMotor leftIntake) {this.leftIntake = leftIntake;}
    public void setRightIntake(DcMotor rightIntake) {this.rightIntake = rightIntake;}
    public void setElevator(DcMotor elevator) {this.elevator = elevator;}
    public void setLeftLatch(Servo leftLatch) {this.leftLatch = leftLatch;}
    public void setRightLatch(Servo rightLatch) {this.rightLatch = rightLatch;}
    public void setLatched(boolean latched) {isLatched = latched;}
    public void setLatchStatus(String latchStatus) {this.latchStatus = latchStatus;}

    // Utilities
    public void latch(Trobot.Mode mode) {
        if (mode == Trobot.Mode.LATCH) {
            leftLatch.setPosition(0.5);
            rightLatch.setPosition(0.3);
        } else if (mode == Trobot.Mode.UNLATCH) {
            leftLatch.setPosition(1);
            rightLatch.setPosition(0);
        }
    }

    public void intake(Trobot.Mode mode) {
        if (mode == Trobot.Mode.INTAKE) {
            leftIntake.setPower(0.5);
            rightIntake.setPower(-0.5);
        } else if (mode == Trobot.Mode.RELEASE) {
            leftIntake.setPower(-0.2);
            rightIntake.setPower(0.2);
        } else if (mode == Trobot.Mode.STOP){
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void moveElevator(Trobot.Mode mode) {
        if (mode == Trobot.Mode.UP) {
            elevator.setPower(0.6);
        } else if (mode == Trobot.Mode.DOWN) {
            elevator.setPower(-0.6);
        }
    }
}
