package org.firstinspires.ftc.teamcode.Archives;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

@TeleOp(name = "Motor Check (beta)", group = "Linear OpMode")
public class MotorCheck extends LinearOpMode {
    private Trobot trobot;

    private String testStatus = "Preparing to run tests...";

    @Override
    public void runOpMode() {
        trobot = new Trobot(hardwareMap);

        telemetry.addData("Status", testStatus);
        telemetry.update();

        waitForStart();
        trobot.getRuntime().reset();

        // Begin tests
        trobot.getDrivetrain().autoDriveDistance(0.5, 100);
    }

}