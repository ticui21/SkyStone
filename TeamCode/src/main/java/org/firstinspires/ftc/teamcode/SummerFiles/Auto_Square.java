package org.firstinspires.ftc.teamcode.SummerFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in
 * either the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Square", group = "Summer")
public class Auto_Square extends LinearOpMode {
    private Trobot trobot;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        telemetry.addData("Status", "Waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        trobot.getRuntime().reset();

        trobot.getDrivetrain().drive(0.3);
        sleep(2000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafeLeft(0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().drive(-0.3);
        sleep(2000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafeRight(0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + trobot.getRuntime().toString());
        telemetry.update();
        //}
    }
}
