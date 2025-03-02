package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Parts;
import org.firstinspires.ftc.teamcode.parts.ArmTemplate;
import org.firstinspires.ftc.teamcode.parts.ClawTemplate;
import org.firstinspires.ftc.teamcode.parts.DriveTemplate;

/**
 * This is the general structure for an autonomous
 */

@Autonomous (name = "Net side", group = "old Auto")
//@Disabled
public class AutoTemplate extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Parts robot = new Parts(hardwareMap); // configure robot
        DriveTemplate drive = new DriveTemplate(); // arm class
        ClawTemplate claw = new ClawTemplate(); // claw class
        ArmTemplate arm = new ArmTemplate(); // drive class

        waitForStart();
        
    }
}
