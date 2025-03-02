package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parts;
import org.firstinspires.ftc.teamcode.parts.ArmTemplate;
import org.firstinspires.ftc.teamcode.parts.ClawTemplate;
import org.firstinspires.ftc.teamcode.parts.DriveTemplate;


/**
 * This is a template for a teleOp
 * It should allow you to move field centric and has the base structure of teleOp along with configuring the robot
 */
@TeleOp (name = "teleOp", group = "TELEOP")
public class teleOpTemplate extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        Parts robot = new Parts(hardwareMap); // configure robot
        DriveTemplate drive = new DriveTemplate(); // arm class
        ClawTemplate claw = new ClawTemplate(); // claw class
        ArmTemplate arm = new ArmTemplate(); // drive class

        waitForStart(); // initialize

        // set the power used for arm motors to 1
        arm.armPower(1); // set arm power
        arm.extendPower(1); // set extend power

        claw.openClosePose(0.4, 0.8); // set claw positions
        claw.sampSpecPose(0.075, 0.6); // set wrist positions

        while (opModeIsActive()) {
            /**
             * I the button in the parameters are the assigned button for that command
             * the stop methods on the other hand you just put the conditions in which you don't want them moving
             *
             * If you use the joysticks or triggers for any case you should go to the interface
             * and set the method to intake a double instead of a boolean and then do the same for
             * the class that extends it.
             *
             * By default the extend and retract use doubles, sense our code used the trigger buttons for those
             * so if you want to use another button you switch it to boolean
             */

            // arm controls
            arm.up(gamepad2.dpad_up);
            arm.down(gamepad2.dpad_down);
            // I put this in my code so that it knew when to stop moving
//            arm.armStop(!(gamepad2.dpad_up || gamepad2.dpad_down));

            // extend controls
            arm.extend(gamepad2.right_trigger);
            arm.retract(gamepad2.left_trigger);
            // I put this in my code so that it knew when to stop moving
//            arm.slideStop((gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 && !gamepad2.dpad_up && !gamepad2.dpad_down));

            // claw controls
            claw.grabs(gamepad2.right_bumper);
            claw.drops(gamepad2.left_bumper);

            // wrist controls
            claw.specimen(gamepad2.y);
            claw.sample(gamepad2.a);

            // the particular buttons for field centric drive are just in the method, so the only parameter you need is the gamepad you want the controls on
            drive.feildCentric(gamepad1);
        }
    }
}
