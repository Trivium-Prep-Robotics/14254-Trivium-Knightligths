package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Parts;

public class KnightmaresDrive implements Drive {
    // Setting the normal and slow speeds
    public void fastSlowSpd(double fast, double slow) {
        Parts.driveMaxSpd = fast;
        Parts.driveSlwSpd = slow;
    }
    
    // for auto movement
    public void moveRobot(double x, double y, double yaw) { // power in the x, y, and turn directions
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        Parts.FL.setPower(leftFrontPower);
        Parts.FR.setPower(rightFrontPower);
        Parts.BL.setPower(leftBackPower);
        Parts.BR.setPower(rightBackPower);
    }

    // for teleOp movement (field centric)
    
    public void feildCentric(Gamepad gamepad) { // yeah just put this in your teleOp
        double botHeading = Parts.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double vertical = -gamepad.left_stick_y * Parts.driveMaxSpd;
        double horizontal = gamepad.left_stick_x * Parts.driveMaxSpd;
        double pivot = gamepad.right_stick_x * Parts.driveMaxSpd;
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        if (gamepad.right_trigger > 0) {
            vertical = -gamepad.left_stick_y * Parts.driveSlwSpd;
            horizontal = gamepad.left_stick_x * Parts.driveSlwSpd;
            pivot = gamepad.right_stick_x * (Parts.driveSlwSpd + 0.1);
        }

        // Kinematics (Counter-acting angle of robot's heading)
        double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
        double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

        // Setting Field Centric Drive
        Parts.FL.setPower((newVertical + newHorizontal + pivot) / denominator);
        Parts.FR.setPower((newVertical - newHorizontal - pivot) / denominator);
        Parts.BL.setPower((newVertical - newHorizontal + pivot) / denominator);
        Parts.BR.setPower((newVertical + newHorizontal - pivot) / denominator);
    }
}
