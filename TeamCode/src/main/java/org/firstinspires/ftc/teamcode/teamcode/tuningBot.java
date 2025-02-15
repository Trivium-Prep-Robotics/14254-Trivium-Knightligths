package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "qualifierCode")
public class tuningBot extends OpMode {

    DcMotor rightFront, leftFront, rightBack, leftBack;

    private DcMotor xOdoPod;
    private DcMotor yOdoPod;
    IMU imu;
    IMU.Parameters myIMUparameters;

    @Override
    public void init () {
        rightFront= hardwareMap.get(DcMotor.class, "rightFront"); //port2
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //port
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //port 1
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //port
        xOdoPod = hardwareMap.get(DcMotor.class, "rightFront"); //port 2
        yOdoPod = hardwareMap.get(DcMotor.class, "rightBack"); // port 1

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        xOdoPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOdoPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xOdoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOdoPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);









       // imu = hardwareMap.get(IMU.class, "imu"); // Initializing IMU in Drivers Hub
        // Reconfiguring IMU orientation
       // myIMUparameters = new IMU.Parameters(
               // new RevHubOrientationOnRobot(
                   //     RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                     //   RevHubOrientationOnRobot.UsbFacingDirection.UP
             //   )
    //   );
     //   imu.initialize(myIMUparameters);
     //   imu.resetYaw();


    }
    @Override
    public void loop () {
        float y = gamepad1.left_stick_y;
        float x = gamepad1.right_stick_x;

        if (gamepad1.x) { //rightFront
            rightFront.setPower(0.3);
        } else {
            rightFront.setPower(0);
        }

        if (gamepad1.y) { //leftFront
            leftFront.setPower(0.3);
        } else {
            leftFront.setPower(0);
        }

        if (gamepad1.b) { //rightBack
            rightBack.setPower(0.3);
        } else {
            rightBack.setPower(0);
        }

        if (gamepad1.a) { //leftBack
            leftBack.setPower(0.3);
        } else {
            leftBack.setPower(0);
        }

        int xPositison = -xOdoPod.getCurrentPosition();
        int yPosition = yOdoPod.getCurrentPosition();

        telemetry.addData("x Encoder", xPositison);
        telemetry.addData("y Encoder", yPosition);
        telemetry.update();

    }//loop

} //class