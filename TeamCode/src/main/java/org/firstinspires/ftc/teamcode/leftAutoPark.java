package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "park")
public class leftAutoPark extends LinearOpMode {

    DcMotor rightFront, leftFront, rightBack, leftBack, leftArm, rightArm, rightExtend, leftExtend;

    public Servo clawMovement, claw, slide;




    @Override
    public void runOpMode() throws InterruptedException {
        rightFront= hardwareMap.get(DcMotor.class, "rightFront"); //port 2 on control
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //port 0 on control
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //port 3 on control
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //port 1 on control
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");   //port 2 on expansion
        rightArm = hardwareMap.get(DcMotor.class, "rightArm"); // port 3  on expansion
        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend"); //port 0 on expansion
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend"); //port 1 on expansion
        clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
        slide = hardwareMap.get(Servo.class, "clawMovement");         //port 1 on control
        claw = hardwareMap.get(Servo.class, "claw");              //port 1 on expansion


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();


        rightFront.setPower(0.4);
        leftFront.setPower(-0.4);
        rightBack.setPower(-0.4);
        leftBack.setPower(0.4);

        sleep(6000);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

    }


} //class





