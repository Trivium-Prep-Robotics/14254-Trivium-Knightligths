/*
Kunal Trivedi
Version 1

Estrella Foothills qualifier competition TeleOp code

arm verticl at: -2145,

max in veritcal extension: -7050, new 2660

max in horizontal extension: -2862, new 1079

312 rpm:537.7

117 rpm: 1425.1


makebthe arm so that a button make it go vertical and another horizontal
when vertical, extend fully,
when not vertical, do not extend fully

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "glendaleComp")
public class glendaleComp extends LinearOpMode {


    DcMotor rightFront, leftFront, rightBack, leftBack, leftArm, rightArm, leftExtend, rightExtend; //initializes motors
    public Servo claw, clawMovement; // non continuous servo initialization


    @Override
    public void runOpMode() {

        //naming them in the configuration of the driver station
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //port 2 on control
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //port 0 on control
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //port 3 on control
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //port 1 on control
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");   //port 2 on expansion
        rightArm = hardwareMap.get(DcMotor.class, "rightArm"); // port 3  on expansion
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend"); //port 0 on expansion
        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend"); //port 1 on expansion
        clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
        claw = hardwareMap.get(Servo.class, "claw");              //port 1 on expansion


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtend.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightExtend.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFront.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        leftFront.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightBack.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        leftBack.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));



        /*
        close claw....left bumper...coded...directions checked...working
        open claw....right bumper...coded...
        bring arm up...y...coded...directions checked...working
        bring arm down...a...coded...directions checked..working
        extend slide...right trigger...coded...directions checked...working
        unextend slides...left trigger...coded...directions checked...working
        move servo to score...x...coded...
        move servo down...b...coded...
        move robot...d pad...not coded...directions checked...working
        move robot slowly...joystick...not coded...directions checked...working
        turn robot...right joystick...coded...directions checked...working

         */

        waitForStart();
        while (opModeIsActive()) {


            float y = gamepad1.left_stick_y;
            int leftSlidePos = leftExtend.getCurrentPosition();
            int rightArmPos = rightArm.getCurrentPosition();
            int rightSlidePos = rightExtend.getCurrentPosition();
            int leftArmPos = leftArm.getCurrentPosition();


            int maxExtend = -2560;
            int changedExtend = -1079;
            int verticalArm = -2145;

            if (rightArmPos > verticalArm) { //if within base position and -2145 (90 ish degrees)
                if (gamepad1.y) { //arm up
                    rightArm.setPower(1);
                    leftArm.setPower(1);
                } else if (gamepad1.a) { //arm down
                    rightArm.setPower(-1);
                    leftArm.setPower(-1);

                } else { //if over that 90 ish degrees
                    rightArm.setPower(0);
                    leftArm.setPower(0);
                }

            } else {
                if (gamepad1.a) { //arm down
                    rightArm.setPower(-1);
                    leftArm.setPower(-1);

                } else {
                    rightArm.setPower(0);
                    leftArm.setPower(0);
                }

            }


            if (gamepad1.right_trigger > 0) { //extends slides
                if (leftSlidePos < 1200) { //if within base and 2500
                    rightExtend.setPower(1);
                    leftExtend.setPower(-1);
                }
            } else if (gamepad1.left_trigger > 0) { //retracts slides
                rightExtend.setPower(-1);
                leftExtend.setPower(1);
            } else {

                rightExtend.setPower(0);
                leftExtend.setPower(0);
            }


/*
        if(gamepad1.dpad_right){ //extend slide
            if(armPos > -2145) {
                leftExtend.setTargetPosition(maxExtend);
                rightExtend.setTargetPosition(maxExtend);

                leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftExtend.setPower(1);

                rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightExtend.setPower(1);

            }
            else{
                leftExtend.setTargetPosition(-changedExtend);
                rightExtend.setTargetPosition(-changedExtend);

              leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              leftExtend.setPower(1);

               rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rightExtend.setPower(1);
            }
        }else {
            leftExtend.setPower(0);
            rightExtend.setPower(0);
        }

/*
            if (gamepad1.dpad_right) {
                if (armPos > -2145) {
                    while (leftExtend.getCurrentPosition() > maxExtend && opModeIsActive()) {
                        leftExtend.setPower(1);
                        rightExtend.setPower(1);

                    }
                    while (leftExtend.getCurrentPosition() > maxExtend && opModeIsActive()) {
                        leftExtend.setPower(0);
                        rightExtend.setPower(0);
                    }

                } else {
                    while (leftExtend.getCurrentPosition() > changedExtend && opModeIsActive()) {
                        leftExtend.setPower(1);
                        rightExtend.setPower(1);

                    }
                    while (leftExtend.getCurrentPosition() > changedExtend && opModeIsActive()) {
                        leftExtend.setPower(0);
                        rightExtend.setPower(0);
                    }

                }
            } else {
                leftExtend.setPower(0);
                rightExtend.setPower(0);
            }

*/
            telemetry.addData("left slide encoder ", leftSlidePos);
            telemetry.addData("right arm encoder ", rightArmPos);
            telemetry.addData("right slide encoder ", rightSlidePos);
            telemetry.addData("left arm encoder", leftArmPos);
            telemetry.update();


            if (gamepad1.right_bumper) { //open claw
                claw.setPosition(0.5);
            }

            if (gamepad1.left_bumper) { //close claw
                claw.setPosition(0);
            }

            double clawPosition = clawMovement.getPosition();



            if (gamepad1.x && clawPosition < 1) { //move claw to score
                clawMovement.setPosition(1);
            }

            if (gamepad1.x && clawPosition == 1){
                clawMovement.setPosition(0.35);
            }

            if (gamepad1.b) { //move claw down
                clawMovement.setPosition(0.25);
            }


            if (gamepad1.left_stick_y > 0) { //forwards
                rightFront.setPower(0.4);
                leftFront.setPower(0.4);
                rightBack.setPower(0.4);
                leftBack.setPower(0.4);

            } else if (gamepad1.left_stick_y < 0) { //backwards
                rightFront.setPower(-0.4);
                leftFront.setPower(-0.4);
                rightBack.setPower(-0.4);
                leftBack.setPower(-0.4);
            } else if (gamepad1.left_stick_x > 0) { //strafe right
                rightFront.setPower(0.4);
                leftFront.setPower(-0.4);
                rightBack.setPower(-0.4);
                leftBack.setPower(0.4);

            } else if (gamepad1.left_stick_x < 0) { //strafe left
                rightFront.setPower(-0.4);
                leftFront.setPower(0.4);
                rightBack.setPower(0.4);
                leftBack.setPower(-0.4);


            } else if (gamepad1.dpad_up) {
                rightFront.setPower(-1);
                leftFront.setPower(-1);
                rightBack.setPower(-1);
                leftBack.setPower(-1);
            } else if (gamepad1.dpad_down) {
                rightFront.setPower(1);
                leftFront.setPower(1);
                rightBack.setPower(1);
                leftBack.setPower(1);
            } else if (gamepad1.dpad_right) { //strafe right
                rightFront.setPower(1);
                leftFront.setPower(-1);
                rightBack.setPower(-1);
                leftBack.setPower(1);

            } else if (gamepad1.dpad_left) { //strafe left
                rightFront.setPower(-1);
                leftFront.setPower(1);
                rightBack.setPower(1);
                leftBack.setPower(-1);
            } else if (gamepad1.right_stick_x > 0) { //turn right
                rightFront.setPower(0.6);
                leftBack.setPower(-0.6);
                rightBack.setPower(0.6);
                leftFront.setPower(-0.6);

            } else if (gamepad1.right_stick_x < 0) { //turn left
                rightFront.setPower(-0.6);
                leftBack.setPower(0.6);
                rightBack.setPower(-0.6);
                leftFront.setPower(0.6);

            } else {
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
            }

        }
    }// loop
} //class
