// autonomos for sample
// encoder value of 117 rpm motor: 	1,425.1


package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "rightAuto", group = "Autonomous")
public class autoFarSide extends LinearOpMode {

    public class Arm {
        private DcMotorEx leftLift, rightLift;
        private DcMotorEx leftExtend, rightExtend;


        public Arm(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotorEx.class, "leftLift"); //port 2 on expansion
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");//port 3 on expansion
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

            leftExtend = hardwareMap.get(DcMotorEx.class, "leftExtend"); //port 0 on expansion
            leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftExtend.setDirection(DcMotorSimple.Direction.FORWARD);

            rightExtend = hardwareMap.get(DcMotorEx.class, "rightExtend"); //port 1 on expansion
            rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightExtend.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public void setExtendPower (double power) {
            rightExtend.setPower(power);
            leftExtend.setPower(power);
        }

        public void setLiftPower (double power) {
            rightLift.setPower(power);
            leftLift.setPower(power);
        }

        public double checkLiftPosition(){
         double  rightPos = rightLift.getCurrentPosition();
         double leftPos = leftLift.getCurrentPosition();

            return leftPos;
        }
        public double checkExtendPosition(){
            double  rightPos = rightExtend.getCurrentPosition();
            double leftPos = leftExtend.getCurrentPosition();

            return rightPos;
        }

        public Action liftUp() {

            return new LiftUp(this);
        }
        public Action liftDown() {
            return new LiftDown(this);
        }
        public Action extendUp() {
            return new ExtendUp (this);
        }

        public Action extendDown() {
            return new ExtendDown (this);
        }

    }

    public class Claw {
        private Servo claw, clawMovement;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw"); //port 1 on control
            clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control

        }


    }
    MecanumDrive drive;


    public class LiftUp implements Action { //actual action for the arm up

        public LiftUp (Arm arm){ //allows arm to pass through it

            this.arm = arm;

        }



        private Arm arm;
        // checks if the lift motor has been powered on
        private boolean initialized = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {



            // powers on motor, if it is not on

            if (!initialized) {
                arm.setLiftPower(0.6);

                initialized = true;
            }

            // checks lift's current position
            double pos = arm.checkLiftPosition();
            packet.put("liftPos", pos);
            if (pos < 356.25) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                arm.setLiftPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 356.25 encoder ticks, then powers it off
        }


    }

    public class LiftDown implements Action { //actual action for the arm down
        public LiftDown (Arm arm){ //allows arm to pass through it

            this.arm = arm;

        }
        private Arm arm;

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm.setLiftPower(-0.8);
                initialized = true;
            }

            double pos = arm.checkLiftPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                arm.setLiftPower(0);
                return false;
            }
        }
    }

    public class ExtendUp implements Action { //actual action for the slides up
        public ExtendUp (Arm arm){ //allows arm to pass through it

            this.arm = arm;

        }
        private Arm arm;

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm.setExtendPower(0.8);
                initialized = true;
            }

            double pos = arm.checkLiftPosition();
            packet.put("liftPos", pos);
            if (pos < 300) {
                return true;
            } else {
                arm.setExtendPower(0.8);
                return false;
            }
        }
    }

    public class ExtendDown implements Action { //actual action for the arm down
        public ExtendDown (Arm arm){ //allows arm to pass through it

            this.arm = arm;

        }
        private Arm arm;

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm.setExtendPower(-0.8);
                initialized = true;
            }

            double pos = arm.checkExtendPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                arm.setExtendPower(0);
                return false;
            }
        }
    }








        @Override
        public void runOpMode() throws InterruptedException {

            waitForStart();
/*
            drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToYSplineHeading(33, Math.toRadians(0))
                    .waitSeconds(2)
                    .setTangent(Math.toRadians(90))
                    .splineTo(new Vector2d(48, 48), Math.PI / 2);
*/

            //Actions.runBlocking();


        }

    } // class


