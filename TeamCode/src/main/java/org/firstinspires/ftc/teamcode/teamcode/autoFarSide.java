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
            leftLift = hardwareMap.get(DcMotorEx.class, "leftArm"); //port 2 on expansion
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

            rightLift = hardwareMap.get(DcMotorEx.class, "rightArm");//port 3 on expansion
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
        public Action extendHalf(){
            return new ExtendHalf(this);
        }
    }

    public class Claw {
        private Servo pinchers, clawMovement, wrist;

        public Claw(HardwareMap hardwareMap) {
            pinchers = hardwareMap.get(Servo.class, "claw"); //port 1 on control
            clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
            wrist = hardwareMap.get(Servo.class, "wrist");

        }
        public void setClawPosition (double pos) {
            pinchers.setPosition(pos);
        }

        public void setClawMovementPosition (double pos) {
            clawMovement.setPosition(pos);
        }

        public void setWristPosition (double pos) {
            wrist.setPosition(pos);
        }

        public Action openClaw() {
            return new OpenClaw(this);
        }

        public Action closeClaw() {
            return new CloseClaw(this);
        }

        public Action wristOriginal() {
            return new WristOriginal(this);
        }

        public Action wristFlipped() {
            return new WristFlipped(this);
        }
        public Action pickUpFloor() {
            return new PickUpFloor(this);
        }

        public Action pickUpWall() {
            return new PickUpWall(this);
        }

        public Action movementScore() {
            return new MovementScore(this);
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
            if (pos > -2000) { //uses left motor
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
            if (pos < -0) {
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
            if (pos > -4300) { //uses right motor
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
            if (pos < 0) {
                return true;
            } else {
                arm.setExtendPower(0);
                return false;
            }
        }
    }

    public class ExtendHalf implements Action { //actual action for the arm down
        public ExtendHalf (Arm arm){ //allows arm to pass through it

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
            if (pos < -1000 || pos > -1000) {
                return true;
            } else {
                arm.setExtendPower(0);
                return false;
            }
        }
    }

    public class OpenClaw implements Action {
        public OpenClaw (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setClawPosition(1.0);
            return false;
        }
    }

    public class CloseClaw implements Action {
        public CloseClaw (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setClawPosition(0);
            return false;
        }
    }

    public class WristOriginal implements Action {
        public WristOriginal (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setWristPosition(0);
            return false;
        }
    }

    public class WristFlipped implements Action {
        public WristFlipped (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setWristPosition(0.6);
            return false;
        }
    }

    public class PickUpFloor implements Action {
        public PickUpFloor (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setClawMovementPosition(0);
            return false;
        }
    }

    public class PickUpWall implements Action {
        public PickUpWall (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setClawMovementPosition(0.6);
            return false;
        }
    }

    public class MovementScore implements Action {
        public MovementScore (Claw claw) {
            this.claw = claw;
        }

        private Claw claw;

        @Override


        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setClawMovementPosition(0.6);
            return false;
        }
    }










    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Claw claw = new Claw(hardwareMap);
        // make a Lift instance
        Arm lift = new Arm(hardwareMap);

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();


        Actions.runBlocking((claw.closeClaw()));

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        lift.extendUp(),
                        lift.extendDown(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );

    }



} // class


