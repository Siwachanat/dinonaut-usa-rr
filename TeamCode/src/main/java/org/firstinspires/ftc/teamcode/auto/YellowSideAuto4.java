package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "YELLOW_AUTO4", group = "Autonomous")
public class YellowSideAuto4 extends LinearOpMode {

    public class Lift {
        private DcMotorEx liftR;
        private DcMotorEx liftL;
        public int LiftReference;

        public Lift(HardwareMap hardwareMap) {
            liftR = hardwareMap.get(DcMotorEx.class, "liftMotorR");
            liftL = hardwareMap.get(DcMotorEx.class, "liftMotorL");
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            LiftReference = liftR.getCurrentPosition();
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(1);
                    liftL.setPower(-1);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posR);
                if (posR < LiftReference +3200) {
                    return true;
                } else {
                    liftL.setPower(0.08);
                    liftR.setPower(0.08);
                    return false;
                }


            }
        }
        public Action liftUp() {
            return new LiftUp();
        }


        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-1);
                    liftL.setPower(1);
                    initialized = true;
                }

                double pos = liftR.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 400) {
                    return true;
                } else {
                    liftR.setPower(-0.055);
                    liftL.setPower(0.055);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public class Mission {
        public Servo S0;
        public Servo S1;
        public Servo S4;
        public Servo S5;
        private Servo Smid;
        private Servo SR;
        private Servo SL;
        private Servo SRG;
        private Servo Gripper;
        private Servo Sarm;
        public Mission(HardwareMap hardwareMap) {

            SRG = hardwareMap.get(Servo.class, "SRG");
            S0 = hardwareMap.get(Servo.class, "S0");
            S1 = hardwareMap.get(Servo.class, "S1");
            S5 = hardwareMap.get(Servo.class, "S5");
            S4 = hardwareMap.get(Servo.class, "S4");
            Smid = hardwareMap.get(Servo.class, "midGrip");
            SR = hardwareMap.get(Servo.class, "SR");
            SL = hardwareMap.get(Servo.class, "SL");
            Gripper = hardwareMap.get(Servo.class, "Gripper");
            Sarm = hardwareMap.get(Servo.class, "Sarm");

        }
        public class Set implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.1);
                sleep(200);
                SRG.setPosition(0.73);
                SR.setPosition(0.57);
                SL.setPosition(0.43);
                Smid.setPosition(0.98);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }

        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SL.setPosition(0.135);
                SR.setPosition(0.865);
                sleep(120);
                Gripper.setPosition(0.1);
                sleep(100);
                S0.setPosition(1);
                sleep(225);
                SRG.setPosition(0.73);
                SR.setPosition(0.58);
                SL.setPosition(0.42);
                Smid.setPosition(0.98);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }

        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.48);
                SL.setPosition(0.52);
                Gripper.setPosition(0.8);
                sleep(150);
                SRG.setPosition(0.73);
                Gripper.setPosition(0.7);
                Smid.setPosition(0.35);
                SL.setPosition(0.2);
                SR.setPosition(0.8);

                return false;
            }
        }
        public Action releases() {
            return new Re();
        }

        public class Re2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.48);
                SL.setPosition(0.52);
                Gripper.setPosition(0.8);
                sleep(150);
                SRG.setPosition(0.73);
                Gripper.setPosition(0.7);
                Smid.setPosition(0.35);
                SL.setPosition(0.4);
                SR.setPosition(0.6);

                return false;
            }
        }
        public Action releases2() {
            return new Re2();
        }
        public class SlideFullUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);
                S1.setPosition(0.95);
                S0.setPosition(0.76);
                S5.setPosition(0);
                return false;
            }
        }
        public Action slideFullUP() {
            return new SlideFullUP();
        }
        public class SlideFullUP90 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(1);
                S1.setPosition(0.95);
                S0.setPosition(0.76);
                S5.setPosition(0);
                return false;
            }
        }
        public Action slideFullUP90() {
            return new SlideFullUP90();
        }
        public class SlideFullUP45 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.65);
                S1.setPosition(0.95);
                S0.setPosition(0.76);
                S5.setPosition(0);
                return false;
            }
        }
        public Action slideFullUP45() {
            return new SlideFullUP90();
        }

        public class SlideIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S0.setPosition(0.28);
                sleep(245);
                S0.setPosition(0.6);
                S5.setPosition(0.842);
                S1.setPosition(0.5);
                sleep(150);
                S1.setPosition(0);
                S4.setPosition(1);
                return false;
            }
        }
        public Action slideIN() {
            return new SlideIN();
        }

        public class SlideIN2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S0.setPosition(0.28);
                sleep(245);
                S0.setPosition(0.6);
                S5.setPosition(0.75);
                sleep(300);
                S1.setPosition(0.5);
                sleep(150);
                S1.setPosition(0);
                S4.setPosition(1);
                return false;
            }
        }
        public Action slideIN2() {
            return new SlideIN2();
        }

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.49);
                return false;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }

        public class SetY implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SRG.setPosition(0.57);
                Smid.setPosition(0.98);
                SR.setPosition(0.53);
                SL.setPosition(0.47);
                return false;
            }
        }
        public Action setY() {
            return new SetY();
        }


        public class ReGrip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.12);
                return false;
            }
        }
        public Action reGrip() {
            return new ReGrip();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d second = new Pose2d(18,18.5,Math.toRadians(-30));
        Pose2d second2 = new Pose2d(16,23.7,Math.toRadians(-8));
        Pose2d Third = new Pose2d(26.3,11.9,Math.toRadians(-55));
        Pose2d Forth = new Pose2d(13,15,Math.toRadians(-48));
        Pose2d Sub = new Pose2d(61.2,-5.6,Math.toRadians(-78));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);


        TrajectoryActionBuilder Putspec1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(18,18.5,Math.toRadians(-30)),-Math.PI*2,new TranslationalVelConstraint(45.0));

        TrajectoryActionBuilder firstspec1 = drive.actionBuilder(second)
                .splineToLinearHeading(new Pose2d(16,23.7,Math.toRadians(-8)),-Math.PI*2,new TranslationalVelConstraint(50.0));

        TrajectoryActionBuilder firstspec2 = drive.actionBuilder(second2)
                .splineToLinearHeading(new Pose2d(26.3,11.9,Math.toRadians(55)),-Math.PI*2);//turn to put in to the basket

        TrajectoryActionBuilder tab3 = drive.actionBuilder(Third)
                .splineToLinearHeading(new Pose2d(13,15,Math.toRadians(-48)),Math.PI*2,new TranslationalVelConstraint(55.0));//go get second spec

        TrajectoryActionBuilder tosub = drive.actionBuilder(Forth)

                .splineToLinearHeading(new Pose2d(61.2,-5.6,Math.toRadians(-78)),Math.PI*2,new TranslationalVelConstraint(65.0));//park

        TrajectoryActionBuilder bas4 = drive.actionBuilder(Sub)
                .splineToLinearHeading(new Pose2d(30,8,Math.toRadians(-50)),Math.PI*2,new TranslationalVelConstraint(65.0))//park
                .splineToLinearHeading(new Pose2d(13,15,Math.toRadians(-48)),Math.PI*2,new TranslationalVelConstraint(65.0));//park






        Action sam1;
        Action sam2;
        Action sam3;
        Action bas3;
        Action Tosub;

        Action Bas4;



        sam1 = Putspec1.build();
        sam2 = firstspec1.build();
        sam3 = firstspec2.build();
        bas3 = tab3.build();
        Tosub = tosub.build();

        Bas4 = bas4.build();
        Actions.runBlocking(mission.reGrip());



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        mission.set(),
                        new ParallelAction(
                                lift.liftUp(),
                                mission.slideFullUP(),
                                sam1
                        ),
                        mission.releases(),
                        lift.liftDown(),
                        mission.slideIN(),
                        new SleepAction(0.5),
                        mission.grip(),
                        lift.liftUp(),
                        mission.releases(),
                        new ParallelAction(

                                new SequentialAction(
                                        lift.liftDown(),
                                        mission.slideFullUP(),
                                        sam2
                                )

                        ),
                        mission.slideIN(),
                        new SleepAction(0.5),
                        mission.grip(),
                        lift.liftUp(),
                        mission.releases(),
                        new ParallelAction(
                                mission.slideFullUP90(),
                                lift.liftDown(),
                                sam3
                        ),
                        mission.slideIN2(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                bas3//put to basket
                        ),
                        mission.grip(),
                        lift.liftUp(),
                        mission.releases(),
                        lift.liftDown(),

                        new SleepAction(1),
                        new ParallelAction(
                                Tosub
                                //slidedown


                        ),
                        new SleepAction(1)
                        //Bas4


                )
        );



    }
}