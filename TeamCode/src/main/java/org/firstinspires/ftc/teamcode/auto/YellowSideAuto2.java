package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name = "YELLOW_AUTO2", group = "Autonomous")
public class YellowSideAuto2 extends LinearOpMode {

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
                if (posR < LiftReference +3450) {
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
                sleep(50);
                Gripper.setPosition(0.1);
                S0.setPosition(1);
                sleep(200);
                SRG.setPosition(0.73);
                SR.setPosition(0.57);
                SL.setPosition(0.43);
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
                SR.setPosition(0.45);
                SL.setPosition(0.55);
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
        public class SlideIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S0.setPosition(0.28);
                sleep(245);
                S0.setPosition(0.6);
                S5.setPosition(0.805);
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
        Pose2d second = new Pose2d(16.5,18,Math.toRadians(-21.5));
        Pose2d second2 = new Pose2d(14.5,22.6,Math.toRadians(-6));
        Pose2d Third = new Pose2d(26.4,14.3,Math.toRadians(-59.5));
        Pose2d Forth = new Pose2d(13.5,15.6,Math.toRadians(-22.5));
        Pose2d Sub = new Pose2d(61.32,-5.6,Math.toRadians(-78));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);


        TrajectoryActionBuilder Putspec1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(16.5,18,Math.toRadians(-21.5)),-Math.PI*2,new TranslationalVelConstraint(45.0));

        TrajectoryActionBuilder firstspec1 = drive.actionBuilder(second)
                .splineToLinearHeading(new Pose2d(14.5,22.6,Math.toRadians(-6)),-Math.PI*2,new TranslationalVelConstraint(50.0));

        TrajectoryActionBuilder firstspec2 = drive.actionBuilder(second2)
                .splineToLinearHeading(new Pose2d(26.4,14.3,Math.toRadians(59.5)),-Math.PI*2);//turn to put in to the basket

        TrajectoryActionBuilder tab3 = drive.actionBuilder(Third)
                .splineToLinearHeading(new Pose2d(13.5,15.6,Math.toRadians(-22.5)),Math.PI*2,new TranslationalVelConstraint(55.0));//go get second spec

        TrajectoryActionBuilder tosub = drive.actionBuilder(Forth)

                .splineToLinearHeading(new Pose2d(61.2,-5.6,Math.toRadians(-78)),Math.PI*2,new TranslationalVelConstraint(65.0));//park

        TrajectoryActionBuilder bas4 = drive.actionBuilder(Sub)
                .splineToLinearHeading(new Pose2d(58,0,Math.toRadians(-78)),Math.PI*2,new TranslationalVelConstraint(65.0))//park
                .splineToLinearHeading(new Pose2d(13.5,15.6,Math.toRadians(-22.5)),Math.PI*2,new TranslationalVelConstraint(65.0));//park






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
                        new ParallelAction(
                                sam1
                        ),



                        new ParallelAction(

                                new SequentialAction(
                                        new SleepAction(0.5),
                                        sam2

                                )

                        ),

                        new ParallelAction(

                                sam3
                        ),

                        new ParallelAction(
                                bas3//put to basket

                        ),
                        new SleepAction(1),


                        new SleepAction(1),
                        new ParallelAction(
                                Tosub
                                //slidedown


                        ),
                        Bas4


                )
        );



    }
}