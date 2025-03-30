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
@Autonomous(name = "RED_AUTO6", group = "Autonomous")
public class RedSideAuto6 extends LinearOpMode {

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
            LiftReference = liftL.getCurrentPosition();
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-0.7);
                    liftL.setPower(0.7);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posL);
                if (posL < LiftReference +3500) {
                    return true;
                } else {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }


            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftUp2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-1);
                    liftL.setPower(1);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();

                packet.put("liftPos", posL);
                if (posL < LiftReference +1500) {
                    return true;
                } else {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }


            }
        }
        public Action liftUp2() {
            return new LiftUp2();
        }


        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(0.95);
                    liftL.setPower(-0.95);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 400) {
                    return true;
                } else {
                    liftR.setPower(0.055);
                    liftL.setPower(-0.055);
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
                sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.32);
                SR.setPosition(0.68);
                Gripper.setPosition(0.215);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }

        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.1);
                sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.30);
                SR.setPosition(0.70);
                Gripper.setPosition(0.215);
                SRG.setPosition(0.15);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }

        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.23);
                SL.setPosition(0.77);
                sleep(250);
                Gripper.setPosition(0.8);
                Smid.setPosition(0.67);
                SRG.setPosition(0.15);
                return false;
            }
        }
        public Action releases() {
            return new Re();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.65);
                Smid.setPosition(0.31);
                SR.setPosition(0.31);
                SL.setPosition(0.69);
                Gripper.setPosition(0.2);
                return false;
            }
        }
        public Action mid() {
            return new Mid();
        }
        public class SlideFullUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);
                S1.setPosition(1);
                S0.setPosition(1);
                sleep(50);
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
                S5.setPosition(0.805);
                S1.setPosition(0);
                S4.setPosition(1);
                return false;
            }
        }
        public Action slideIN() {
            return new SlideIN();
        }


        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.045);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.15);
                return false;
            }
        }
        public Action armMid() {
            return new ArmMid();
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
                Gripper.setPosition(0.8);
                return false;
            }
        }
        public Action reGrip() {
            return new ReGrip();
        }

        public class Grippush implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S0.setPosition(0.42);
                sleep(245);
                S0.setPosition(0.7);
                S5.setPosition(0.805);
                S1.setPosition(0.5);
                sleep(150);
                S1.setPosition(0);
                S4.setPosition(1);
                return false;
            }
        }
        public Action grippush() {
            return new Grippush();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d second = new Pose2d(31.75,0,Math.toRadians(0));
        Pose2d third = new Pose2d(6.4,-35,Math.toRadians(0));
        Pose2d forth = new Pose2d(31.75,-2,Math.toRadians(0));
        Pose2d forthfixed = new Pose2d(25,-2,Math.toRadians(0));
        Pose2d fifth = new Pose2d(7,-35,Math.toRadians(0));
        Pose2d fifth2 = new Pose2d(7,-35,Math.toRadians(0));
        Pose2d six = new Pose2d(31.75,2,Math.toRadians(0));
        Pose2d sev = new Pose2d(31.75,-4,Math.toRadians(0));
        Pose2d exx = new Pose2d(31.75,-4.5,Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(31.5,0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-60,80));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                //.strafeTo(new Vector2d(26,-8),null,new ProfileAccelConstraint(-100,100))
                .splineToSplineHeading(new Pose2d(26,-0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-100,100))
                .splineToLinearHeading(new Pose2d(8.5,-25,-Math.PI*5.5/6 ),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .stopAndAdd(mission.slideFullUP())
                .stopAndAdd(mission.slideIN())
                .splineToLinearHeading(new Pose2d(3.5,-30,-Math.PI/3.5 ),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .stopAndAdd(mission.armDown())
                .splineToLinearHeading(new Pose2d(21 ,-34,-Math.PI*4.5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(30,-34.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(32,-40,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-40,-Math.PI*5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-42,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(32,-46,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-46.5,-Math.PI*5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(6.4 ,-35,-Math.PI*2),-Math.PI*1.5,new TranslationalVelConstraint(90));

        TrajectoryActionBuilder Tosam2 = drive.actionBuilder (third)
                .strafeTo(new Vector2d(31.75,-2),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam3 = drive.actionBuilder (forth)
                .splineToConstantHeading(new Vector2d(25,-2),Math.PI*2,new TranslationalVelConstraint(80));

        TrajectoryActionBuilder Tosam3fix = drive.actionBuilder (forthfixed)
                .strafeTo(new Vector2d(7,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam4 = drive.actionBuilder (fifth2)
                .strafeTo(new Vector2d(31.75,2),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam5 = drive.actionBuilder (six)
                .strafeTo(new Vector2d(7,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam6 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(31.75,-4),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam7 = drive.actionBuilder (sev)

                .strafeTo(new Vector2d(7,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam8 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(31.75,-4.5),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam9 = drive.actionBuilder (exx)

                .strafeTo(new Vector2d(7,-45),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam10 = drive.actionBuilder (fifth)
                .splineToSplineHeading( new Pose2d(-54,-50,-Math.PI/2),-Math.PI*1.4,new TranslationalVelConstraint(90))
                .splineToSplineHeading( new Pose2d(-55.5,50,-Math.PI/2),-Math.PI*2,new TranslationalVelConstraint(90));








        Actions.runBlocking(mission.set());
        Actions.runBlocking(mission.slideIN());

        Actions.runBlocking(mission.set());
        Actions.runBlocking(mission.slideIN());


        Action Middle;
        Action Sam1;
        Action Sam2;
        Action Sam1_5;
        Action Sam3;
        Action Sam3fix;
        Action Sam4;
        Action Sam5;
        Action Sam6;
        Action Sam7;
        Action Sam8;
        Action Sam9;
        Action Sam10;



        Middle = Tomid.build();
        Sam1 = Tosam1.build();
        Sam2 = Tosam2.build();
        //Sam1_5 = Tosam1_5.build();
        Sam3 = Tosam3.build();
        Sam3fix = Tosam3fix.build();
        Sam4 = Tosam4.build();
        Sam5 = Tosam5.build();
        Sam6 = Tosam6.build();
        Sam7 = Tosam7.build();
        Sam8 = Tosam8.build();
        Sam9 = Tosam9.build();
        Sam10 = Tosam10.build();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                mission.slideFullUP(),
                                Middle,//first speciment Trjectory Moving Path
                                mission.set() //Set Top gripper Ready to ng speciment
                        ),
                        mission.grippush(),
                        new ParallelAction(

                                mission.releases(),
                                mission.slideIN(),
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        mission.armMid()
                                )
                        ),


                        new ParallelAction(
                                Sam2,
                                mission.armUp(),
                                mission.grip()
                        ),
                        new ParallelAction(
                                mission.releases(),
                                Sam3fix
                        ),
                        new ParallelAction(
                                Sam4,
                                mission.grip()

                        ),
                        new ParallelAction(
                                mission.releases(),
                                Sam5
                        ),
                        new ParallelAction(
                                Sam6,
                                mission.grip()

                        ),
                        new ParallelAction(
                                mission.releases(),
                                Sam7
                        ),
                        new ParallelAction(
                                Sam8,
                                mission.grip()

                        ),
                        new ParallelAction(
                                Sam9,
                                mission.releases()
                        ),
//                        new ParallelAction(
//                                Sam10,
//                                mission.grip(),
//                                lift.liftUp(),
//                                new SequentialAction(
//                                        new SleepAction(0.85),
//                                        mission.setY()
//                                )
//                        ),
//                        mission.reGrip(),
                        new SleepAction(1)

                )
        );



    }
}