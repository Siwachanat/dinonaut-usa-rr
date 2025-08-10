package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.auto.ServoInterpolator;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "RED_AUTO6+0", group = "Autonomous")
public class RedSideAuto6_0 extends LinearOpMode {

    public ServoInterpolator ITPS;
    public Limelight3A limelight;

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
                if (posR < LiftReference +3525) {
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
                if (pos > 350) {
                    return true;
                } else {
                    liftR.setPower(-0.1);
                    liftL.setPower(0.1);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Mission2 {
        public Servo S0;
        public Servo S1;
        public Servo S4;
        public Servo S5;
//        private Servo Smid;
//        private Servo SR;
//        private Servo SL;
//        private Servo SRG;
//        private Servo Gripper;
//        private Servo Sarm;






        public Mission2(HardwareMap hardwareMap) {
//            SRG = hardwareMap.get(Servo.class, "SRG");
            S0 = hardwareMap.get(Servo.class, "S0");
            S1 = hardwareMap.get(Servo.class, "S1");
            S5 = hardwareMap.get(Servo.class, "S5");
            S4 = hardwareMap.get(Servo.class, "S4");
//            Smid = hardwareMap.get(Servo.class, "midGrip");
//            SR = hardwareMap.get(Servo.class, "SR");
//            SL = hardwareMap.get(Servo.class, "SL");
//            Gripper = hardwareMap.get(Servo.class, "Gripper");
//            Sarm = hardwareMap.get(Servo.class, "Sarm");
        }








        public class Set implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                Gripper.setPosition(0.1);
//                sleep(70);
//                Smid.setPosition(0.72);
//                SL.setPosition(0.32);
//                SR.setPosition(0.68);
//                Gripper.setPosition(0.215);
                return false;
            }
        }


        public Action set() {
            return new Set();
        }


        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                Gripper.setPosition(0.1);
//                sleep(70);
//                Smid.setPosition(0.72);
//                SL.setPosition(0.30);
//                SR.setPosition(0.70);
//                Gripper.setPosition(0.215);
                return false;
            }
        }


        public Action grip() {
            return new Grip();
        }


        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                SR.setPosition(0.23);
//                SL.setPosition(0.77);
//                sleep(250);
//                Gripper.setPosition(0.8);
//                Smid.setPosition(0.67);
//                SRG.setPosition(0.15);
                return false;
            }
        }


        public Action releases() {
            return new Re();
        }


        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                Gripper.setPosition(0.65);
//                Smid.setPosition(0.31);
//                SR.setPosition(0.31);
//                SL.setPosition(0.69);
//                Gripper.setPosition(0.2);
                return false;
            }
        }


        public Action mid() {
            return new Mid();
        }


        public class SlideFullUP implements Action {
            private double s4Position;
            private double s5Position;// position 0 = ยืดสุด


            public SlideFullUP(double s4Position,double s5Position) {
                this.s4Position = s4Position;
                this.s5Position = s5Position;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S5.setPosition(s5Position);//ดึงสไลด์กลับ
                S1.setPosition(1);
                S4.setPosition(s4Position);
                S0.setPosition(0.8);




                return false;
            }
        }


        public Action slideFullUP(double s4Position,double s5Position) {
            return new SlideFullUP(s4Position, s5Position);
        }










        public class SlideIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S5.setPosition(0.85);
                S1.setPosition(0);
                S4.setPosition(0.97);


                return false;
            }
        }


        public Action slideIN() {
            return new SlideIN();
        }
        public class Gripsampar implements Action {
            private double s0Position;


            public Gripsampar(double s0Position) {
                this.s0Position = s0Position;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(1);
                sleep(1000);


                S0.setPosition(0.2);


                return false;
            }
        }
        public Action gripsampar(double s0Position) {
            return new Gripsampar(s0Position);
        }




        public class Gripsamperp implements Action {
            private double s0Position;


            public Gripsamperp(double s0Position) {
                this.s0Position = s0Position;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);
                sleep(1000);


                S0.setPosition(0.2);


                return false;
            }
        }
        public Action gripsamperp(double s0Position) {
            return new Gripsamperp(s0Position);
        }


        public class Lewy implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {




                S0.setPosition(0.55);
//                sleep(245);
//                S0.setPosition(0.875);
//                S5.setPosition(0.8);
//                S1.setPosition(0.5);
//                sleep(150);
//                //S1.setPosition(0);
//                S4.setPosition(0.5);
                return false;
            }
        }


        public Action lewan() {
            return new Lewy();
        }
        public  class Test0 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                S0.setPosition(0.2);//กด
                return false;
            }




        }
        public Action test0(){
            return new Test0();
        }
        public  class Test1 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                S0.setPosition(0.55);//ดึง
                return false;
            }




        }
        public Action test1(){
            return new Test1();
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
                Gripper.setPosition(0.08);
                sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.315);
                SR.setPosition(0.685);
                Gripper.setPosition(0.165);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }

        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.08);
                sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.308);
                SR.setPosition(0.692);
                Gripper.setPosition(0.165);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }

        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.212);
                SL.setPosition(0.788);
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
                S1.setPosition(0);
                S0.setPosition(1);
                S5.setPosition(0.55);
                return false;
            }
        }
        public Action slideFullUP() {
            return new SlideFullUP();
        }
        public class SlideIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S5.setPosition(0.855);
                S1.setPosition(0);
                S4.setPosition(0.97);
                return false;
            }
        }
        public Action slideIN() {
            return new SlideIN();
        }


        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.065);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
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
                Gripper.setPosition(0.1);
                SRG.setPosition(0.73);
                SR.setPosition(0.57);
                SL.setPosition(0.43);
                Smid.setPosition(0.98);
                return false;
            }
        }
        public Action setY() {
            return new SetY();
        }


        public class ReGrip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.4);
                SL.setPosition(0.6);
                sleep(150);
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
        public Action reGrip() {
            return new ReGrip();
        }
        public class ReGrip2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.12);
                return false;
            }
        }
        public Action reGrip2() {
            return new ReGrip2();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d second = new Pose2d(30.75,0,Math.toRadians(0));
        Pose2d third = new Pose2d(5.2,-39,Math.toRadians(0));
        Pose2d forth = new Pose2d(30.75,-2,Math.toRadians(0));
        Pose2d forthfixed = new Pose2d(25,-2,Math.toRadians(0));
        Pose2d fifth = new Pose2d(7,-39,Math.toRadians(0));
        Pose2d fifth2 = new Pose2d(6,-39,Math.toRadians(0));
        Pose2d six = new Pose2d(30.75,-3,Math.toRadians(0));
        Pose2d sev = new Pose2d(30.75,-4,Math.toRadians(0));
        Pose2d exx = new Pose2d(30.75,-5,Math.toRadians(0));
        Pose2d sam6 = new Pose2d(19,65,Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);
        Mission2 mission2 = new Mission2(hardwareMap);
        Servo light = hardwareMap.get(Servo.class, "light");
        Servo Sllc = hardwareMap.get(Servo.class, "Sllc");
        Sllc.setPosition(0.98);
        light.setPosition(1);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(30.75,0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-60,80));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                //.strafeTo(new Vector2d(26,-8),null,new ProfileAccelConstraint(-100,100))
                .splineToSplineHeading(new Pose2d(26,-0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-100,100))
                .splineToConstantHeading(new Vector2d(24,-20),-Math.PI*0.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(30.5,-30,-Math.PI/3.5 ),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(18 ,-35.5,-Math.PI*4.3/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-34.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(33,-40,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-40,-Math.PI*4.5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-42,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(33,-45.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-46,-Math.PI*4.5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(5.2 ,-39,-Math.PI*2),-Math.PI*1.5,new TranslationalVelConstraint(90));

        TrajectoryActionBuilder Tosam2 = drive.actionBuilder (third)
                .strafeTo(new Vector2d(30.75,-2),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam3 = drive.actionBuilder (forth)
                .splineToConstantHeading(new Vector2d(25,-2),Math.PI*2,new TranslationalVelConstraint(80));

        TrajectoryActionBuilder Tosam3fix = drive.actionBuilder (forthfixed)
                .strafeTo(new Vector2d(7,-39),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam4 = drive.actionBuilder (fifth)
                .strafeTo(new Vector2d(30.75,-3),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam5 = drive.actionBuilder (six)
                .strafeTo(new Vector2d(7,-39),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam6 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(30.75,-4),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam7 = drive.actionBuilder (sev)

                .strafeTo(new Vector2d(7,-39),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam8 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(30.75,-5),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam9 = drive.actionBuilder (exx)

                .strafeTo(new Vector2d(6,-39),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam10 = drive.actionBuilder (fifth2)
                .strafeTo(new Vector2d(17,64.5),new TranslationalVelConstraint(100))
                .turnTo(Math.toRadians(-21));

        TrajectoryActionBuilder Tosam11 = drive.actionBuilder (sam6)
                .strafeTo(new Vector2d(0,-30),new TranslationalVelConstraint(100))
                .strafeTo(new Vector2d(6,-40),new TranslationalVelConstraint(100));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(3); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0); // Switch to pipeline number 0








        Actions.runBlocking(mission.reGrip2());
        Actions.runBlocking(mission.slideIN());
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
        Action Sam11;




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
        Sam11 = Tosam11.build();
        int s=242;

        double[] distances = {32, 29.4, 27.6, 25.7, 23.6, 21.6, 19, 16.5, 13.8, 11, 8.2, 5.6, 3.5, 1.9};
        double[] servoPositions = {0.05, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85};
        double[] limelightitp = {226, 202, 176, 154, 129, 107, 89, 71, 54, 45,36};
        double[] cm = {12, 15, 18, 21, 24, 27, 30, 33, 36, 39,42};
        double[] limelightitp2 = {235,190,154,120};
        double[] cm2 = { 15, 20, 25, 30};
        double[] cmx = {10,40};
        double[] xsetoff = {228,206};

        double[] arrx = {-3, -2, -1, 0, 1, 2, 3, .4, 5, 6, 7, 8};
        double[] limex = {300 - s, 283 - s, 278 - s, 263 - s, 242 - s, 224 - s, 184 - s, 162 - s, 142 - s, 120 - s, 102 - s, 82 - s};
        limelight.start();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                mission.set(),
                                new SequentialAction(
                                        new SleepAction(0.35),
                                        Middle,//first speciment Trjectory Moving Path
                                        mission.set() //Set Top gripper Ready to ng speciment
                                )
                        ),
                        new ParallelAction(
                                mission.releases()
                        )
                )
        );
        //double ty = 0;

        limelight.pipelineSwitch(4);


        LLResult result = limelight.getLatestResult();
        double[] pythonOutput= result.getPythonOutput();

        double ax = pythonOutput[1],ty = pythonOutput[2],ta = pythonOutput[3];

        if (result != null) {

            pythonOutput = result.getPythonOutput();
            //double ty = pythonOutput[2];




//           LLResult result = limelight.getLatestResult();
//           double[] pythonOutput = result.getPythonOutput();

            telemetry.addData("Limelight", ax);
            telemetry.addData("Limelight", ty);
            telemetry.addData("Limelight", ta);
            telemetry.update();
            ITPS = new ServoInterpolator(limelightitp, cm);
            double Xcm = ITPS.interpolatePosition(ty);
            ITPS = new ServoInterpolator(cmx,xsetoff);
            double Xoffset = ITPS.interpolatePosition(Xcm);
            double kx = 0.035;
            double ky = 0.045;


            double fX = ax - Xoffset; //pre calculate the position
            double valY = (ty - 168) * ky;
            ITPS = new ServoInterpolator(limex, arrx);
            double valX = ITPS.interpolatePosition(fX);

            if (ax == 0 && ty == 0) valX = 0; //if image not found, don't move
            TrajectoryActionBuilder limelight2 = drive.actionBuilder(second)
                        //.waitSeconds(500)
                .strafeTo(new Vector2d(30.75, valX));



                //.waitSeconds(0.5);


            Action T0LL2;
            T0LL2 = limelight2.build();


            Actions.runBlocking(
                    new SequentialAction(
                            T0LL2
                    )


            );
            result = limelight.getLatestResult();
            pythonOutput = result.getPythonOutput();
            double tx = pythonOutput[1];
                //telemetry.addData("Limelight", targetx);
            tx = pythonOutput[1];
            telemetry.addData("Limelight", tx);
            telemetry.update();
            sleep(500);
            double fix1, fix2 = 0;
            ITPS = new ServoInterpolator(cm, limelightitp);
            fix1 = ITPS.interpolatePosition(fix2);
        } else {
            telemetry.addData("Result", "No Result");
            telemetry.update();
        }

        result = limelight.getLatestResult();
        pythonOutput = result.getPythonOutput();
        double ny = pythonOutput[2];

        //sleep(50);
        //double ta = pythonOutput[3];
        ITPS = new ServoInterpolator(limelightitp2, cm2);


        double cmy = ITPS.interpolatePosition(ty);


        ITPS = new ServoInterpolator(distances, servoPositions);
        double targetServoPosition = ITPS.interpolatePosition(cmy);

        double angle = 0;
        if(ta>112.5 && ta<157.5)angle=0.25;
        else if(ta>67.5 && ta<112.5)angle=1;
        else if(ta>22.5 && ta<67.5)angle=0.75;
        else angle=0.5;
        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                                mission2.slideFullUP(angle, targetServoPosition),
                                new SleepAction(2),
                                mission2.test0(),
                                new SleepAction(10)

                        )




                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                Sam1,
                                new SequentialAction(
                                        new SleepAction(0.85),
                                        mission.armDown()
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
                        new ParallelAction(
                                Sam10,
                                mission.grip(),
                                lift.liftUp(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        mission.setY()
                                )
                        ),
                        mission.reGrip(),
                        new ParallelAction(
                                lift.liftDown(),

                                new SequentialAction(
                                        new SleepAction(0.5),
                                        Sam11
                                )
                        ),
                        new SleepAction(1)

                )
        );



    }
}