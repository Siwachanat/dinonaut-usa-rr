
package org.firstinspires.ftc.teamcode.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.auto.ServoInterpolator;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


@Config

@Autonomous(name = "Limelight_Tuning", group = "Autonomous")
public class limelighttuning extends LinearOpMode {
    public Limelight3A limelight;
    public int adjX = 260;
    public ServoInterpolator ITPS;

    public class light {
        private Servo light;
        public light(HardwareMap hardwareMap) {
            light = hardwareMap.get(Servo.class, "light");
        }
        public class L1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                light.setPosition(1);
                return false;
            }
        }
        public class L0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                light.setPosition(0.0);
                return false;
            }
        }
        public Action l1() {
            return new L1();
        }
        public Action l0() {
            return new L0();
        }
    }

    public class GBlight {
        private Servo light;
        public GBlight(HardwareMap hardwareMap) {
            light = hardwareMap.get(Servo.class, "light");
        }
        public class LSet1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                light.setPosition(1);
                return false;
            }
        }
        public class LSet0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                light.setPosition(0.0);
                return false;
            }
        }
        public Action lset1() {
            return new LSet1();
        }
        public Action lset0() {
            return new LSet0();
        }
    }
//

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
            private double s0Position;// position 0 = ยืดสุด

            public SlideFullUP(double s0Position) {
                this.s0Position = s0Position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S5.setPosition(s0Position);//ดึงสไลด์กลับ
                S1.setPosition(1);
//        s4_hand_rot.setPosition(0.0);//par
                S0.setPosition(0.8);


                return false;
            }
        }

        public Action slideFullUP(double s0Position) {
            return new SlideFullUP(s0Position);
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

        public class Havertz implements Action{
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

        public Action havertz() {
            return new Havertz();
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



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        Servo s0_grip = hardwareMap.get(Servo.class, "S0");
        Servo s1_arm_rot = hardwareMap.get(Servo.class, "S1");
        Servo s5_arm_range = hardwareMap.get(Servo.class, "S5");
        Servo s4_hand_rot = hardwareMap.get(Servo.class, "S4");
        Servo light = hardwareMap.get(Servo.class,"light");
        Servo Sllc = hardwareMap.get(Servo.class,"Sllc");
        Sllc.setPosition(0.98);
        sleep(1000);
        s1_arm_rot.setPosition(0);

        s5_arm_range.setPosition(0.95);//ดึงสไลด์กลับ

        light.setPosition(1);
//        int ix=0;
//
//
//        while(ix<10000) {
//            LLResult result = limelight.getLatestResult();
//            double[] pythonOutput = result.getPythonOutput();
//            telemetry.addData("Limelight", pythonOutput[1]);
//            telemetry.addData("Limelight", pythonOutput[2]);
//            telemetry.addData("Limelight", pythonOutput[3]);
//            double tx = pythonOutput[1]-270;
//            double ty = pythonOutput[2]-169;
//            double ta = pythonOutput[3];
//
////            if (result != null && result.isValid()) {
////             // How far up or down the target is (degrees)
//////                double ta ;  // How big the target looks (0%-100% of the image)
//////
//////                telemetry.addData("Target X", tx);
//////                telemetry.addData("Target Y", ty);
//////                telemetry.addData("Target Area", ta);
////            } else {
////                telemetry.addData("Limelight", "No Targets" +ix);
////            }
//            telemetry.update();
//            sleep(300);
//            ix++;
//        }




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
        Mission2 mission2 = new Mission2(hardwareMap);
        GBlight gBlight = new GBlight(hardwareMap);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(31.5,0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-60,80));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                //.strafeTo(new Vector2d(26,-8),null,new ProfileAccelConstraint(-100,100))
                .splineToSplineHeading(new Pose2d(26,-0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-100,100))
                .splineToConstantHeading(new Vector2d(24,-16),-Math.PI*0.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(30.5,-30,-Math.PI/3.5 ),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(18 ,-35.5,-Math.PI*4.3/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-34.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(32,-40,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-40,-Math.PI*5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-42,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(32,-45.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-46,-Math.PI*5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

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







//
//        Actions.runBlocking(mission.set());
//        Actions.runBlocking(mission.slideIN());
//
//        Actions.runBlocking(mission.set());
//        Actions.runBlocking(mission.slideIN());


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
        int s = 242;

        int rnd = 0;
        limelight.start();
        light.setPosition(1);
//        double[] distances = {32,29.4,27.6,25.7,23.6,21.6,19,16.5,13.8,11,8.2,5.6,3.5,1.9};
//        double[] servoPositions = {0.05,0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85};
//        double[] limelightitp={226,202,176,154,129,107,89,71,54,45};
//        double[] cm ={12,15,18,21,24,27,30,33,36,39};
//        double[] xsetoff={255,255,253,251,245,242,240,237,233,230};
//        double[] arrx={-3,-2,-1,0,1,2,3,.4,5,6,7,8};
//        double[] limex={300-s,283-s,278-s,263-s,242-s,224-s,184-s,162-s,142-s,120-s,102-s,82-s};

        double[] distances = {6,8.5,11,12.5,14,18,21,23,25,26.5,28,30,32,32.5,33,33.5};
        double[] servoPositions = {0.9,0.85,0.8,0.75,0.7,0.65,0.6,0.55,0.5,0.45,0.4,0.35,0.3,0.25,0.2,0.15};
        double[] limelightitp={0.2203,0.1722,0.122,0.078,0.034,-0.0078,-0.0434};
        double[] cm ={15,18,21,24,27,30,33};
        double[] xsetoff={0.1942,0.1875,0.1875,0.1807,0.1784,0.1739,0.1693};
        double[] arrx={-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10};
        double[] limex={0.3007,0.2576,0.2124,0.1693,0.1263,0.081,0.0403,-0.0073,-0.0503,-0.0979,-0.1409,-0.1862,-0.2292,-0.2745};












        waitForStart();


        limelight.pipelineSwitch(0);

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //mission.slideFullUP(0.4)
                        gBlight.lset1(),
                        mission2.slideIN(),
                        //new SleepAction(999),

                        //mission.test1(),
                        new SleepAction(2)
                )
        );




        LLResult result = limelight.getLatestResult();
        double[] pythonOutput = result.getPythonOutput();
        double ty = pythonOutput[2];
        double h = 1;

        for (byte i = 0; i < 3; i++) {
//            LLResult result = limelight.getLatestResult();
//            double[] pythonOutput = result.getPythonOutput();
            double ax = pythonOutput[1];
            //double ty = pythonOutput[2];
            double ta = pythonOutput[3];
            telemetry.addData("Limelight", ax);
            telemetry.addData("Limelight",  ty);
            telemetry.addData("Limelight", ta);
            telemetry.update();
            ITPS =new ServoInterpolator(limelightitp,xsetoff);
            double Xsetoff = ITPS.interpolatePosition(ty);
//            ITPS =new ServoInterpolator(limex,arrx);
//            double valX = ITPS.interpolatePosition(ax);

            double kx = 0.035 ;
            double ky = 0.045;

            double fX = ax-0.1875; //pre calculate the position
            //double valY = (ty-adjY)*ky;
            ITPS =new ServoInterpolator(limex,arrx);
            double valX = ITPS.interpolatePosition(fX);
            if(ax==0 && ty==0)valX=0; //if image not found, don't move
            TrajectoryActionBuilder limelight2 = drive.actionBuilder(initialPose)
                    //.waitSeconds(500)
                    .strafeTo(new Vector2d(0, valX*h));
                    h=h/2;

                    //.waitSeconds(0.5);

            Action T0LL2;
            T0LL2 = limelight2.build();


            Actions.runBlocking(
                    new SequentialAction(

                            T0LL2
                            //gBlight.lset0()

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
            }
        double fix1,fix2=0;
        ITPS =new ServoInterpolator(cm,limelightitp);
        fix1 = ITPS.interpolatePosition(fix2);

//
//        for (byte i = 0; i < 2; i++) {
////            LLResult result = limelight.getLatestResult();
////            double[] pythonOutput = result.getPythonOutput();
//            double rx = pythonOutput[1];
//            double ty7 = pythonOutput[2];
//            double ta = pythonOutput[3];
//            telemetry.addData("Limelight", rx);
//            telemetry.addData("Limelight",  ty);
//            telemetry.addData("Limelight", ta);
//            telemetry.update();
//            ITPS =new ServoInterpolator(limelightitp,xsetoff);
//            double Xsetoff2 = ITPS.interpolatePosition(ty7);
////            ITPS =new ServoInterpolator(limex,arrx);
////            double valX = ITPS.interpolatePosition(ax);
//
//            double kx = 0.035 ;
//            double ky = 0.045;
//
//            double fX = rx-Xsetoff2; //pre calculate the position
//            //double valY = (ty-adjY)*ky;
//            ITPS =new ServoInterpolator(limex,arrx);
//            double valX7 = ITPS.interpolatePosition(fX);
//            if(rx==0 && ty==0)valX7=0; //if image not found, don't move
//            TrajectoryActionBuilder limelight2 = drive.actionBuilder(initialPose)
//                    //.waitSeconds(500)
//                    .strafeTo(new Vector2d(0, valX7))
//
//
//                    .waitSeconds(0.5);
//
//            Action T0LL2;
//            T0LL2 = limelight2.build();
//
//
//            Actions.runBlocking(
//                    new SequentialAction(
//
//                            T0LL2
//                            //gBlight.lset0()
//
//                    )
//
//            );
//            result = limelight.getLatestResult();
//            pythonOutput = result.getPythonOutput();
//            double tx = pythonOutput[1];
//            //telemetry.addData("Limelight", targetx);
//            tx = pythonOutput[1];
//            telemetry.addData("Limelight", tx);
//            telemetry.update();
//            sleep(500);
//        }






//
////
//        LLResult result = limelight.getLatestResult();
//        double[] pythonOutput = result.getPythonOutput();
//        double tx2 = pythonOutput[1];
//        double ty2 = pythonOutput[2];
//        //ta = pythonOutput[3];
//
//        //telemetry.addData("Limelight", " python [3] " + ta);
//        ITPS =new ServoInterpolator(limex,arrx);
//        double valX2 = ITPS.interpolatePosition(tx2);
////        ITPS =new ServoInterpolator(limelightitp,xsetoff);
////        double targetx2 = ITPS.interpolatePosition(ty2);
//        double kx = 0.02 ;
//        double ky = 0.005;
//
//        //double valX = (tx-adjX)*kx; //pre calculate the position
//        // double valY = (ty-adjY)*ky;
//
//        //double valX2 = (tx2-300-0)*kx; //pre calculate the position
//        //double valY2 = (ty2-adjY)*ky*0;
//
//        if(tx2==0 && ty2==0)valX2=0; //if image not found, don't move
//        TrajectoryActionBuilder limelight3 = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(0,valX2))
//                //                        .strafeTo(new Vector2d(0,0.1))
//                .waitSeconds(0.5);
//        //                        .strafeTo(new Vector2d(0,10));
//
//        Action T0LL3;
//        T0LL3 = limelight3.build();
//        tx2 = pythonOutput[1];
//
//
//        Actions.runBlocking(
//                new SequentialAction(
//
//                        T0LL3
//
//                )
//        );
//        //telemetry.addData("Limelight",);
//        telemetry.addData("Limelight", tx2);
//        telemetry.addData("Limelight", valX2);
//        telemetry.update();
        //sleep(3000);

//                result = limelight.getLatestResult();
//                pythonOutput = result.getPythonOutput();
//                double tx3 = pythonOutput[1];
//                double ty3 = pythonOutput[2];
//
//
//                telemetry.addData("Limelight", " python [1] " + tx3);
//                telemetry.addData("Limelight", " python [2] " + ty3);
//                telemetry.addData("Limelight", " python [3] " + ta);
//                telemetry.update();
//
//                double valX3 = (tx3-adjX)*kx; //pre calculate the position
//                double valY3 = (ty3-adjY)*ky*0;
//                if(tx3==0 && ty3==0) valY3=0; //if image not found, don't move
//                TrajectoryActionBuilder limelight4 = drive.actionBuilder(initialPose)
//                        .strafeTo(new Vector2d(0, -valX3))
//                        .waitSeconds(0.5);
//
//                //.waitSeconds(5);
//
//
//                Action T0LL4;
//                T0LL4 = limelight4.build();
//                Actions.runBlocking(
//                        new SequentialAction(
//                                T0LL4
//
//
//                        )
//                );
//        result = limelight.getLatestResult();
//        pythonOutput = result.getPythonOutput();
//        double ta = pythonOutput[3];

        double ny=0;





        for(int i=0; i<1000; i++){
            result = limelight.getLatestResult();
            pythonOutput = result.getPythonOutput();
            if(pythonOutput[2]!=0) {
                if (ny != 0) ny = (ny + pythonOutput[2]) / 2;
                else ny = pythonOutput[2];
            }
            //sleep(50);
        }
        double ta = pythonOutput[3];
        ITPS =new ServoInterpolator(limelightitp,cm);

        double cmy = ITPS.interpolatePosition(ty);

        ITPS = new ServoInterpolator(distances,servoPositions);
        double targetServoPosition = ITPS.interpolatePosition(cmy);

        double valY=(110-ny)*-0.023;
        double valY2=(0.8-(215-ny)/180);
        double valY3=(0.7-((210-ny)/200)*1.2);
        double valY4=(0.65-((220-ny)/220)*0.7);
        //double sy = (43.1-cm)/50.8;
        if(ny==0)valY=0;
        //double ta = pythonOutput[3];
        telemetry.addData("raw_ny: ",ny);
        telemetry.addData("servo_angle",valY);
        telemetry.update();


        Actions.runBlocking(
                new SequentialAction(

                        new SequentialAction(
                                mission2.slideFullUP(targetServoPosition), // Move more upwards if the arm is very high
                                new SleepAction(1)
                        ),


                        (45<ta && ta<135) ?
                                new SequentialAction(
                                        mission2.gripsampar(0),
                                        new SleepAction(1)
                                ) :
                                new SequentialAction(
                                        mission2.gripsamperp(0),
                                        new SleepAction(1)
                                ),

                        new SleepAction(1),
                        new SleepAction(20)
                )
        );



//        if (45<ta && ta<135) {
//            result = limelight.getLatestResult();
//            pythonOutput = result.getPythonOutput();
//
//            double ny = pythonOutput[2];
//            double adjS5 = 121-ny;
//            telemetry.addData("angle","sample"+ny);
//            telemetry.addData("angle","sample"+adjS5);
//            telemetry.update();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            //mission.gripsamperp((adjS5)),
//                            //mission.gripsamperp((0.4)),
//                            new SleepAction(1),
//                            mission.slideFullUP(0.4),
//                            //mission.slideFullUP((121-ny/82.5)),
//                            //mission.slideFullUP((0.4)),
//                            new SleepAction(1),
//                            //mission.test0(),
//                            new SleepAction(1),
//                            mission.slideIN(),
//                            new SleepAction(10)
//
//
//                    )
//            );
//
//        }
//        else {
//            result = limelight.getLatestResult();
//            pythonOutput = result.getPythonOutput();
//
//            double ny = pythonOutput[2];
//            //double ta = pythonOutput[3];
//
//            double adjS5 = 100-ny;
//            telemetry.addData("angle","sample"+ny);
//            telemetry.addData("angle","sample"+adjS5);
//            telemetry.update();
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            //mission.gripsampar((adjS5)),
//                            new SleepAction(1),
//                            //mission.gripsamperp((0.4)),
//
//                            //mission.slideFullUP(0.5),
//                            mission.slideFullUP((adjS5/-160)),
//                            //mission.slideFullUP((0.4)),
//                            new SleepAction(1),
//                            //mission.test0(),
//                            new SleepAction(1),
//                            mission.slideIN(),
//                            new SleepAction(10)
//
//
//                    )
//            );
//
//        }




//                Actions.runBlocking(
//                        new SequentialAction(
//
//                                mission.slideIN()
//
//
//                        )
//                );
//
//                Actions.runBlocking(
//                        new SequentialAction(
//
//                                T0LL3
//
//                        )
//                );


//                        new ParallelAction(T0LL2
//                                Middle,//first speciment Trjectory Moving Path
//                                mission.set() //Set Top gripper Ready to ng speciment
//                        ),
//                        new ParallelAction(
//                                mission.releases(),
//                                Sam1,
//                                new SequentialAction(
//                                        new SleepAction(0.85),
//                                        mission.armDown()
//                                )
//                        ),
//
//
//                        new ParallelAction(
//                                Sam2,
//                                mission.armUp(),
//                                mission.grip()
//                        ),
//                        new ParallelAction(
//                                mission.releases(),
//                                Sam3fix
//                        ),
//                        new ParallelAction(
//                                Sam4,
//                                mission.grip()
//
//                        ),
//                        new ParallelAction(
//                                mission.releases(),
//                                Sam5
//                        ),
//                        new ParallelAction(
//                                Sam6,
//                                mission.grip()
//
//                        ),
//                        new ParallelAction(
//                                mission.releases(),
//                                Sam7
//                        ),
//                        new ParallelAction(
//                                Sam8,
//                                mission.grip()
//
//                        ),
//                        new ParallelAction(
//                                Sam9,
//                                mission.releases()
//                        ),
////                        new ParallelAction(
////                                Sam10,
////                                mission.grip(),
////                                lift.liftUp(),
////                                new SequentialAction(
////                                        new SleepAction(0.85),
////                                        mission.setY()
////                                )
////                        ),
////                        mission.reGrip(),
//                        new SleepAction(1)
//
        //           )
        //)
//


    }
}