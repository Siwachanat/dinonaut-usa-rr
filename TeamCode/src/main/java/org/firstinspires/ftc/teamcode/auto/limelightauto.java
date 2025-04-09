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

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;



@Config
@Autonomous(name = "Limelight Auto", group = "Autonomous")
public class limelightauto extends LinearOpMode {
    Limelight3A limelight;

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
                Gripper.setPosition(0.08);
                sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.3225);
                SR.setPosition(0.6675);
                Gripper.setPosition(0.17);
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
                SL.setPosition(0.3225);
                SR.setPosition(0.6675);
                Gripper.setPosition(0.17);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }

        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.215);
                SL.setPosition(0.785);
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

        public class Slidebyrange implements Action {
            private double s0Position;// position 0 = ยืดสุด

            public Slidebyrange(double s0Position) {
                this.s0Position = s0Position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);
                S1.setPosition(0);
                S0.setPosition(1);
                S5.setPosition(s0Position);
                return false;
            }
        }

        public Action slidebyrange(double s0Position) {
            return new Slidebyrange(s0Position);
        }

        public class Gripsampar implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(1);

                return false;
            }
        }
        public Action gripsampar() {
            return new Gripsampar();
        }


        public class Gripsamperp implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);

                return false;
            }
        }
        public Action gripsamperp() {
            return new Gripsamperp();
        }

        public class Gripdown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                S0.setPosition(0.55);

                return false;
            }
        }
        public Action gripdown() {
            return new Gripdown();
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
                S5.setPosition(0.85);
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
    }



    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10); // This sets how often we ask Limelight for data (100 times per second)
        // This tells Limelight to start looking!
        limelight.start();
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d second = new Pose2d(30.75,0,Math.toRadians(0));
        Pose2d third = new Pose2d(5.5,-35,Math.toRadians(0));
        Pose2d forth = new Pose2d(30.75,-2,Math.toRadians(0));
        Pose2d forthfixed = new Pose2d(25,-2,Math.toRadians(0));
        Pose2d fifth = new Pose2d(6.2,-35,Math.toRadians(0));
        Pose2d fifth2 = new Pose2d(6.2,-35,Math.toRadians(0));
        Pose2d six = new Pose2d(30.75,2,Math.toRadians(0));
        Pose2d sev = new Pose2d(30.75,-4,Math.toRadians(0));
        Pose2d exx = new Pose2d(30.75,-3,Math.toRadians(0));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);
        GBlight gBlight = new GBlight(hardwareMap);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(30.75,0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-60,80));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                //.strafeTo(new Vector2d(26,-8),null,new ProfileAccelConstraint(-100,100))
                .splineToSplineHeading(new Pose2d(26,-0,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-100,100))
                .splineToConstantHeading(new Vector2d(24,-16),-Math.PI*0.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(30.5,-30,-Math.PI/3.5 ),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(18 ,-35.5,-Math.PI*4.3/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-34.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(33,-40,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-40,-Math.PI*4.5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(30,-42,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(33,-45.5,-Math.PI/3.5 ),-Math.PI*0.3,new TranslationalVelConstraint(100))
                .splineToLinearHeading(new Pose2d(21,-46,-Math.PI*4.5/6),-Math.PI*1.5,new TranslationalVelConstraint(100))

                .splineToLinearHeading(new Pose2d(5.5 ,-35,-Math.PI*2),-Math.PI*1.5,new TranslationalVelConstraint(90));

        TrajectoryActionBuilder Tosam2 = drive.actionBuilder (third)
                .strafeTo(new Vector2d(30.75,-2),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam3 = drive.actionBuilder (forth)
                .splineToConstantHeading(new Vector2d(25,-2),Math.PI*2,new TranslationalVelConstraint(80));

        TrajectoryActionBuilder Tosam3fix = drive.actionBuilder (forthfixed)
                .strafeTo(new Vector2d(6.2,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam4 = drive.actionBuilder (fifth2)
                .strafeTo(new Vector2d(30.75,2),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam5 = drive.actionBuilder (six)
                .strafeTo(new Vector2d(6.2,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam6 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(30.75,-4),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam7 = drive.actionBuilder (sev)

                .strafeTo(new Vector2d(6.2,-35),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam8 = drive.actionBuilder (fifth)

                .strafeTo(new Vector2d(30.75,-3),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam9 = drive.actionBuilder (exx)

                .strafeTo(new Vector2d(6.2,-45),new TranslationalVelConstraint(100));

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
                                gBlight.lset1(),
                                Middle,//first speciment Trjectory Moving Path
                                mission.set(), //Set Top gripper Ready to ng speciment
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        mission.releases()
                                )
                        )

                )
        );

//
        LLResult result = limelight.getLatestResult();
        double[] pythonOutput = result.getPythonOutput();

        double tx = pythonOutput[1];
        double ty = pythonOutput[2];
        double ta = pythonOutput[3];
//        telemetry.addData("Limelight", " python [1] " + tx);
//        telemetry.addData("Limelight", " python [2] " + ty);
//        telemetry.addData("Limelight", " python [3] " + ta);
        telemetry.update();
        double kx = 0.045 ;
        double ky = 0.045;
        int adjX = 220, adjY=56;
        double valX = (tx-adjX)*kx; //pre calculate the position
        //double valY = (ty-adjY)*ky;
        if(tx==0 && ty==0)valX=0; //if image not found, don't move

        TrajectoryActionBuilder limelight2 = drive.actionBuilder(second)
                //.waitSeconds(500)
                .strafeTo(new Vector2d(30.75, -valX))
                .waitSeconds(0.5);
        Pose2d ll = new Pose2d(30.75,-valX,Math.toRadians(0));

        Action T0LL2;
        T0LL2 = limelight2.build();

        Actions.runBlocking(
                new SequentialAction(

                        T0LL2
                )
        );




//
//
        result = limelight.getLatestResult();
        pythonOutput = result.getPythonOutput();
        double tx2 = pythonOutput[1];
        double ty2 = pythonOutput[2];
        //ta = pythonOutput[3];
//        telemetry.addData("Limelight", " python [1] " + tx2);
//        telemetry.addData("Limelight", " python [2] " + ty2);
        //telemetry.addData("Limelight", " python [3] " + ta);
        telemetry.update();
        kx = 0.045 ;
        ky = 0.045;
        //double valX = (tx-adjX)*kx; //pre calculate the position
        // double valY = (ty-adjY)*ky;

        double valX2 = (tx2-adjX)*kx; //pre calculate the position
        //double valY2 = (ty2-adjY)*ky*0;
        if(tx2==0 && ty2==0)valX2=0; //if image not found, don't move
        TrajectoryActionBuilder limelight3 = drive.actionBuilder(ll)
                .strafeTo(new Vector2d(30.75,-valX2))
                //                        .strafeTo(new Vector2d(0,0.1))
                .waitSeconds(0.5);
        TrajectoryActionBuilder backtosecond = drive.actionBuilder(ll)
                .strafeTo(new Vector2d(30.75,0))
                //                        .strafeTo(new Vector2d(0,0.1))
                .waitSeconds(0.5);
        //                        .strafeTo(new Vector2d(0,10));

        Action T0LL3;
        T0LL3 = limelight3.build();

        Action BTS;
        BTS = backtosecond.build();



        Actions.runBlocking(
                new SequentialAction(

                        T0LL3

                )
        );


        double ny=0;

        for(int i=0; i<10; i++){
            result = limelight.getLatestResult();
            pythonOutput = result.getPythonOutput();
            if(pythonOutput[2]!=0) {
                if (ny != 0) ny = (ny + pythonOutput[2]) / 2;
                else ny = pythonOutput[2];
            }
            sleep(50);
        }

        double valY=1.6-(165-ny)/142.5;
        double valY2=1.8-(215-ny)/180;
        double valY3=1.7-((210-ny)/200)*1.2;
        double valY4=1.65-((220-ny)/220)*0.7;
        if(ny==0)valY=0;
        //double ta = pythonOutput[3];
//        telemetry.addData("raw_ny: ",ny);
//        telemetry.addData("servo_angle",valY);
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        (95 < ny && ny < 140) ? // ไกล
                                new SequentialAction(
                                        mission.slidebyrange(valY),
                                        new SleepAction(1)
                                ) :
                                (140 <= ny && ny < 170) ? // กลาง
                                        new SequentialAction(
                                                mission.slidebyrange(valY2),
                                                new SleepAction(1)
                                        ) :
                                        (ny <= 95 || ny == 0) ? // ไกลสุด
                                                new SequentialAction(
                                                        mission.slidebyrange(0),
                                                        new SleepAction(1)
                                                ) :
                                                (170 <= ny && ny < 190) ? // ใกล้
                                                        new SequentialAction(
                                                                mission.slidebyrange(valY3),
                                                                new SleepAction(1)
                                                        ) :
                                                        new SequentialAction( // ใกล้สุด
                                                                mission.slidebyrange(valY4),
                                                                new SleepAction(1)
                                                        ),
                        (45<ta && ta<135) ?
                                new SequentialAction(
                                        mission.gripsamperp(),
                                        new SleepAction(1)
                                ) :
                                        new SequentialAction(
                                                mission.gripsampar(),
                                                new SleepAction(1)
                                        ),
                        new SleepAction(999),
                        mission.gripdown(),
                        mission.slideIN()
                )
        );


        Actions.runBlocking(
                new SequentialAction(
                        BTS,

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