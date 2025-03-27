package org.firstinspires.ftc.teamcode.Teleop;

import android.text.LoginFilter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp(name = "ATeleopManual", group = "TeleOp")

public class Teleop extends LinearOpMode {

    public Servo S0;
    public Servo S1;
    public Servo S4;
    public Servo S5;
    private Servo Smid;
    private Servo SR;
    private Servo SL;
    private Servo SRG;
    private Servo Gripper;
    private DcMotor liftR;
    private DcMotor liftL;

    private Servo Sarm;
    public int LiftReference;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-63, -3.5, Math.toRadians(0));
        SRG = hardwareMap.get(Servo.class, "SRG");
        S0 = hardwareMap.get(Servo.class, "S0");
        S1 = hardwareMap.get(Servo.class, "S1");
        S5 = hardwareMap.get(Servo.class, "S5");
        S4 = hardwareMap.get(Servo.class, "S4");
        Smid = hardwareMap.get(Servo.class, "midGrip");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        liftR = hardwareMap.get(DcMotor.class, "liftMotorR");
        liftL = hardwareMap.get(DcMotor.class, "liftMotorL");
        Sarm = hardwareMap.get(Servo.class, "Sarm");
        LiftReference = liftL.getCurrentPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap,  new Pose2d(0, 0, 0));
        S0.setPosition(0.75);
        S5.setPosition(0.8);
        S1.setPosition(0);
        S4.setPosition(0.5);
        Sarm.setPosition(0.59);
        double spy = 0;
        double spx = 0;
        double spr = 0;
        double pickPost = 0.5;
        double count = 0;
        boolean lastPressDPadLeft = false;
        boolean lastPressDPadRight = false;

        telemetry.addData(">", "Press Start");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            if ( gamepad1.left_stick_y>=0 )
                if (gamepad1.left_stick_y<0.05)
                    spy = 0;
                else if (gamepad1.left_stick_y< 0.6)
                    spy = 0.2;
                else
                    spy = Math.pow(gamepad1.left_stick_y,5)+0.2;
            else
            if (gamepad1.left_stick_y>-0.05)
                spy = 0;
            else if (gamepad1.left_stick_y> -0.6)
                spy = -0.2;
            else
                spy = Math.pow(gamepad1.left_stick_y,5)-0.2;

            if ( gamepad1.left_stick_x>=0 )
                if (gamepad1.left_stick_x<0.05)
                    spx = 0;
                else if (gamepad1.left_stick_x< 0.6)
                    spx = 0.2;
                else
                    spx = Math.pow(gamepad1.left_stick_x,5)+0.2;
            else
            if (gamepad1.left_stick_x>-0.05)
                spx = 0;
            else if (gamepad1.left_stick_x> -0.6)
                spx = -0.2;
            else
                spx = Math.pow(gamepad1.left_stick_x,5)-0.2;;

            if ( gamepad1.right_stick_x>=0 )
                if (gamepad1.right_stick_x<0.05)
                    spr = 0;
                else if (gamepad1.right_stick_x< 0.6)
                    spr = 0.2;
                else if (gamepad1.right_stick_x< 0.85)
                    spr = 0.25;
                else
                    spr = 1;
            else
            if (gamepad1.right_stick_x>-0.1)
                spr = 0;
            else if (gamepad1.right_stick_x> -0.6)
                spr = -0.2;
            else if (gamepad1.right_stick_x> -0.85)
                spr = -0.25;
            else
                spr = -1;
            // End Steering

            telemetry.addData("> : LeftStick", "Y: %.0f%%   X: %.0f%%", gamepad1.left_stick_y*100 , gamepad1.left_stick_x*100);
            telemetry.addData("> : RightStick", "R: %.0f%% ", gamepad1.right_stick_x*100);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -spy,
                            -spx
                    ),
                    -spr
            ));

            drive.updatePoseEstimate();
            if (gamepad1.dpad_up) {
                S0.setPosition(1);
            } else if (gamepad1.dpad_down) {

                S0.setPosition(0.42);
                Thread.sleep(245);
                S0.setPosition(0.7);
                S5.setPosition(0.805);
                S1.setPosition(0.5);
                Thread.sleep(150);
                S1.setPosition(0);
                S4.setPosition(1);
            }
            if (gamepad1.dpad_left) {
                S4.setPosition(0.97);
            } else if (gamepad1.dpad_right) {
                S4.setPosition(0.5);
            }
            // Adjust Picker Position
            if (gamepad2.dpad_left && !lastPressDPadLeft) {
                pickPost -= 0.249;
                if (pickPost <= 0) {
                    pickPost = 0.5;
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
//                    gamepad1.rumble(500);
//                    gamepad2.rumble(500);
                }
                S4.setPosition(pickPost);

            } else if (gamepad2.dpad_right && !lastPressDPadRight) {
                pickPost += 0.249;
                if (pickPost >= 1) {
                    pickPost = 0.5;
//                        gamepad1.rumble(500);
//                        gamepad2.rumble(500);
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                }
                S4.setPosition(pickPost);
            }
            lastPressDPadLeft = gamepad2.dpad_left;
            lastPressDPadRight = gamepad2.dpad_right;
            //----------------


            int posL = liftL.getCurrentPosition();
            telemetry.addData("Position bef", posL);
            telemetry.update();
            LiftReference = liftL.getCurrentPosition();

            if (gamepad1.left_bumper) {
                S4.setPosition(0.5);
                S1.setPosition(1);
                S0.setPosition(1);
                Thread.sleep(100);
                S5.setPosition(0);
            } else if (gamepad1.right_bumper){
                S5.setPosition(0.85);
                Thread.sleep(200);
                S1.setPosition(0);
                S4.setPosition(0.5);
            } else if (gamepad2.left_bumper) {
                SR.setPosition(0.21);
                SL.setPosition(0.79);
                Thread.sleep(300);
                Gripper.setPosition(0.8);
                Smid.setPosition(0.65);
                SRG.setPosition(0.15);

            } else if (gamepad2.right_bumper){
                Gripper.setPosition(0.8);
            } else if (gamepad1.b || gamepad2.b){
                SL.setPosition(0.135);
                SR.setPosition(0.865);
                Thread.sleep(50);
                Gripper.setPosition(0.1);
                S0.setPosition(1);
                Thread.sleep(200);
                SL.setPosition(0.5);
                SR.setPosition(0.5);
                // Rot 0
                SRG.setPosition(0.15);
            }else if (gamepad1.a || gamepad2.a) {
                SR.setPosition(0.23);
                SL.setPosition(0.77);
                Gripper.setPosition(0.8);
                Smid.setPosition(0.67);
                SRG.setPosition(0.2);
            } else if (gamepad1.y || gamepad2.y) {
                Gripper.setPosition(0.1);
                Thread.sleep(70);
                Smid.setPosition(0.72);
                SL.setPosition(0.30);
                SR.setPosition(0.70);
                Gripper.setPosition(0.215);
            } else if (posL > 2500) {
                SRG.setPosition(0.73);
                Smid.setPosition(0.98);
                SR.setPosition(0.53);
                SL.setPosition(0.47);
            } else if (gamepad1.x || gamepad2.x){
                SRG.setPosition(0.73);
                Gripper.setPosition(0.7);
                Smid.setPosition(0.35);
                SL.setPosition(0.2);
                SR.setPosition(0.8);
            }


            if (gamepad1.left_trigger > 0.2 || gamepad2.dpad_down){
                liftR.setPower(1);
                liftL.setPower(-1);
            } else if (gamepad1.right_trigger > 0.2 || gamepad2.dpad_up) {
//                Smid.setPosition(0);
                liftR.setPower(-1);
                liftL.setPower(1);
//                Thread.sleep(550);
//                Gripper.setPosition(0.8);
            }else {

                liftR.setPower(0.08);
                liftL.setPower(0.08);
            }



//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            count = count + 1;
            telemetry.addData("> : count ", count);
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}