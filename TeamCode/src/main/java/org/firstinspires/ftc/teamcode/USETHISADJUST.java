package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp
public class USETHISADJUST extends LinearOpMode {
   
    double Speed = 1;
    double currentSpeed;
    double slowSpeed = 0.2;
    double normalSpeed = 1;
    double F = 18.6;
    double P = 30.2;

    private DcMotor LFront;
    private DcMotor LBack;
    private DcMotor RFront;  
    private DcMotor RBack;
    private DcMotor Intake;
    private DcMotorEx FlyWheel;
    private DcMotor AscensionLeft;
    private DcMotor AscensionRight;
    private Servo PaddleLeft;
    private Servo PaddleRight;
    private CRServo ConveyorLeft;
    private CRServo ConveyorRight;
    private HuskyLens huskyLens;
   
    @Override
    public void runOpMode() throws InterruptedException {
       
        // say motors exist
        LFront = hardwareMap.dcMotor.get("LFront");
        LBack = hardwareMap.dcMotor.get("LBack");
        RFront = hardwareMap.dcMotor.get("RFront");
        RBack = hardwareMap.dcMotor.get("RBack");
        Intake = hardwareMap.dcMotor.get("Intake");
        FlyWheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
        AscensionLeft = hardwareMap.dcMotor.get("AscensionLeft");
        AscensionRight = hardwareMap.dcMotor.get("AscensionRight");
        PaddleLeft = hardwareMap.servo.get("PaddleLeft");
        PaddleRight = hardwareMap.servo.get("PaddleRight");
        ConveyorLeft = hardwareMap.crservo.get("ConveyorLeft");
        ConveyorRight = hardwareMap.crservo.get("ConveyorRight");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        VoltageSensor controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AscensionLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AscensionRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RBack.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        
        waitForStart();
       
        while (opModeIsActive()) {
       
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
           
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            //x = x * 0.50;

            /*if (gamepad1.left_bumper) {
            Speed = 1;
            } else if (gamepad1.right_bumper) {
            Speed = 0.3;
            } else {
            Speed = 0.7;
            }*/
           
            double LFrontP = (y - x - rx) / denominator * Speed;
            double LBackP = (y + x - rx) / denominator * Speed;
            double RFrontP = (y + x + rx) / denominator * Speed;
            double RBackP = (y - x + rx) / denominator * Speed;
           
            LFront.setPower(LFrontP);
            LBack.setPower(LBackP);
            RFront.setPower(RFrontP);
            RBack.setPower(RBackP);
            
            if (gamepad2.a) {
                 Intake.setPower(1);
             } else {
                 Intake.setPower(0);
             }
            
            //Flywheel Power Up for Positioning
            if (gamepad2.dpad_down) {
                FlyWheel.setVelocity(1350);
            } else if (gamepad2.dpad_up) {
                FlyWheel.setVelocity(1640);
            } else if (gamepad2.b) {
                FlyWheel.setVelocity(0);
            } else if (gamepad2.left_trigger > 0.2) {
                FlyWheel.setVelocity(-250);
                ConveyorLeft.setPower(0.70);
                ConveyorRight.setPower(-0.70);
            } else if (gamepad2.x) {
                ConveyorLeft.setPower(-1);
                ConveyorRight.setPower(1);
            } else if (gamepad2.y) {
                ConveyorLeft.setPower(0);
                ConveyorRight.setPower(0);
            } else if (gamepad2.dpad_left) {
                ConveyorLeft.setPower(0.70);
                ConveyorRight.setPower(-0.70);
                Intake.setPower(-1);
            } else {
                Intake.setPower(0);
            }
            
            if (gamepad2.dpad_right) {
                PaddleLeft.setPosition(0);
                PaddleRight.setPosition(0.645);
            } else {
                PaddleLeft.setPosition(0.65);
                PaddleRight.setPosition(0);
            }
            
            if (gamepad2.left_bumper) {
                AscensionLeft.setPower(0.955);
                AscensionRight.setPower(-0.998);
            } else if (gamepad2.right_bumper) {
                AscensionLeft.setPower(-1);
                AscensionRight.setPower(0.969);
            } else {
                AscensionLeft.setPower(0);
                AscensionRight.setPower(0);
            }
            
            telemetry.addData("speed", Speed);
            telemetry.update();
        }
    }
}