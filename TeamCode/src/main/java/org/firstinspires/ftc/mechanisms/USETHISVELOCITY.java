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
public class USETHISVELOCITY extends LinearOpMode {
   
    double Speed = 1;
    double currentSpeed;
    double slowSpeed = 0.2;
    double normalSpeed = 1;
    double F = 25.6;
    double P = 37.2;

    private DcMotor LFront;
    private DcMotor LBack;
    private DcMotor RFront;  
    private DcMotor RBack;
    private DcMotor Intake;
    private DcMotorEx FlyWheelLeft;
    private DcMotorEx FlyWheelRight;
    //private DcMotor AscensionLeft;
    //private DcMotor AscensionRight;
    private Servo Paddle;
    private HuskyLens huskyLens;
    
   
    @Override
    public void runOpMode() throws InterruptedException {
       
        // say motors exist
        LFront = hardwareMap.dcMotor.get("LFront");
        LBack = hardwareMap.dcMotor.get("LBack");
        RFront = hardwareMap.dcMotor.get("RFront");
        RBack = hardwareMap.dcMotor.get("RBack");
        Intake = hardwareMap.dcMotor.get("Intake");
        FlyWheelLeft = hardwareMap.get(DcMotorEx.class, "FlyWheelLeft");
        FlyWheelRight = hardwareMap.get(DcMotorEx.class, "FlyWheelRight");
        //AscensionLeft = hardwareMap.dcMotor.get("AscensionLeft");
        //AscensionRight = hardwareMap.dcMotor.get("AscensionRight");
        Paddle = hardwareMap.servo.get("Paddle");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        VoltageSensor controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //AscensionLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //AscensionRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RBack.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyWheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyWheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlyWheelLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        FlyWheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
             } else if (gamepad2.y) {
                 Intake.setPower(-0.5);
             } else {
                 Intake.setPower(0);
             } 
            
            //Flywheel Power Up for Positioning
            if (gamepad2.dpad_down) {
                FlyWheelLeft.setVelocity(700);
                FlyWheelRight.setVelocity(700);
            } else if (gamepad2.dpad_up) {
                FlyWheelLeft.setVelocity(1650);
                FlyWheelRight.setVelocity(1650);
            } else if (gamepad2.b) {
                FlyWheelLeft.setVelocity(0);
                FlyWheelRight.setVelocity(0);
            } else if (gamepad2.left_trigger > 0.1) {
                FlyWheelLeft.setVelocity(-400);
                FlyWheelRight.setVelocity(-400);
            } else {
                //Intake.setVelocity(0);
            }
            
            if (gamepad2.x) {
                Paddle.setPosition(0.40);
            } else {
                Paddle.setPosition(1);
            }
            
            /*if (gamepad2.left_bumper) {
                AscensionLeft.setPower(0.955);
                AscensionRight.setPower(-0.998);
            } else if (gamepad2.right_bumper) {
                AscensionLeft.setPower(-1);
                AscensionRight.setPower(0.969);
            } else {
                AscensionLeft.setPower(0);
                AscensionRight.setPower(0);
            } */
            
            telemetry.update();
        }
    }
    }
