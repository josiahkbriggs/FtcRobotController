package org.firstinspires.ftc.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Joystick extends LinearOpMode {
    
    int AP = 0;
    int VP = 0;
    
    int maxRot = 2500;
    int minRot = -50;
    int rotSpeed = 2000;

    
    double slowSpeed = 0.2;
    double normalSpeed = 0.8;
    double currentSpeed;
    double VspeedL = 2500;
    double VspeedR = 2500;
    
 private DcMotor LFront;
 private DcMotor LBack;
 private DcMotor RFront;  
 private DcMotor RBack;
 private DcMotor Intake;
 private DcMotor AscensionLeft;
 private DcMotor AscensionRight;
 private DcMotorEx FlyWheel;
 private CRServo ConveyorLeft;
 private CRServo ConveyorRight;
 
    @Override
    public void runOpMode() throws InterruptedException {
        
        
        LFront = hardwareMap.dcMotor.get("LFront");
        LBack = hardwareMap.dcMotor.get("LBack");
        RFront = hardwareMap.dcMotor.get("RFront");
        RBack = hardwareMap.dcMotor.get("RBack");
        Intake = hardwareMap.dcMotor.get("Intake");
        AscensionLeft = hardwareMap.dcMotor.get("AscensionLeft");
        AscensionRight = hardwareMap.dcMotor.get("AscensionRight");
        FlyWheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
        CRServo ConveyorLeft = hardwareMap.crservo.get("ConveyorLeft");
        CRServo ConveyorRight = hardwareMap.crservo.get("ConveyorRight");

        /* Reverse the right side motors. This may be wrong for your setup.
         If your robot moves backwards when commanded to go forwards,
         reverse the left side instead.
         See the note about this earlier on this page.*/
        LFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RBack.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
            
         while (opModeIsActive()) {
            
            //Ramp up to Flywheel Launch
            if (gamepad2.x) {
                ConveyorLeft.setPower(-1);
                ConveyorRight.setPower(1);
            } else if (gamepad2.y) {
                ConveyorLeft.setPower(0);
                ConveyorRight.setPower(0);
            }
            
            if (gamepad1.left_bumper) {
                currentSpeed = slowSpeed;
            } else {
                currentSpeed = normalSpeed;
            }
            
            /*Denominator is the largest motor power or 1 this ensures 
            all the powers maintain the same ratio, but only when
            at least one is out of the range [-1, 1]*/
            double max;
            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double yaw = -gamepad1.left_stick_x * 1.1;
            double lateral = -gamepad1.right_stick_x;
            
            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
            
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double LFrontPower = axial + lateral + yaw;
            double RFrontPower = axial - lateral - yaw;
            double LBackPower = axial + lateral - yaw;
            double RBackPower = axial - lateral + yaw;
            
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(LFrontPower), Math.abs(RFrontPower));
            max = Math.max(max, Math.abs(LBackPower));
            max = Math.max(max, Math.abs(RBackPower));
            
            if (max > 1.0) {
                LFrontPower  /= max;
                RFrontPower /= max;
                LBackPower   /= max;
                RBackPower  /= max;
            }
            
            // Send calculated power to wheels
            LFront.setPower(LFrontPower*currentSpeed);
            RFront.setPower(RFrontPower*currentSpeed);
            LBack.setPower(LBackPower*currentSpeed);
            RBack.setPower(RBackPower*currentSpeed);
            
            //Intake Power Up
            if (gamepad2.a) {
                Intake.setPower(0.7);
            } else {
                Intake.setPower(0);
            }
            
            //Flywheel Power Up and Positioning
            if (gamepad2.right_trigger > 0.2) {
                FlyWheel.setVelocity(500);
            } else if (gamepad2.left_trigger > 0.2) {
                FlyWheel.setVelocity(1500);
            } else if (gamepad2.b) {
                FlyWheel.setVelocity(0);
            }
            
            //Ascension
            if (gamepad2.left_bumper) {
                AscensionLeft.setPower(-0.975);
                AscensionRight.setPower(1);
            } else if (gamepad2.right_bumper) {
                AscensionLeft.setPower(1);
                AscensionRight.setPower(-1);
            } else {
                AscensionLeft.setPower(0);
                AscensionRight.setPower(0);
            }
            
           /* int rotPos = (VsL.getCurrentPosition() + Math.abs(VsR.getCurrentPosition()))/2;
            if (gamepad2.dpad_up) {
                setRot(rotPos+rotSpeed);
            } else if (gamepad2.dpad_down) {
                setRot(rotPos-rotSpeed);
            } else {
                setRot(rotPos);
            }*/
    
   /* void setRot(int pos) {
        if (pos >= maxRot) {
            VsL.setTargetPosition(maxRot);
            VsR.setTargetPosition(maxRot);
            
            VsL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VsR.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            
            VsL.setVelocity(2000);
            VsR.setVelocity(2000);
        } else if (pos <= minRot) {
            VsL.setTargetPosition(minRot);
            VsR.setTargetPosition(minRot);
            
            VsL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VsR.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            
            VsL.setVelocity(2000);
            VsR.setVelocity(2000);
        } else {
            VsL.setTargetPosition(pos);
            VsR.setTargetPosition(pos);
            
            VsL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VsR.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
            
            VsL.setVelocity(2000);
            VsR.setVelocity(2000);
        }
    }*/
    
    
   /* void holdArmPosition(int VsLtargetPosition, int VsRtargetPosition, double VspeedL, double VspeedR) {
    VsL.setTargetPosition(VsLtargetPosition);
    VsR.setTargetPosition(VsRtargetPosition);
    VsL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    VsR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    VsL.setPower(VspeedL);
    VsR.setPower(VspeedR);
    
    }*/
        }
    }
}
