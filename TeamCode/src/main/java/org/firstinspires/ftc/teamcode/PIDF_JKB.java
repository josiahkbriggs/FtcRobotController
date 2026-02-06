package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class PIDF_JKB extends OpMode {
    
        public DcMotorEx FlyWheel;

        public double highVelocity = 1650;
        public double lowVelocity = 700;
        
        double curTargetVelocity = highVelocity;
        
        double F = 18.6;
        double P = 30.2;
        
        double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
        
        int stepIndex = 1;
        

        @Override
        public void init() {
            FlyWheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
            FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            FlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            telemetry.addLine("Init Complete");
            
        }
        
        @Override
        public void loop() {
            //get gamepad commands
            //set target velocity
            //update telemetry
            
            if (gamepad2.xWasPressed()) {
                if(curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else { curTargetVelocity = highVelocity; }
            }
            
            if (gamepad2.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            
            if (gamepad2.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }
            if (gamepad2.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }
            
            if (gamepad2.dpadDownWasPressed()) {
                P += stepSizes[stepIndex];
            }
            if (gamepad2.dpadUpWasPressed()){
                P -= stepSizes[stepIndex];
            }
            
            //set new PIDF  coefficients
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            FlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            
            //set velocity
            FlyWheel.setVelocity(curTargetVelocity);

            double curVelocity = FlyWheel.getVelocity();
            double error = curTargetVelocity - curVelocity;
            
            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R", F);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            
        }
}