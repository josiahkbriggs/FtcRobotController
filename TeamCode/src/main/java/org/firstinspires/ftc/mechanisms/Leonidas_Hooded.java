// package org.firstinspires.ftc.teamcode.onbotjava;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.VoltageSensor;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
// import com.qualcomm.hardware.dfrobot.HuskyLens;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import org.firstinspires.ftc.robotcore.internal.system.Deadline;
// import java.util.concurrent.TimeUnit;
// 
// 
// @TeleOp
// public class Leonidas_Hooded extends LinearOpMode {
//     
//     double Speed = 1;
//     int top = 2;
//     int Intaker = 2;
//     int ballTop = 2;
//     int ballBottom = 2;
//     int ballIntake = 2;
//     int lifter =2;
//     //int nuclearLaunch = 0;
// 
//     private DistanceSensor DST;
//     private DistanceSensor DSB;
//     private DistanceSensor DSI;
//     private HuskyLens huskyLens;
//     
//     @Override
//     public void runOpMode() throws InterruptedException {
//         
//         //say motors exist
//         DcMotor LFront = hardwareMap.dcMotor.get("LFront");
//         DcMotor LBack = hardwareMap.dcMotor.get("LBack");
//         DcMotor RFront = hardwareMap.dcMotor.get("RFront");
//         DcMotor RBack = hardwareMap.dcMotor.get("RBack");
//         DcMotor Intake = hardwareMap.dcMotor.get("Intake");
//         DcMotor FlyWheel = hardwareMap.dcMotor.get("FlyWheel");
//         DcMotor AscensionLeft = hardwareMap.dcMotor.get("AscensionLeft");
//         DcMotor AscensionRight = hardwareMap.dcMotor.get("AscensionRight");
// 
//         huskyLens = hardwareMap.get(HuskyLens.class, "HL");
//         VoltageSensor controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//         
//         LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         LBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         RBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 
//             babyGronkLeft.setDirection(DcMotor.Direction.REVERSE);
//             freakLeft.setDirection(DcMotor.Direction.REVERSE);
//             
//             //huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//             
//         waitForStart();
//         
//         while (opModeIsActive()) {
//         
//             double y = gamepad1.left_stick_y;
//             double x = gamepad1.left_stick_x;
//             double rx = gamepad1.right_stick_x;
//             
//             double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//             
//             if (gamepad1.left_bumper) {
//             Speed = 1;
//             } else if (gamepad1.right_bumper) {
//             Speed = 0.3;
//             } else {
//             Speed = 0.7;
//             }
//             
//             double freakLeftPower = (y - x - rx) / denominator * Speed;
//             double babyGronkLeftPower = (y + x - rx) / denominator * Speed;
//             double freakRightPower = (y + x + rx) / denominator * Speed;
//             double babyGronkRightPower = (y - x + rx) / denominator * Speed;
//             
//             freakLeft.setPower(freakLeftPower);
//             babyGronkLeft.setPower(babyGronkLeftPower);
//             freakRight.setPower(freakRightPower);
//             babyGronkRight.setPower(babyGronkRightPower);
//             
//             if (gamepad2.b && gamepad2.x) {
//             Intaker = 1;
//             } else if (gamepad2.b) {
//             Intaker = 0;
//             } else if (gamepad2.x) {
//             Intaker = 2;
//             }
//             
//             if (gamepad1.a) {
//             liftUp.setPower(-1);
//             } else if (gamepad1.b) {
//             liftUp.setPower(1);
//             } else {
//             liftUp.setPower(0);
//             }
//             
//             if (Intaker == 0 && lifter != 0) {
//             Intake.setPower(0.6);
//             } else if (Intaker == 1) {
//             Intake.setPower(-0.5);
//             } else if (lifter == 0) {
//             Intake.setPower(0);
//             }
//             
//             if (gamepad2.y) {
//             PL.setPosition(0.23);
//             PR.setPosition(0.47);
//             lifter = 0;
//             } else {
//             PL.setPosition(0.94);
//             PR.setPosition(0.84);
//             lifter = 1;
//             }
//             
//             if (gamepad2.a) {
//             BP.setPosition(0.9);
//             } else {
//             BP.setPosition(0.35);
//             }
//             
//             if (gamepad2.dpad_up) {
//             launchRight.setPower(0.67);
//             launchLeft.setPower(0.67);
//             } else if (gamepad2.dpad_left) {
//             launchRight.setPower(0.69);
//             launchLeft.setPower(0.69);
//             } else if (gamepad2.dpad_down) {
//             launchRight.setPower(0.84);
//             launchLeft.setPower(0.84);
//             } else if (gamepad2.dpad_right) {
//             launchRight.setPower(1);
//             launchLeft.setPower(1);
//             }
//             
// 
//             //double currentVoltage = controlHubVoltageSensor.getVoltage();
//             
//             //Distance sensors
//             /*if (DST.getDistance(DistanceUnit.CM) < 10) {
//             ballTop = 0;
//             } else {
//             ballTop = 1;
//             }
//             
//             if (DSB.getDistance(DistanceUnit.CM) < 10) {
//             ballBottom = 0;
//             } else {
//             ballBottom = 1;
//             }
//             
//             if (DSI.getDistance(DistanceUnit.CM) < 10) {
//             ballIntake = 0;
//             } else {
//             ballIntake = 1;
//             }*/
//             
//             /*if (gamepad2.dpad_up && controlHubVoltageSensor.getVoltage() > 12.7) {
//                 rizzRight.setPosition(0.10);
//                 rizzLeft.setPosition(0.90);
//                 ligmaRight.setPower(0.42);
//                 ligmaLeft.setPower(-0.42);
//             } else if (gamepad2.dpad_up && controlHubVoltageSensor.getVoltage() <= 12.7 && controlHubVoltageSensor.getVoltage() > 12.45) {
//                 rizzRight.setPosition(0.10);
//                 rizzLeft.setPosition(0.90);
//                 ligmaRight.setPower(0.44);
//                 ligmaLeft.setPower(-0.44);
//             } else if (gamepad2.dpad_up && controlHubVoltageSensor.getVoltage() <= 12.45) {
//                 rizzRight.setPosition(0.10);
//                 rizzLeft.setPosition(0.90);
//                 ligmaRight.setPower(0.46);
//                 ligmaLeft.setPower(-0.46);
//             } else if (gamepad2.dpad_left && controlHubVoltageSensor.getVoltage() > 12.7) {
//                 rizzRight.setPosition(0.15);
//                 rizzLeft.setPosition(0.85);
//                 ligmaRight.setPower(0.46);
//                 ligmaLeft.setPower(-0.46);
//             } else if (gamepad2.dpad_left && controlHubVoltageSensor.getVoltage() <= 12.7 && controlHubVoltageSensor.getVoltage() > 12.45) {
//                 rizzRight.setPosition(0.15);
//                 rizzLeft.setPosition(0.85);
//                 ligmaRight.setPower(0.48);
//                 ligmaLeft.setPower(-0.48);
//             } else if (gamepad2.dpad_left && controlHubVoltageSensor.getVoltage() <= 12.45) {
//                 rizzRight.setPosition(0.15);
//                 rizzLeft.setPosition(0.85);
//                 ligmaRight.setPower(0.5);
//                 ligmaLeft.setPower(-0.5);
//             } if (gamepad2.dpad_down && controlHubVoltageSensor.getVoltage() > 12.7) {
//                 rizzRight.setPosition(0.21);
//                 rizzLeft.setPosition(0.79);
//                 ligmaRight.setPower(0.61);
//                 ligmaLeft.setPower(-0.61);
//             } else if (gamepad2.dpad_down && controlHubVoltageSensor.getVoltage() <= 12.7 && controlHubVoltageSensor.getVoltage() > 12.45) {
//                 rizzRight.setPosition(0.21);
//                 rizzLeft.setPosition(0.79);
//                 ligmaRight.setPower(0.64);
//                 ligmaLeft.setPower(-0.64);
//             } else if (gamepad2.dpad_down && controlHubVoltageSensor.getVoltage() <= 12.45) {
//                 rizzRight.setPosition(0.21);
//                 rizzLeft.setPosition(0.79);
//                 ligmaRight.setPower(0.66);
//                 ligmaLeft.setPower(-0.66);
//             }*/
//             
//             //telemetry.addData("Voltage", controlHubVoltageSensor.getVoltage());
//             telemetry.addData("ballTop", ballTop);
//             telemetry.addData("ballBottom", ballBottom);
//             telemetry.addData("ballIntake", ballIntake);
//             telemetry.update();
//         }
//     }
// }
// 
