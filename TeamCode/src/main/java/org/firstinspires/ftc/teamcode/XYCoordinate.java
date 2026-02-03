// package org.firstinspires.ftc.teamcode.onbotjava;
// 
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.robot.Robot;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.AnalogSensor;
// import com.qualcomm.robotcore.hardware.AnalogInput;
// import com.qualcomm.robotcore.hardware.DigitalChannel;
// import java.io.StringWriter;
// import com.qualcomm.robotcore.hardware.SwitchableLight;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.Gamepad;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// 
// @Autonomous
// 
// public class XYCoordinate {
// 
//     private Robot robot = new Robot(this);
//     
//     public void runOpMode()
//     {
//         robot.initialize(true);
//         
//         telemetry.addData(">", "Touch Play to run Auto");
//         telemetry.update();
//         
//         waitForStart();
//         robot.resetHeading();
//         
//         if (opModeIsActive())
//         {
//             //Drive Rectangle
//             robot.drive(84, 0.60, 0.25);
//             robot.turnTo(90, 0.45, 0.5);
//             robot.drive(72, 0.60, 0.25);
//             robot.turnTo(180, 0.45, 0.5);
//             robot.drive(84, 0.60, 0.25);
//             robot.turnTo(270, 0.45, 0.5);
//             robot.drive(72, 0.60, 0.25);
//             robot.turnTo(0, 0.45, 0.5);
//             
//             sleep(500);
//             
//             //Drive with Strafe
//             robot.drive(84, 0.60, 0.15);
//             robot.strafe(72, 0.60, 0.15);
//             robot.drive(-84, 0.60, 0.15);
//             robot.strafe(-72, 0.60, 0.15);
//             
//         }
//     }
// }