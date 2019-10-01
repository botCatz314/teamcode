package org.firstinspires.ftc.teamcode.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
@Disabled
@Autonomous (name = "ezraTeleopSaftyBot")
public class ezraTeleopSaftyBot extends LinearOpMode {
    private DcMotor left, right, launch; //declares drive motors
    @Override
    public void runOpMode() {
        //motors

        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        launch = hardwareMap.dcMotor.get("launch");
        waitForStart();

        right.setPower(gamepad1.left_stick_x);
        left.setPower(gamepad1.right_stick_x);

        launch.setPower(gamepad1.left_trigger);
        launch.setPower(-gamepad1.right_trigger);



        idle();
    }
}