/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="GoldMiniBot", group="DogeCV")

public class GoldMiniBot extends LinearOpMode
{
    private GoldAlignDetector detector;
    private DcMotor left, right;
    private Servo phoneServo;
    private String position = null;
    boolean control = false;
    @Override
    public void runOpMode(){

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 10; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 5000; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 0.8;

        detector.setAlignSettings(0,1000); //changed from 400...........
        detector.enable();

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        phoneServo = hardwareMap.servo.get("phoneServo");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        phoneServo.setPosition(1);
        sleep(3000);

        waitForStart();
        if(detector.getAligned()){
            telemetry.addData("aligned: ", true);
            telemetry.update();
            sleep(1000);
            phoneServo.setPosition(phoneServo.getPosition() -0.01);
            sleep(3000);
            if(detector.getAligned()){
                telemetry.addData("still aligned: ", true);
                telemetry.update();
                position = "right";
            }
        }
        sleep(1000);
        phoneServo.setPosition(0.5);
        sleep(3000);

        if(detector.getAligned() && position == null){
            telemetry.addData("aligned: ", true);
            telemetry.update();
            phoneServo.setPosition(phoneServo.getPosition() - 0.01);
            sleep(1000);
            if(detector.getAligned()) {
                position = "left";
            }
        }
        else if (position == null){
            position = "center";
        }
        sleep(1000);

        telemetry.addData("the position is: ", position);
        telemetry.update();
        phoneServo.setPosition(0.92);
        sleep(100);

        detector.disable();

        if (position == "left"){
            left.setPower(-0.2);
            right.setPower(0.2);
            sleep(1200);
            left.setPower(-1);
            right.setPower(-1);
            sleep(2000); //negative is forwards


        }
        if(position == "right"){
            left.setPower(0.2);
            right.setPower(-0.2);
            sleep(375);
            left.setPower(-1);
            right.setPower(-1);
            sleep(1200);
        }
        if(position == "center"){
            left.setPower(-1);
            right.setPower(-1);
            sleep(750);
            left.setPower(0);
            right.setPower(0);
        }

    }

}
