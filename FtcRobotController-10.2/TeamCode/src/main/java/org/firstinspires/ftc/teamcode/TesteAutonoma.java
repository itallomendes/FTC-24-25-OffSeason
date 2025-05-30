package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TesteAutonoma extends LinearOpMode {

    SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

    Pose2d posicaoInicial = new Pose2d(new Vector2d(30, 30), Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
