package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DIASDELUTA extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robo = new SampleMecanumDrive(hardwareMap);
        Pose2d posicaoinicial = new Pose2d(new Vector2d(0,0), Math.toRadians(0));
        robo.setPoseEstimate(posicaoinicial);

        Trajectory Score = robo.trajectoryBuilder(posicaoinicial)
                .build();
        waitForStart();

        robo.followTrajectory(Score);
    }
}
