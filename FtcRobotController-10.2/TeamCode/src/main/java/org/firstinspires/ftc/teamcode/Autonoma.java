package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Autonoma extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Vector2d sigma = new Vector2d();
        Pose2d posicaoInicial = new Pose2d();

        chassi.setPoseEstimate(posicaoInicial);

        Trajectory traj1 = chassi.trajectoryBuilder(posicaoInicial)
                .splineTo(new Vector2d(), Math.toRadians(90))
                .build();

        Trajectory traj2 = chassi.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(), Math.toRadians(90))
                .build();

        waitForStart();

        chassi.followTrajectory(traj1);
        chassi.followTrajectory(traj2);
    }
}
