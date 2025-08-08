package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonomaObv extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Elevador elevador = new Elevador(hardwareMap);
        SistemasDoRobo sistemas = new SistemasDoRobo(hardwareMap, true, true);
        SampleMecanumDrive chassi = new SampleMecanumDrive(hardwareMap);

        Pose2d posicaoInicial = new Pose2d(-9.2,62, Math.toRadians(0));
        chassi.setPoseEstimate(posicaoInicial);
        waitForStart();
        Trajectory InitialScore = chassi.trajectoryBuilder(posicaoInicial, true)
                .lineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)))
                .build();

        Trajectory ColetaSamples = chassi.trajectoryBuilder(InitialScore.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 35, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-37, 35, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-37, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-45, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-45, 55, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-45, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-55, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-55, 55, Math.toRadians(-90)))
                .build();
                /*.lineToLinearHeading(new Pose2d(-62, 14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-62, 55, Math.toRadians(-90)))
                 */

        Trajectory Specimen1 = chassi.trajectoryBuilder(ColetaSamples.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-37, 61, Math.toRadians(90)))
                .build();

        Trajectory Specimen1T = chassi.trajectoryBuilder(Specimen1.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)))
                .build();

        Trajectory Specimen2 = chassi.trajectoryBuilder(Specimen1T.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 61, Math.toRadians(90)))
                .build();

        Trajectory Specimen2T = chassi.trajectoryBuilder(Specimen2.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)))
                .build();

        Trajectory Specimen3 = chassi.trajectoryBuilder(Specimen2T.end(), true)
                .lineToLinearHeading(new Pose2d(-37, 61, Math.toRadians(90)))
                .build();

        Trajectory Specimen3T = chassi.trajectoryBuilder(Specimen3.end(), true)
                .lineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)))
                .build();

        //elevador.DescerPraColetarEspecime();

        chassi.followTrajectoryAsync(InitialScore);
        chassi.followTrajectoryAsync(ColetaSamples);
        chassi.followTrajectoryAsync(Specimen1);
        chassi.followTrajectoryAsync(Specimen1T);
        chassi.followTrajectoryAsync(Specimen2);
        chassi.followTrajectoryAsync(Specimen2T);
        chassi.followTrajectoryAsync(Specimen3);
        chassi.followTrajectoryAsync(Specimen3T);

    }
}
