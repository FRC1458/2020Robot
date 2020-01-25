package frc.team1458.robot

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.team1458.lib.util.flow.systemTimeSeconds

class RamseteFollower(
        val path: Trajectory,
        val odom: () -> Pose2d,
        val b: Double,
        val zeta: Double,
        val goalToleranceFeet: Double = 1.0,
        val goalToleranceDegrees: Double = 10.0) {

    val controller = RamseteController(b, zeta)

    val isFinished: Boolean
        get() {
            val poseError = path.sample(path.totalTimeSeconds).poseMeters.relativeTo(odom());
            val linearError = poseError.getTranslation().norm.toDouble()
            val angularError = poseError.getRotation().degrees.toDouble()
            return (Math.abs(linearError) < goalToleranceFeet) && (Math.abs(angularError) < goalToleranceDegrees)
        }


    var startTime = -100.0

    // Calculate linvel, angvel
    fun calculate(): Pair<Double, Double> {
        if (startTime < 0) {
            startTime = systemTimeSeconds
        }

        val referencePoint = path.sample(systemTimeSeconds - startTime)
        val referencePose = referencePoint.poseMeters
        val poseError = referencePose.relativeTo(odom())

        val errorX: Double = poseError.getTranslation().x
        val errorY: Double = poseError.getTranslation().y
        val errorTheta: Double = poseError.getRotation().radians

        val velocityTarget = referencePoint.velocityMetersPerSecond
        val angularVelocityTarget = referencePoint.velocityMetersPerSecond * referencePoint.curvatureRadPerMeter

        val k: Double = 2.0 * zeta * Math.sqrt(Math.pow(angularVelocityTarget, 2.0) + b * Math.pow(velocityTarget, 2.0))

        val output = Pair<Double, Double>(
                velocityTarget * poseError.getRotation().getCos() + k * errorX,
                angularVelocityTarget + k * errorTheta + b * velocityTarget * safeSinC(errorTheta) * errorY
        )

        //println("RAMSYEETAR,${referencePoint.poseMeters.translation.x}")

        return output
    }

    private fun safeSinC(x: Double): Double {
        return if (Math.abs(x) < 1e-9) {
            1.0 - 1.0 / 6.0 * x * x
        } else {
            Math.sin(x) / x
        }
    }
}