package frc.team1458.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.FX
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.util.maths.LookupTable2D

class Shooter(private val motor: FX,
              private val linearInterpolator: LookupTable2D,
              private val pidConstants: PIDConstants = PIDConstants.DISABLE) {
    init {
        motor.pidConstants = pidConstants
        motor.setVoltage(0.0)

        try {
            SmartDashboard.putNumber("shooterMotorPPR", motor.encoderPPR)
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }
    var error = 0.0

    var distanceSetpoint: Double = 0.0
        set(value) {
            field = value
            reconfigureSpeed()
        }

    private fun reconfigureSpeed() {
        // set pid based on this.distanceSetpoint
        val rpmSetpoint = linearInterpolator.interpolate(distanceSetpoint)

        motor.setVelocity(rpmSetpoint)
        error = motor.encoder.rate - rpmSetpoint

        SmartDashboard.putNumber("shooterMotorAngularVelocity", motor.encoder.rate)
        SmartDashboard.putNumber("shooterMotorError", error)
    }
}