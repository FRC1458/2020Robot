package frc.team1458.lib.actuator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import frc.team1458.lib.pid.PID
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor

/* TODO Implement this new 2020 current limiting for FX and SRX (output and input limits):
 * Talon FX supports both stator(output) current limiting and supply(input) current limiting.
 *
 * Supply limiting (supported by Talon SRX and FX) is useful for preventing breakers from tripping in the PDP.
 *
 * Stator limiting (supported by Talon FX) is useful for limiting acceleration/heat.
 *
 * The new API leverages the configSupplyCurrentLimit and configStatorCurrentLimit routines.The configs are similar to
 * the existing legacy API, but the configs have been renamed to better communicate the design intent. For example,
 * instead of configPeakCurrentLimit, the setting is referred to as triggerThresholdCurrent.
 *
 * https://gist.github.com/jcorcoran/e56d985fd1ee290b075d13c9f9aa3595
 *
 */


class FX(val canID: Int,
         val master: SRX? = null,
         val encoderPPR: Double = 2048.0,
         val invert: Boolean = false,
         val invertEncoder: Boolean = false,
         pidConstants: PIDConstants = PIDConstants.DISABLE,
         val voltageCompensation: Boolean = true,
         val voltageCompensationMax: Double = 12.0) {

    val inst = WPI_TalonFX(canID)
    val isMaster = (master == null);

    var pidConstants: PIDConstants = pidConstants
        public get
        public set(value) {
            field = value;
            refreshPID()
        }

    val encoder: AngleSensor = if (master == null) {
        object : AngleSensor {
            override val angle: Double
                get() {
                    return SmartMotor.___talonRotationsToDegrees(inst.getSelectedSensorPosition(0).toDouble(), encoderPPR)
                }

            override val rate: Double
                get() = SmartMotor.___talonVelocityToDegreesPerSecond(inst.getSelectedSensorVelocity(0).toDouble(), encoderPPR)

            override fun zero() {
                inst.selectedSensorPosition = 0;
            }
        }
    } else {
        master.encoder
    }

    val closedLoopError: Double
        get() {
            if (master == null) {
                return SmartMotor.___talonVelocityToDegreesPerSecond(inst.closedLoopError.toDouble(), encoderPPR)
            } else {
                return master.closedLoopError
            }
        }


    init {
        inst.inverted = invert
        inst.setNeutralMode(NeutralMode.Brake)

        if (master == null) {
            inst.setSensorPhase(invertEncoder)
            inst.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10)
            inst.selectedSensorPosition = 0

            refreshPID()
        } else {
            inst.follow(master.inst);
        }

        if (voltageCompensation) {
            inst.configVoltageCompSaturation(voltageCompensationMax)
            inst.enableVoltageCompensation(true)
        }
    }

    fun refreshPID() {
        if (isMaster) {
            inst.config_kP(0, pidConstants.kP)
            inst.config_kI(0, pidConstants.kI)
            inst.config_kD(0, pidConstants.kD)
            inst.config_kF(0, pidConstants.kF)
        }
    }

    fun setRaw(power: Double) {
        inst.set(power)
    }

    fun setVoltage(voltage: Double) {
        inst.setVoltage(voltage)
    }

    // Takes degrees/sec
    fun setVelocity(vel: Double) {
        println("yeeting talon ${SmartMotor.___degreesPerSecondToTalonVelocity(vel, encoderPPR)}, ${pidConstants.kS}, ${Math.copySign(vel, pidConstants.kS)}")
        inst.set(ControlMode.Velocity, SmartMotor.___degreesPerSecondToTalonVelocity(vel, encoderPPR), DemandType.ArbitraryFeedForward, Math.copySign(pidConstants.kS, vel))
    }

    companion object {
        fun _talonRotationsToDegrees(talonUnits: Double, ppr: Double): Double {
            val revolutions = talonUnits / (ppr)
            val degrees = revolutions * 360.0
            return degrees
        }

        fun _degreesToTalonRotations(degrees: Double, ppr: Double): Double {
            val revolutions = degrees / 360.0
            val talonUnits = revolutions * (ppr)
            return talonUnits
        }

        fun _talonVelocityToDegreesPerSecond(talonUnitsPerDecisecond: Double, ppr: Double): Double {
            val degreesPerDecisecond = _talonRotationsToDegrees(talonUnitsPerDecisecond, ppr)
            val degreesPerSecond = degreesPerDecisecond * 10.0

            return degreesPerSecond
        }

        fun _degreesPerSecondToTalonVelocity(degreesPerSecond: Double, ppr: Double): Double {
            val degreesPerDecisecond = degreesPerSecond / 10.0
            val talonUnitsPerDecisecond = _degreesToTalonRotations(degreesPerDecisecond, ppr)
            return talonUnitsPerDecisecond
        }
    }
}