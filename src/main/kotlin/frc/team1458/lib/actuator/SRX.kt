package frc.team1458.lib.actuator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import frc.team1458.lib.pid.PID
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor

class SRX(val canID: Int,
          val master: SRX? = null,
          val encoderPPR: Double = 4096.0,
          val invert: Boolean = false,
          val invertEncoder: Boolean = false,
          pidConstants: PIDConstants = PIDConstants.DISABLE,
          val voltageCompensation: Boolean = true,
          val voltageCompensationMax: Double = 12.0) {

    val inst = WPI_TalonSRX(canID)
    val isMaster = (master == null);

    var pidConstants: PIDConstants = pidConstants
        public get
        public set(value) {
            field = value;
            refreshPID()
        }

    val encoder: AngleSensor = if(master == null) {
        object : AngleSensor {
            override val angle: Double
                get() {
                    return SmartMotor.___talonRotationsToDegrees(inst.getSelectedSensorPosition(0).toDouble(), encoderPPR)
                }

            override val rate: Double
                get() = SmartMotor.___talonVelocityToDegreesPerSecond(inst.getSelectedSensorVelocity(0).toDouble(), encoderPPR)

            override fun zero() {
                inst.setSelectedSensorPosition(0);
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
        inst.setInverted(invert);
        inst.setNeutralMode(NeutralMode.Brake);

        if(master == null) {
            inst.setSensorPhase(invertEncoder);
            inst.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
            inst.setSelectedSensorPosition(0);

            refreshPID()
        } else {
            inst.follow(master.inst);
        }

        if(voltageCompensation) {
            inst.configVoltageCompSaturation(voltageCompensationMax)
            inst.enableVoltageCompensation(true)
        }
    }

    fun refreshPID() {
        if(isMaster) {
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
        //no anish println("yeeting talon ${SmartMotor.___degreesPerSecondToTalonVelocity(vel, encoderPPR)}, ${pidConstants.kS}, ${Math.copySign(vel, pidConstants.kS)}")
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