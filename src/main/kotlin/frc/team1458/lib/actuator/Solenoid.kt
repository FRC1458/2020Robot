package frc.team1458.lib.actuator

import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.team1458.lib.input.interfaces.Switch

interface Solenoid {
    val position : Position

    fun extend()
    fun retract()

    operator fun plus(other: Solenoid): Solenoid {
        val thisThing = this

        return object : Solenoid {

            override val position: Position
                get() = thisThing.position

            override fun extend() {
                thisThing.extend()
                other.extend()
            }

            override fun retract() {
                thisThing.retract()
                other.retract()
            }

        }
    }

    companion object {

        fun doubleSolenoid(PCMcanID: Int = 0, extendChannel: Int, retractChannel: Int) : Solenoid {
            val solenoid : DoubleSolenoid = DoubleSolenoid(PCMcanID, extendChannel, retractChannel)

            val sol = object : Solenoid {
                var _pos: Position = Position.UNKNOWN

                override val position: Position
                    get() = _pos

                override fun extend() {
                    solenoid.set(DoubleSolenoid.Value.kForward)
                    _pos = Position.EXTENDED
                }

                override fun retract() {
                    solenoid.set(DoubleSolenoid.Value.kReverse)
                    _pos = Position.RETRACTED
                }

            }

            return sol
        }

    }

    enum class Position {
        EXTENDED, RETRACTED, UNKNOWN
    }
}