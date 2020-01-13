package frc.team1458.lib.sensor.interfaces

import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.interfaces.Gyro
import frc.team1458.lib.util.flow.go
import frc.team1458.lib.util.flow.periodic
import frc.team1458.lib.util.flow.systemTimeMillis
import frc.team1458.lib.util.maths.MovingAverage
import frc.team1458.lib.util.maths.TurtleMaths

interface AngleSensor : Zeroable {
    val inverted : AngleSensor
        get() = create({ -1 * angle }, { -1 * rate })

    /**
     * Angle in degrees
     */
    val angle : Double

    // Superior unit
    val radians : Double
        get() = TurtleMaths.constrainAngle(angle * 0.0174533)

    /**
     * Angular velocity in degrees per second
     */
    val rate : Double

    /**
     * Heading in degrees in the interval [0, 360)
     */
    val heading : Double
        get() = angle % 360.0

    override fun zero()

    companion object {
        fun create(angle: () -> Double, rate: () -> Double): AngleSensor {
            return object : AngleSensor {
                var _zero = 0.0

                override val angle: Double
                    get() {
                        return angle() - _zero
                    }

                override val rate: Double
                    get() = rate()

                override fun zero() {
                    _zero = angle()
                }
            }
        }

        fun create(samplesToAverage: Int = 10, angle: () -> Double): AngleSensor {
            var sensor = object : AngleSensor {
                var _zero = 0.0
                @Volatile var _rate = 0.0

                override val angle: Double
                    get() = angle() - _zero

                override val rate: Double
                    get() = _rate

                override fun zero() {
                    _zero = angle()
                }
            }

            var lastVal = 0.0
            var lastTime = systemTimeMillis - 1
            var movingAverage = MovingAverage(samplesToAverage)
            go { periodic (hz = 50) {
                movingAverage.addNumber((sensor.angle - lastVal) / ((systemTimeMillis - lastTime) / 1000))
                sensor._rate = movingAverage.average
                lastVal = sensor.angle
                lastTime = systemTimeMillis
            } }

            return sensor
        }

        fun encoder(portA: Int, portB: Int = portA + 1, ratio: Double = 1.0, reverse: Boolean = false): AngleSensor {
            var enc = Encoder(portA, portB, reverse)
            enc.distancePerPulse = ratio
            return create(enc::getDistance, enc::getRate)
        }

        fun hallSensor(port: Int, countHighs: Boolean = false, scale: Double = 8.2125): AngleSensor {
            var c = Counter()
            c.setUpSource(port)
            c.setUpDownCounterMode()
            c.setSemiPeriodMode(countHighs)
            c.setMaxPeriod(1.0)
            c.setDistancePerPulse(1.0)
            c.samplesToAverage = 6

            return create({ c.distance * scale }, { c.rate * scale })
        }

        fun fromGyroscope(gyro: Gyro) : AngleSensor {
            return create(gyro::getAngle, gyro::getAngle)
        }
    }
}
