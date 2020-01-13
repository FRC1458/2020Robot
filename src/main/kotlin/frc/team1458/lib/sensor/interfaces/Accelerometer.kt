package frc.team1458.lib.sensor.interfaces

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.interfaces.Accelerometer


interface Accelerometer {
    /**
     * Acceleration on x-axis in m/s^2
     */
    val x : Double

    /**
     * Acceleration on y-axis in m/s^2
     */
    val y : Double

    /**
     * Acceleration on z-axis in m/s^2
     */
    val z : Double

    companion object {
        fun create(xSource: () -> Double, ySource: () -> Double, zSource: () -> Double) : frc.team1458.lib.sensor.interfaces.Accelerometer {
            return object : frc.team1458.lib.sensor.interfaces.Accelerometer {
                override val x: Double
                    get() = xSource()
                override val y: Double
                    get() = ySource()
                override val z: Double
                    get() = zSource()
            }
        }

        fun ADXL345(port: I2C.Port, range: Accelerometer.Range = Accelerometer.Range.k8G) : frc.team1458.lib.sensor.interfaces.Accelerometer {
            var accelerometer = ADXL345_I2C(port, range)
            return create(accelerometer::getX, accelerometer::getY, accelerometer::getZ)
        }

        fun ADXL345(port: SPI.Port, range: Accelerometer.Range = Accelerometer.Range.k8G) : frc.team1458.lib.sensor.interfaces.Accelerometer {
            var accelerometer = ADXL345_SPI(port, range)
            return create(accelerometer::getX, accelerometer::getY, accelerometer::getZ)
        }

        fun builtIn() :frc.team1458.lib.sensor.interfaces.Accelerometer {
            var accelerometer = BuiltInAccelerometer()
            return create(accelerometer::getX, accelerometer::getY, accelerometer::getZ)
        }
    }
}