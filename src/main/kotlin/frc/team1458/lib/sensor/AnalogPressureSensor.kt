package frc.team1458.lib.sensor

import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController

/**
 * Analog pressure sensor for pneumatics such as https://www.andymark.com/product-p/am-3219.htm
 */
class AnalogPressureSensor(val channel: Int) {
    val analogSensor = AnalogInput(channel)

    /**
     * Pressure, in PSI
     */
    val pressure : Double
        get() = 250 * (analogSensor.voltage / RobotController.getVoltage5V()) - 25

    val pressureBar : Double
        get() = 0.0689476 * pressure
}