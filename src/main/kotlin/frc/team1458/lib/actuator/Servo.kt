package frc.team1458.lib.actuator

import frc.team1458.lib.util.maths.TurtleMaths

class Servo(port: Int, val minAngle: Double = 0.0, val maxAngle: Double = 180.0) {
    val servo = edu.wpi.first.wpilibj.Servo(port)

    fun setRaw(position: Double) {
        servo.set(TurtleMaths.constrain(position, 0.0, 1.0))
    }

    var angle : Double = minAngle
        set(angle) {
            field = angle
            setRaw(TurtleMaths.shift(angle, minAngle, maxAngle, 0.0, 1.0))
        }
        get
}