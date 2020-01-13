package frc.team1458.robot

import edu.wpi.first.wpilibj.XboxController
import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.POV
import frc.team1458.lib.input.interfaces.Switch

class OI {
    private val xboxController: Gamepad = Gamepad.xboxController(0)

    var throttle = xboxController.rightX.scale(0.7)
    var steer = xboxController.leftY.scale(0.5) // TODO Check invert

}
