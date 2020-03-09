package frc.team1458.robot

import edu.wpi.first.wpilibj.XboxController
import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.POV
import frc.team1458.lib.input.interfaces.Switch

class OI {
    val xboxController: Gamepad = Gamepad.xboxController(0)

    var throttle = xboxController.leftY.scale(1.0).inverted
    var steer = xboxController.rightX.scale(0.5)

}
