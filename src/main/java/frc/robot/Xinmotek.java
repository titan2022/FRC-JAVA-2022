package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Xinmotek {
    private Joystick panel1, panel2;

    public class ButtonPad {
        public final JoystickButton topLeft, topRight, bottomLeft, bottomRight;

        ButtonPad(Joystick panel, int tl, int bl, int tr, int br) {
            topLeft = new JoystickButton(panel, tl);
            bottomLeft = new JoystickButton(panel, bl);
            topRight = new JoystickButton(panel, tr);
            bottomRight = new JoystickButton(panel, br);
        }
    }

    public final JoystickButton upButton, downButton;
    public final ButtonPad leftPad, middlePad, rightPad;

    public Xinmotek(int port1, int port2) {
        panel1 = new Joystick(port1);
        panel2 = new Joystick(port2);
        leftPad = new ButtonPad(panel1, 1, 2, 3, 4);
        middlePad = new ButtonPad(panel1, 5, 6, 7, 8);
        rightPad = new ButtonPad(panel2, 1, 2, 3, 4);
        upButton = new JoystickButton(panel2, 5);
        downButton = new JoystickButton(panel2, 6);
    }

    public double getLeftX() {
        return panel1.getRawAxis(0);
    }
    public double getLeftY() {
        return -panel1.getRawAxis(1);
    }
    public double getRightX() {
        return panel2.getRawAxis(0);
    }
    public double getRightY() {
        return -panel2.getRawAxis(1);
    }
}
