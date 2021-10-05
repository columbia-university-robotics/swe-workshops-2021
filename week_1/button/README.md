
# Buttons

This tutorial is based on this: https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button

## About the Circuit

Connect three wires to the board. The first two, red and black, connect to the two long vertical rows on the side of the breadboard to provide access to the 5 volt supply and ground. The third wire goes from digital pin 2 to one leg of the pushbutton. That same leg of the button connects through a pull-down resistor (here 10K ohm) to ground. The other leg of the button connects to the 5 volt supply.

Please note, when the pushbutton is open (unpressed) there is no connection between the two legs of the pushbutton, so the pin (connected to the Arduino) is connected to ground (through the pull-down resistor). Therefore, we read a LOW through `digitalRead()`. When the button is closed (pressed), it makes a connection between its two legs, connecting the pin to 5 volts, so that we read a HIGH through `digitalRead()`.

You can also wire this circuit the opposite way, with a pullup resistor keeping the input HIGH, and going LOW when the button is pressed. If so, the behavior of the sketch will be reversed, with the LED normally on and turning off when you press the button.

If you disconnect the digital I/O pin from everything, the LED may blink erratically. This is because the input is "floating" - that is, it will randomly return either HIGH or LOW. That's why you need a pull-up or pull-down resistor in the circuit.

## Comments

Recall that for each pin on the Arduino, you can either read the value of that pin `digitalRead()`, or write to that pin `digitalWrite()`. In the previous tutorial, we have demonrated `digitalWrite()`. The goal of this tutorial is to demonstrate the `digitalRead()` feature of the Arduino. Reading the value of a digital pin will be crucial in building most Arduino projects. 
