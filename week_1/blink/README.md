# Blink

Software Engineering workshop week 1, part 1. The tutorial is based on https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink

## Introduction

This example shows the simplest thing you can do with an Arduino to see physical output: it blinks an LED on the breadboard.


## Code

After you build the circuit plug your Arduino board into your computer, start the Arduino Software (IDE) and enter the code below. You may also load it from the menu File/Examples/01.Basics/Blink . The first thing you do is to initialize `LED_BUILTIN` pin as an output pin with the line

    pinMode(LED_BUILTIN, OUTPUT);

In the main loop, you turn the LED on with the line:

    digitalWrite(LED_BUILTIN, HIGH);

This supplies 5 volts to the LED anode. That creates a voltage difference across the pins of the LED, and lights it up. Then you turn it off with the line:

    digitalWrite(LED_BUILTIN, LOW);

That takes the `LED_BUILTIN` pin back to 0 volts, and turns the LED off. In between the on and the off, you want enough time for a person to see the change, so the delay() commands tell the board to do nothing for 1000 milliseconds, or one second. When you use the delay() command, nothing else happens for that amount of time. Once you've understood the basic examples, check out the BlinkWithoutDelay example to learn how to create a delay while doing other things.

Once you've understood this example, check out the DigitalReadSerial example to learn how read a switch connected to the board.


## Explanation

Every piece of Arduino software we write will have two parts, `setup()` and `loop()`. In the `setup()` method, which will only be executed once after the Arduino powers on, you will initialize all of the input and output pins, and any other variables you might need for your program. In the `loop()` method, you should specify the behavior of your program. This `loop()` will run forever until you reset the Arduino or powers it off. Remember that the loop runs at a very high frequency.   
