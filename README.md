<p>
<img src = "https://github.com/jcturing/RoboTrace/blob/master/Multimedia/WhatsApp%20Image%202019-04-16%20at%2021.04.07.jpeg" align = "right" width = "100"/>
</p>

# Micromouse
Everything about my RoboTrace Robot.

## Concept
This RoboTrace robot was a project for a competition ([Cybertech](https://www.reset.etsii.upm.es/cybertech/cybertech2019/)) in the UPM in Madrid.
It basically has to run the circuit made by a black line as fast as possible.

## Description
The robot has a light reflectance array sensor to detect the black line and encoders to know if we make a turn or we go straight.

## Prerequisites
### Hardware
Here I list all the component that the robot contains:
* Arduino Uno
* 30:1 Pololu Motors
* Battery
* Pololu Magnetic Encoders
* Pololu Motor Driver DRV8835 
* QTR-8A Light Sensors
* Buzzer, Led, Buttons, Ball and others...

### Software
The code provided in this repository uses the **Circuit** library. You can download it [here](https://github.com/jcturing/Arduino-Libraries).
This library also depends on others so you may have to download the rest of them.

In the Circuit library, all the segments of the circuit are stored and depending on their right and left lenght, it guessses if wwe are in a straight line or a turn
so it increments or decrement speed respectively.

## Contributing
I am aware that I can improve this robot a lot. My will is to create one like the profesional ones by myself.
This is my first try in RoboTrace and more advanced version might come later on.
However, I tried to do my best with what I had. If you want to contribute with this project any help is welcome!

## Acknowledgments
I want to thank to all the people that shares their knowledge about RoboTrace online.
