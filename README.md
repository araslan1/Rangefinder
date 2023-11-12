# Rangefinder

# Inspiration

Rangefinder served as my final project for an embedded systems class at USC under Professor Alan Webber. 

# Overview: 

The goal of this project was to use a range sensor to accurately measure how far an object is at the press of a button. I manually wired and hooked up the components onto a blackboard and connected them to an Arduino. I programmed the microcontroller with C to set up and manipulate the components to do as follows: 

The "Acquire" button is used to initiate a range measurement. Each time it is pressed the ultrasonic range sensor takes a new range measurement and the results are displayed on the LCD. The limits of the range sensor can detect object’s distances up to 400 centimeters away. 


Besides displaying the range on the LCD, the distance is also shown on a round dial by using a servo motor to rotate an indicator to point to the range. The servo is controlled by Timer/Counter2 since this is the only timer with an output signal that is not blocked by the LCD panel

The project also incorporates a multicolor (red, green, blue) LED that is used to show the comparison between the most recent local range measurement and the local range threshold setting done with the rotary encoder. If the range measured is greater than or equal to the range threshold, the green LED is lit. If the range is less than the threshold the red LED is lit. The ranges should be compared to an accuracy of 1 cm. If no range measurement has yet been made, or if the measurement of the range failed for some reason, such as being beyond the 400cm limit, the red and green LEDs should go off and the blue LED should come on. The blue LED should stay on until there is a valid range measurement.
<img width="200" alt="Screenshot 2023-11-12 at 3 50 12 PM" src="https://github.com/araslan1/Rangefinder/assets/108353316/fa110beb-d26b-4179-9b45-20959fe35775">
