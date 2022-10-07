Components used:
	1. Two 1000 RPM motor
            2. Five I.R sensors
	3. Arduino UNO
	4. L298N motor driver
	5. Jumper wires
	6. Metal chassis and two tires
	7. Caster wheel
	8. Three 3.7V  batteries connected in series

Connections:

	I.R output - > A0 - A4.

	Arduino's 5v - > vcc of ir sensors.

	Arduino's gnd - > gnd of ir sensors.

	Motor A inputs - > 2 and 3.

	Motor B inputs - > 4 and 5.

	Enable  A - > 10.

	Enable B - > 11.

	Arduino's vcc port and Gnd is connected to 5v of L298N motor driver and Gnd. 

	L298N motor driver's 12v port and Gnd is directly connected to the battery(10V).

	Output 1 and output 2 is connected to one geared Motor (1000rpm,12V).

	Output 3 and output 4 is connected to another geared motor (1000rpm, 12V).

Working:

The robot first performs a dry run to find the shortest path and store it in the memory and then perform the actual run from start to finish through the shortest path.
