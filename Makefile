program_5: steve.c
	avr-gcc -mmcu=atmega328p -DF_CPU=16000000 -O2 -o steve.elf steve.c
	avr-objcopy -O ihex steve.elf steve.hex
	avr-size steve.elf
				  
#Flash the Arduino
#Be sure to change the device (the argument after -P) to match the device on your  computer
#On Windows, change the argument after -P to appropriate COM port
program: steve.hex
	avrdude -pm328p -P /dev/ttyACM0 -c arduino -F -u -U flash:w:steve.hex
							  
#remove build files
clean:
	rm -fr *.elf *.hex *.o

