main:
	avr-gcc -g -Os -mmcu=attiny13a -c display.c -o bin/display.o
	avr-gcc -g -Os -mmcu=attiny13a -c main.c -o bin/main.o
	avr-gcc -g -mmcu=attiny13a -o bin/main.elf bin/main.o bin/display.o
	avr-objcopy -j .text -j .data -O ihex bin/main.elf bin/main.hex
	avrdude -c usbasp -p t13 -P usb -U flash:w:bin/main.hex