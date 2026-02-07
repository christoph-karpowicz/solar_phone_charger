main:
	avr-gcc -g -Os -mmcu=attiny13a -c display.c
	avr-gcc -g -Os -mmcu=attiny13a -c main.c
	avr-gcc -g -mmcu=attiny13a -o main.elf main.o display.o
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avrdude -c usbasp -p t13 -P usb -U flash:w:main.hex