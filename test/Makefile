MCU=atmega8
F_CPU=8000000
CC=avr-gcc
OBJCOPY=avr-objcopy
MEMCHECK=avr-size
IFLAGS=-I../ -I../uart -I../twi

CFLAGS=-std=c17 -Wall -Wextra -g -O3 -mmcu=${MCU} -DF_CPU=${F_CPU} ${IFLAGS}
TARGET=main
SRCS=main.c ../MPU9250.c ../uart/uart.c ../twi/twi.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex
	${MEMCHECK} -d ${TARGET}.bin

clean:
	rm -f *.bin *.hex