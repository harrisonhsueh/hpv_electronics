#include "mbed.h"
#include <limits.h>
#include <stdio.h>
#define NO_SPOKE 0
#define SPOKE 1
Serial device(p9, p10); //tx, rx
DigitalOut xbeeout(p11);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
//DigitalIn light_in(P0_10);
Serial pc(USBTX, USBRX); // tx, rx
Serial xbee(p13, p14);

void wheelspeed_interrupt();
volatile int count = 0;
int prev_count = 0;
double speed = 0.0;
int num_spokes = 24;
int send_interval = 2; //interval between sending data via XBEE, in seconds
double circumference = 0.00130239; // in miles for 700x23c wheel w/ tire
int update_interval = 2000; // in ms
int last_action = NO_SPOKE;

/* Prints speed to terminal through a usb. */
void show_usbterm_speed(double speed) {
	pc.printf("speed: %f\n", speed);
}

/* Sends speed to XBEE. */
void send_xbee_speed(double speed) {
	xbee.printf("speed: %f\n", speed);
}

/* Shows the speed using the 4 leds as binary values.
in format 0b{led4,led3,led2,led1} */
void show_binary_speed(double speed) {
	led1 = 0;
	led2 = 0;
	led3 = 0;
	led4 = 0;
	if (speed >= 8) {
		led4 = 1;
		speed -= 8;
	}
	if (speed >= 4) {
		led3 = 1;
		speed -= 4;
	}
	if (speed >= 2) {
		led2 = 1;
		speed -= 2;
	}
	if (speed >= 1) {
		led1 = 1;
		speed -= 1;
	}
}
int main() {
	// convert units before to minimize calculations in critical section
	double speed_per_spoke = circumference * 3600 * 1000 / update_interval / num_spokes;
	int send_time_left = send_interval;
	device.baud(9600);
	device.attach(&wheelspeed_interrupt, Serial::RxIrq);
	pc.printf("Starting logging.\n");
	while(1) {
		wait_ms(update_interval / 2);
		led1 = 1;
		wait_ms(update_interval / 2);
		pc.printf("count: %d\n", count);
		NVIC_DisableIRQ(UART1_IRQn); // start critical section
		speed = count * speed_per_spoke;
		count = 0;
		NVIC_EnableIRQ(UART1_IRQn); //end critical section
		led1 = 0;
		led2 = 0;
		//show_binary_speed(speed);
		show_usbterm_speed(speed);
		send_time_left -= 1;
		if (send_time_left == 0) {
			send_xbee_speed(speed);
			send_time_left = send_interval;
		}
	}
}



void wheelspeed_interrupt() {
	led2 = 1;
	device.getc();
	count ++;
	return;
}
