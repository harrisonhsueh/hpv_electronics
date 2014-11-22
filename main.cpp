#include "mbed.h"
#include <limits.h>
#include <stdio.h>
Serial device(p9, p10); //tx, rx
DigitalOut xbeeout(p11);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX); // tx, rx
Serial xbee(p13, p14);

void wheelspeed_interrupt();
volatile int count = 0; //wheelspeed_interrupt increments this, reset to 0 after send_interval
double speed = 0.0; //calculated speed
int num_spokes = 24; //set to number of spokes on wheel, used for calculating speed
int send_interval = 2; //interval between sending data via XBEE, in seconds
double circumference = 0.00130239; // in miles for 700x23c wheel w/ tire
int update_interval = 2000; // in ms

/* Prints speed to terminal through a usb. */
void show_usbterm_speed(double speed) {
	pc.printf("speed: %f\n", speed);
}

/* Sends speed to XBEE. */
void send_xbee_speed(double speed) {
	xbee.printf("speed: %f\n", speed);
}

/* Shows the speed using the 4 leds as binary values.
 * in format 0b{led4,led3,led2,led1}
 * This is deprecated, but could be useful if no terminal available
 */
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

/* Main sending loop. */
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
		show_usbterm_speed(speed);
		send_time_left -= 1;
		if (send_time_left == 0) {
			send_xbee_speed(speed);
			send_time_left = send_interval;
		}
	}
}

/* Serial interrupt for light sensor
 * measures number of times light is interrupted by spoke to calculate wheelspeed.
 */

void wheelspeed_interrupt() {
	led2 = 1;
	device.getc();
	count ++;
	return;
}
