#include "mbed.h"
#include <limits.h>
#include <stdio.h>
#include "Servo.h"
#include "xbee.h"

Serial spoke_sensor(p9, p10); //tx, rx
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX); // tx, rx
xbee xbee(p13, p14, p12);
Servo servo(p21);

void wheelspeed_interrupt();
volatile int count = 0; //wheelspeed_interrupt increments this, reset to 0 after send_interval
double speed = 0.0; //calculated speed
int num_spokes = 24; //set to number of spokes on wheel, used for calculating speed
int send_interval = 2; //interval between sending data via XBEE, in seconds
double circumference = 0.00130239; // in miles for 700x23c wheel w/ tire
int update_interval = 2000; // in ms
double speed_per_spoke;
float pos;
float new_pos = 0.0;
char speed_buffer[50];
char *speed_buffer_val;
double shift_vals[11];

/* Prints speed to terminal through a usb. */
void show_usbterm_speed(double speed, float pos) {
	pc.printf("speed: %f\n", speed);
	pc.printf("servo position: %f\n", pos);
}

/* Sends speed to XBEE. */
void send_xbee_speed(double speed) {
	sprintf(speed_buffer_val, "%f", speed);
	xbee.SendData(speed_buffer);
	//xbee.printf("speed: %f\n", speed);
	//pc.printf("XBEE SEND\n");
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

void telemetry_init() {
	// convert units before to minimize calculations in critical section
	pc.printf("Initializing.\n");
	speed_per_spoke = circumference * 3600 * 1000 / update_interval / num_spokes;
	spoke_sensor.attach(&wheelspeed_interrupt, Serial::RxIrq);
	sprintf(speed_buffer, "speed: ");
	speed_buffer_val = speed_buffer + 7;
}

void shift_init() {
	//shift_vals = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
	servo.calibrate(0.001, 0.0);
}
void init() {
	spoke_sensor.baud(9600);
	telemetry_init();
	shift_init();
}
/* Main sending loop. */
int main() {
	init();
	int send_time_left = send_interval;
	pc.printf("Starting Logging.\n");
	while(1) {
		pos = servo.read();
		servo.write(new_pos);
		new_pos = new_pos + 0.5;
		if (new_pos > 1.0) {
			new_pos = 0.0;
		}
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
		show_usbterm_speed(speed, pos);
		send_time_left -= 1;
		//if (send_time_left == 0) {
			send_xbee_speed(speed);
			//send_time_left = send_interval;
		//}
	}
}

/* Serial interrupt for light sensor
 * measures number of times light is interrupted by spoke to calculate wheelspeed.
 */
void wheelspeed_interrupt() {
	led2 = 1;
	spoke_sensor.getc();
	count ++;
	return;
}

/* Serial interrupt for shift up. */
void shift_up() {
	led2 = 1;
}	
