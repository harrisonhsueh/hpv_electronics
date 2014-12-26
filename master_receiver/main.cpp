#include "mbed.h"
#include <limits.h>
#include <stdio.h>
#include "Servo.h"
#include "xbee.h"
#include "nRF24L01P.h"

#define RF24_TRANSFER_SIZE 32
#define XBEE_SEND_INTERVAL 2
#define PC_SEND_INTERVAL 1
#define paddr_size 5
#define MY_ADDR 0

//Serial spoke_sensor(p9, p10); //tx, rx
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial pc(USBTX, USBRX); // tx, rx
xbee xbee(p13, p14, p12);
//Servo servo(p21);
nRF24L01P rf_receiver(p5, p6, p7, p8, p9, p10);

Ticker events;
Timeout timeout;
void wheelspeed_interrupt();
volatile int count = 0; //wheelspeed_interrupt increments this, reset to 0 after send_interval
double speed = 0.0; //calculated speed
double cadence = 0.0;
int num_spokes = 24; //set to number of spokes on wheel, used for calculating speed
double circumference = 0.00130239; // in miles for 700x23c wheel w/ tire
double speed_per_spoke;
float pos;
float new_pos = 0.0;
char speed_buffer[20];
char *speed_buffer_val;
double shift_vals[11];
char receive_buffer[RF24_TRANSFER_SIZE];
char send_buffer[RF24_TRANSFER_SIZE];
char *sensor_names[255] = {0};
void (*sensor_handlers[255])(char *data);
/* Prints speed to terminal through a usb. */
void show_usbterm_speed() {
	pc.printf("speed: %f\n", speed);
	pc.printf("servo position: %f\n", pos);
}

/* Sends speed to XBEE. */
void send_xbee_speed() {
	sprintf(speed_buffer_val, "%f\n", speed);
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
void rf24_init() {
	init_sensor(0, "receiver", &receiver_handler);
	init_sensor(1, "speed", &speed_handler);
	init_sensor(2, "cadence", &cadence_handler);
	init_sensor(3, "rear_lights", &rear_lights_handler);
	init_sensor(4, "front_lights", &front_lights_handler);
	init_sensor(5, "shifter", &shifter_handler);
	setRxAddress(0x0000000001, paddr_size, NRF24L01P_PIPE_P1);
	setRxAddress(0x0000000002, paddr_size, NRF24L01P_PIPE_P2);
	setRxAddress(0x0000000003, paddr_size, NRF24L01P_PIPE_P3);
	setRxAddress(0x0000000004, paddr_size, NRF24L01P_PIPE_P4);
	setRxAddress(0x0000000005, paddr_size, NRF24L01P_PIPE_P5);
	rf_receiver.powerUp();
	rf_receiver.setReceiveMode();
}
void init_sensor(int id, char *name, void *handler) {
	sensor_names[id] = name;
	sensor_handlers[id] = handler;
}
void init() {
	spoke_sensor.baud(9600);
	telemetry_init();
	shift_init();
	rf24_init();
}
/* Main sending loop. */
int main() {
	init();
	int send_time_left = send_interval;
	pc.printf("Starting Logging.\n");
	events.attach(&send_xbee_speed, XBEE_SEND_INTERVAL);
	events.attach(&show_usbterm_speed, PC_SEND_INTERVAL);
	while(1) {
		pos = servo.read();
		servo.write(new_pos);
		new_pos = new_pos + 0.5;
		if (new_pos > 1.0) {
			new_pos = 0.0;
		}
		pc.printf("count: %d\n", count);
		while (rf_receiver.readable()) {
			rf_receiver.read(NRF24L01P_PIPE_P0, receive_buffer, RF24_TRANSFER_SIZE);
			process_rf_input();
		}
		/*NVIC_DisableIRQ(UART1_IRQn); // start critical section
		speed = count * speed_per_spoke;
		count = 0;
		NVIC_EnableIRQ(UART1_IRQn); //end critical section*/
	}
}

void process_rf_input() {
	uint8_t dest_addr = (uint8_t) receive_buffer[0];
	if (dest_addr == MY_ADDR) {
		void *handler;
		uint8_t src_addr = (uint8_t) receive_buffer[1];
		handler = sensor_handlers[src_addr];
		if (handler){
			handler(receive_buffer[2]);
		}
	}
}

/* Send to a sensor with name.
 * Sending format: [dest_address(1), src_address(1), data(30)]
 */
void send_sensor_name(char *name, char *data) {
	uint8_t id = find_id(name);
	send_sensor(id, data);
}
/* Send to a sensor with an id. */
void send_sensor(uint8_t id, char *data) {
	send_buffer[0] = id;
	send_buffer[1] = (uint8_t) 0;
	send_buffer[2] = data;
	uint64_t pipe_addr = (id << 8) & 1;
	rf_receiver.setTxAddress(pipe_addr, paddr_size);
	rf_receiver.write(NRF24L01P_PIPE_P0, send_buffer, RF24_TRANSFER_SIZE);
}

/* Gets the id from the name of the sensor. */
uint8_t find_id(char *name) {
	uint8_t id = 0;
	while (strcmp(sensor_names[id], name)) {
		id += 1;
	}
	return id;
}
void reset_led2() {
	led2 = 0;
}
/* Serial interrupt for light sensor
 * measures number of times light is interrupted by spoke to calculate wheelspeed.
 */
void wheelspeed_interrupt() {
	led2 = 1;
	spoke_sensor.getc();
	count ++;
	timeout.attach(&reset_led2, 1);
	return;
}

/* Serial interrupt for shift up. */
void shift_up() {
	led2 = 1;
	timeout.attach(&reset_led2, 1);
}	

/* RF24 Handlers. They must all take in a char* parameter. */

/* Gets the seqno from the data packet. */
unsigned int get_seqno(char *data) {
	uint8_t msb = (uint8_t) data[0];
	uint8_t lsb = (uint8_t) data[1];
	unsigned int seqno = msb * 255 + lsb;
	return seqno;
}

/* Sends an ack to the relevant sensors. */
void send_ack(int id, unsigned int seqno, char *data) {
	uint8_t msb = (uint8_t) seqno >> 8;
	uint8_t lsb = (uint8_t) seqno & 0x0F;
	char *send_data = (char *) malloc(RF24_TRANSFER_SIZE - 2);
	send_data[0] = msb;
	send_data[1] = lsb;
	sprintf(send_data + 2, "%s", data);
	send_sensor(id, send_data);
}

/* The receiver's handler should never really do anything.
 * If the receiver gets a message from itself, something went wrong.
 */
void receiver_handler(char *data) {
	led4 = 1; //We really shouldn't be doing anything. This is to warn us if this handler is ever triggered.
}

/* Get speed from data packet. */
double gete_speed(char *data) {
	double spd;
	sscanf(data, "%lf", &spd);
	return spd;
}

/* Speed handler
 * Receive data in the format [seqno(2), speed(8)]
 * speed is a double.
 */
unsigned int speed_seqno = 0;
char *spd_string = (char *) malloc(8);
void speed_handler(char *data) {
	unsigned int seqno = get_seqno(data);
	if (seqno > speed_seqno) {
		speed = get_speed(data); //should we update anything?
		sprintf(spd_string, "%3.4f", speed);
		send_ack(1, speed_seqno, spd_string);
	} else if (seqno == 0) {
		send_ack(1, 0, "ack");
		if (speed_seqno > 0) {
			send_ack(1, speed_seqno, spd_string);
		}
	}
}

/* Get cadence from data packet. */
double get_cadence(char *data) {
	double cad;
	sscanf(data, "%lf", &cad);
	return cad;
}

/* Cadence handler
 * Receive data in the format [seqno(2), cadence(8)]
 * cadence is a double
 */
unsigned int cadence_seqno = 0;
char *cad_string = (char *) malloc(8);
void cadence_handler(char *data) {
	unsigned int seqno = get_seqno(data);
	if (seqno > cadence_seqno) {
		cadence = get_cadence(data);
		sprintf(cad_string, "%3.4f", cadence);
		send_ack(2, cadence_seqno, cad_string);
	} else if (seqno == 0) {
		send_ack(2, 0, "ack");
		if (cadence_seqno > 0) {
			send_ack(2, cadence_seqno, cad_string);
		}
	}
}

/* Rear Light handler
 */
void rear_lights_handler(char *data) {
}

/* Front Light handler
 * Not sure what goes here yet. Probably messages like battery, change of status, etc
 */
void front_lights_handler(char *data) {
}

/* Shifter handler
 * not sure what goes here yet. Probably messages like battery, change of status, etc
 */
void shifter_handler(char *data) {
}

