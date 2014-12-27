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
nRF24L01P rf24(p5, p6, p7, p8, p9, p10);

Ticker events;
Timeout timeout;
double speed = 0.0; //calculated speed
double cadence = 0.0;
char receive_buffer[RF24_TRANSFER_SIZE];
char send_buffer[RF24_TRANSFER_SIZE];
char *sensor_names[255] = {0};
void (*sensor_handlers[255])(char *data);

/* RF24 Handlers. They must all take in a char* parameter. */
void receiver_handler(char *data);
void speed_handler(char *data);
void cadence_handler(char *data);
void rear_lights_handler(char *data);
void front_lights_handler(char *data);
void shifter_handler(char *data);

/* Prints speed to terminal through a usb. */
void show_usbterm_speed() {
	pc.printf("speed: %f\n", speed);
	pc.printf("servo position: %f\n", pos);
}

/* Sends speed to XBEE. */
void send_xbee_speed() {
	sprintf(speed_buffer_val, "%f\n", speed);
}

/* Initialize xbee for telemetry. */
void telemetry_init() {
	// convert units before to minimize calculations in critical section
	pc.printf("Initializing.\n");
}

/* Initialize one sensor's values. */
void init_sensor(int id, char *name, void *handler) {
	sensor_names[id] = name;
	sensor_handlers[id] = handler;
}

/* initialize all rf24 sensor data. */
void rf24_init() {
	init_sensor(0, "receiver", &receiver_handler);
	init_sensor(1, "speed", &speed_handler);
	init_sensor(2, "cadence", &cadence_handler);
	init_sensor(3, "rear_lights", &rear_lights_handler);
	init_sensor(4, "front_lights", &front_lights_handler);
	init_sensor(5, "shifter", &shifter_handler);
	rf24.setRxAddress(0x0000000001, paddr_size, NRF24L01P_PIPE_P1);
	rf24.setRxAddress(0x0000000002, paddr_size, NRF24L01P_PIPE_P2);
	rf24.setRxAddress(0x0000000003, paddr_size, NRF24L01P_PIPE_P3);
	rf24.setRxAddress(0x0000000004, paddr_size, NRF24L01P_PIPE_P4);
	rf24.setRxAddress(0x0000000005, paddr_size, NRF24L01P_PIPE_P5);
	rf24.powerUp();
	rf24.setReceiveMode();
}

/* Initialize everything necessary for the scripts. */
void init() {
	telemetry_init();
	rf24_init();
}

/* Process RF24 input and send it to the correct handler. */
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
/* Main sending loop. */
int main() {
	init();
	pc.printf("Starting Logging.\n");
	events.attach(&send_xbee_speed, XBEE_SEND_INTERVAL);
	events.attach(&show_usbterm_speed, PC_SEND_INTERVAL);
	while(1) {
		if (rf24.readable()) {
			rf24.read(NRF24L01P_PIPE_P0, receive_buffer, RF24_TRANSFER_SIZE);
			process_rf_input();
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
	rf24.setTxAddress(pipe_addr, paddr_size);
	rf24.write(NRF24L01P_PIPE_P0, send_buffer, RF24_TRANSFER_SIZE);
}

/* Gets the id from the name of the sensor. */
uint8_t find_id(char *name) {
	uint8_t id = 0;
	while (strcmp(sensor_names[id], name)) {
		id += 1;
	}
	return id;
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
double get_speed(char *data) {
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

