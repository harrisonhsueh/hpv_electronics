#include <RF24.h>
#define RF24_TRANSFER_SIZE 32
#define MY_ADDR 255
RF24 rf24(CE, CS);

/* Run setup code. */
void setup() {
  begin();
  connect_master();
}

char *name = "template";
uint64_t master_general_address; //master will read on this address
uint64_t master_connection_address = 0x0000000001; //this is the address we write to connect to the master
char read_buffer[RF24_TRANSFER_SIZE];
char write_buffer[RF24_TRANSFER_SIZE];

void connect_master() {
  //send, listen, send, listen.... until we get a response from master
  //master uses pipe1 to establish connections, use pipe2 to flood out to all sensors, pipe3 to connect to privileged sensors, pipe4 to receive in
  rf24.openWritingPipe(master_connection_address);
  rf24.write(start_msg, len(start_msg));
  rf24.openReadingPipe(1, 0xFFFFFFFF01);
  rf24.startListening();
  not_connected = true;
  while (not_connected) {
    if (rf24.available()) {
      rf24.read(read_buffer, RF24_TRANSFER_SIZE);
      not_connected = false;
    } else {
      wait(500);
      if (rf24.available()) {
        rf24.read(read_buffer, RF24_TRANSFER_SIZE);
        not_connected = false;
      }
    }
  }
}

/* Writes data to master
 * Use this for sensor-type slaves that gather data locally and send to the master
 * Comment out the write_data line in loop function if not needed.
 */
void write_data() {
	/* Write code here. */
	rf24.stopListening();
	rf24.write(write_buffer, RF24_TRANSFER_SIZE);
	rf24.startListening();
}

/* Handles an incoming packet meant for this slave.
 * @data The data packet meant for this slave.
 */
void read_handler(char *data) {
	
}

/* Reads data from master
 * All slaves should use this, to receive command messages such as shutdown.
 */
void read_data() {
	if (rf24.available()) {
		rf24.read(read_buffer, RF24_TRANSFER_SIZE);
		if (read_buffer[0] == MY_ADDR) {
			read_handler(read_buffer + 2);
		}
	}
	
}
void loop() {
  // put your main code here, to run repeatedly:
	//send, sleep, send, sleep...
	read_data();
	write_data();
}
