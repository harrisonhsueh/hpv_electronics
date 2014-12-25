#include <RF24.h>
#define RF24_TRANSFER_SIZE 32

void setup() {
  begin();
  connect_master();
}
char *name = "template";
uint64_t master_write_address = 0x0F0F0F0F00; //master will read on 
uint64_t master_connection_address = 0x0F0F0F0F01;

void connect_master() {
  //send, listen, send, listen.... until we get a response from master
  //master uses pipe1 to establish connections, use pipe2 to flood out to all sensors, pipe3 to connect to privileged sensors, pipe4 to receive in
  openWritingPipe(master_write_address);
  write(start_msg, len(start_msg));
  openReadingPipe(1, master_connection_address);
  startListening();
  not_connected = true;
  while (not_connected) {
    if (available()) {
      read();
      not_connected = false;
    } else {
      wait(500);
      if (available()) {
        read();
        not_connected = false;
      }
    }
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
