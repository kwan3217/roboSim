#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <inttypes.h>
#include <stdio.h>

int file_i2c;
int length;
unsigned char buffer[60] = {0};

int main() {
  setbuf(stdout,NULL); //Turn off buffering on stdout
  //----- OPEN THE I2C BUS -----
  if ((file_i2c = open("/dev/i2c-1", O_RDWR)) < 0)	{
    //ERROR HANDLING: you can check errno to see what went wrong
    printf("Failed to open the i2c bus");
  }

  int addr = 0x55;          //<<<<<The I2C address of the slave
  if (ioctl(file_i2c, I2C_SLAVE, addr) < 0) {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    //ERROR HANDLING; you can check errno to see what went wrong
  }

  //----- WRITE BYTES -----
  if (write(file_i2c, "\x10", 1) != 1) {
    //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    // ERROR HANDLING: i2c transaction failed
    printf("Failed to write to the i2c bus.\n");
  }

  //----- READ BYTES -----
  length = 2;			//<<< Number of bytes to read
  if (read(file_i2c, buffer, length) != length) {
    //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    //ERROR HANDLING: i2c transaction failed
    printf("Failed to read from the i2c bus.\n");
  } else {
    printf("Data read: %02x%02x\n", buffer[0],buffer[1]);
  }

  for(;;) {
    for(uint16_t i=0;i<100;i++) {
      buffer[0]=0x2C;
      buffer[1]=((i+100)<<0) & 0xFF;
      buffer[2]=((i+100)<<8) & 0xFF;
      buffer[3]=((199-i)<<0) & 0xFF;
      buffer[4]=((199-i)<<8) & 0xFF;
      length=5;
      int actual=write(file_i2c,buffer,length);
      if(actual!=length) {
        printf("Failed to write servo value (wanted to write %d, did write %d.\n",length,actual);
      } else {
        printf(".");
      }
      usleep(32174);
    }
  }
}
