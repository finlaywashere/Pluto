#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int main(){
	int usb = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	struct termios tty;
	memset(&tty,0,sizeof(tty));
	if(tcgetattr(usb,&tty) != 0){
		printf("Erorr1!\n");
		return 1;
	}
	cfsetospeed(&tty,(speed_t)B115200);
	cfsetispeed(&tty,(speed_t)B115200);
	tty.c_cflag &= ~PARENB;			// Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;		   // no flow control
	tty.c_cflag |= CREAD | CLOCAL;	 // turn on READ & ignore ctrl lines
	cfmakeraw(&tty);
	tcflush(usb, TCIFLUSH);
	if(tcsetattr(usb,TCSANOW,&tty) != 0) {
		printf("Error2!\n");
		return 2;
	}
	unsigned char buffer[17];
	int n = 0;
	int checksum = 0;
	while(1){
		n += read(usb,&buffer[n],1);
		if(n == 1){
			if(buffer[0] != 0xFF){
				n = 0;
				continue;
			}
		}
		if(n != 17){
			continue;
		}
		checksum = 0;
		for(int i = 0; i < 16; i++){
			checksum += buffer[i];
		}
		checksum %= 255;
		if(checksum != buffer[16]){
			printf("Warning: Checksums do not match! %d vs %d. Instruction %d\n",checksum,buffer[16],buffer[1]);
			for(int i = 0; i < 17; i++){
				printf("%d ",buffer[i]);
			}
			printf("\n");
			n = 0;
			continue;
		}
		printf("Instruction %d: ",buffer[1]);
		for(int i = 0; i < 17; i++){
				printf("%d ",buffer[i]);
		}
		printf("\n");
		n = 0;
	}
}
