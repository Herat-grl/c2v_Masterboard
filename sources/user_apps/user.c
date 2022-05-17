#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "../kernel_modules/pcie/altera_dma_cmd.h"
#include <unistd.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>

#include <asm/termios.h> /*termio.h for serial IO api*/



#define BUFFER_LENGTH 40
#define TIME_STAMP
#define DEEP_DEBUG

#define DMA_PCIE_X8 1
#define DMA_PCIE_X4 2
#define DMA_UART 3
#define DMA_SPI 4
#define DMA_I2C 5
#define GPIO_PIN 6

#define DATA_MAX_SIZE 512

char sendBuff[DATA_MAX_SIZE];
char readBuff[DATA_MAX_SIZE];

char menu_flag;
char test_input;

static const char *spi_device = "/dev/spidev0.0";
static const char *uart_device = "/dev/ttyTHS0";
static const char *pcie_device_x8 = "/dev/altera_dma_x8";
static const char *pcie_device_x4 = "/dev/altera_dma_x4";
static const char *gpio = "/dev/phy_intr";
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t spi_speed;
static uint32_t uart_speed;
static uint16_t delay;
static int verbose;

uint8_t default_tx[DATA_MAX_SIZE] ; 

uint8_t default_rx[DATA_MAX_SIZE] = {0, };

char *input_tx;
struct timespec start_uart_w, stop_uart_w, start_uart_r, stop_uart_r, start, stop;
uint64_t elapsed = 0, start_time = 0, stop_time = 0, elapsed_rw = 0, elapsed_r = 0, elapsed_w = 0;
int count;
int status_flag = 0, inval_flag = 0;

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch == 27)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int check_dump()
{
	int i, flag = 1;

	printf("status flag : %d\n", status_flag);
#ifdef DEEP_DEBUG
	for(i=0; i<DATA_MAX_SIZE-1; i++) {
		printf("(%#x %#x)\n", default_tx[i], default_rx[i+1]);
	}
#endif
	for(i=0; i<DATA_MAX_SIZE-1; i++) {
		if(default_tx[i] != default_rx[i+1]) {
			status_flag = 1;
			count++;
			break;
		}
	}

	if(status_flag) {
		printf("Data does not matched!\n");
	} else {
		printf("Data match!!\n");
	}

	return status_flag;
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = spi_speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

#ifdef TIME_STAMP
	/* POSIX.1-2008 way */
	if (clock_gettime(CLOCK_REALTIME,&start))
	{
		printf("Failed to get Start Time!\n");
	}
#endif

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

#ifdef TIME_STAMP
	/* POSIX.1-2008 way */
	if (clock_gettime(CLOCK_REALTIME,&stop))
	{
		printf("Failed to get Stop Time!\n");
	}
#endif
	if (ret < 1)
		printf("can't send spi message\n");

}

/*
 * The values for speed are
 * B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800, etc
 *
 *  The values for parity are 0 (meaning no parity),
 * PARENB|PARODD (enable parity and use odd),
 * PARENB (enable parity and use even),
 * PARENB|PARODD|CMSPAR (mark parity),
 * and PARENB|CMSPAR (space parity).
 * */

/*
 * if waitTime  < 0, it is blockmode
 *  waitTime in unit of 100 millisec : 20 -> 2 seconds
 */
int SetInterfaceAttribs(int fd, int speed, int parity, int waitTime)
{
	int isBlockingMode;
	struct termios2 tty;
	speed_t get_speed;

	isBlockingMode = 0;
	if(waitTime < 0 || waitTime > 255)
		isBlockingMode = 1;

	memset (&tty, 0, sizeof tty);

	ioctl(fd, TCGETS2, &tty);
#if 0
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
			| INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cflag &= ~(CSIZE | PARENB);
	tty.c_cflag |= CS8 | CREAD | CLOCAL;
	tty.c_cc[VTIME] = 1;
	tty.c_cc[VMIN] = 0;
	tty.c_ispeed = uart_speed;
	tty.c_ospeed = uart_speed;
#else
	tty.c_oflag &= ~OPOST;	//added output flag which is required
	tty.c_cflag &= ~CBAUD;
    	tty.c_cflag |= BOTHER;
	tty.c_cflag |= CLOCAL;
	tty.c_cflag |= CSTOPB;
	tty.c_iflag = 0;
	tty.c_ispeed = speed;
	tty.c_ospeed = speed;
#endif
//	printf("tty.c_cflag : %d \n",tty.c_cflag);
//	printf("tty.c_iflag : %d \n",tty.c_iflag);
//	printf("tty.c_ispeed : %d \n",tty.c_ispeed);
//	printf("tty.c_ospeed : %d \n",tty.c_ospeed);

	ioctl(fd, TCSETS2, &tty);

	return 0;
}/*SetInterfaceAttribs*/

int data_check()
{
	int i;

#ifdef DEEP_DEBUG
	for(i = 0; i< DATA_MAX_SIZE; i++) {
		printf("(%#x %#x)\n", sendBuff[i], readBuff[i]);
	}
#endif

	for(i = 0; i< DATA_MAX_SIZE; i++) {
		if(readBuff[i] != sendBuff[i]) {
		//	printf("Sent and Recv Data both are not matched : %d!\n", i);
		//	printf("(%#x %#x)\n", sendBuff[i], readBuff[i]);
			status_flag = 1;
			break;
		}
	}
//	if(status_flag == 0)
//		printf("Sent And Recv Data Matched\n");

}

void *sendThread(void *parameters)
{
	int fd;

	ssize_t len, recv_len;

	fd = *((int*)parameters);

	for(int i = 0; i < DATA_MAX_SIZE; i++)
		sendBuff[i] = i;

#ifdef TIME_STAMP
	/* POSIX.1-2008 way */
	if (clock_gettime(CLOCK_REALTIME,&start_uart_w))
	{
		printf("Failed to get Time!\n");
	}
	/* seconds, multiplied with 1 million */
#endif

	while(1)
	{
		len = write(fd, &sendBuff[0], DATA_MAX_SIZE);

	//	printf("Sent Len = %d \n", len);

		if(len == DATA_MAX_SIZE)
		{
#ifdef TIME_STAMP
			/* POSIX.1-2008 way */
			if (clock_gettime(CLOCK_REALTIME,&stop_uart_w))
			{
				printf("Failed to get Time!\n");
			}
			/* seconds, multiplied with 1 million */
#endif
			break;
		}

		// sleep enough to transmit the length plus receive 25:
		// approx 100 uS per char transmit
		usleep((strlen(&sendBuff[0]) + 25) * 100);

		usleep(500*1000);
	}/*while*/

#ifdef TIME_STAMP
	/* POSIX.1-2008 way */
	if (clock_gettime(CLOCK_REALTIME,&start_uart_r))
	{
		printf("Failed to get Time!\n");
	}
#endif
	pthread_exit(0);
}/*sendThread */

void *readThread(void *parameters)
{
	int fd;
	ssize_t recv_len = 0;
	fd = *((int*)parameters);
	ssize_t len;
	int i;

	while(1)
	{
		do
		{
			len = read(fd, &readBuff[recv_len], DATA_MAX_SIZE);
			recv_len += len;
		}while(recv_len < DATA_MAX_SIZE);

		if(recv_len == DATA_MAX_SIZE)
		{
#ifdef TIME_STAMP
			/* POSIX.1-2008 way */
			if (clock_gettime(CLOCK_REALTIME,&stop_uart_r))
			{
				printf("Failed to get Time!\n");
			}
			/* seconds, multiplied with 1 million */
			break;
#endif
		}

		if (-1 == len)
		{
			switch(errno)
			{
				case EAGAIN:
					printf("__FUNCTION__ = %s, __LINE__ = %d\n", __FUNCTION__, __LINE__);
					usleep(5*1000);
					continue;
					break;

				default:
					printf("__FUNCTION__ = %s, __LINE__ = %d\n", __FUNCTION__, __LINE__);
					pthread_exit(0);
					break;
			}
		}

		if(len == 0)
		{
			printf("devic is lost!\n"); /*device maybe be unplugged*/
			exit(0);
		}/*if*/
		if(recv_len == DATA_MAX_SIZE)
			break;
	}/*while*/

	data_check();

	pthread_exit(0);
}/*readThread */

void red () {
  printf("\033[1;31m");
}
void reset () {
  printf("\033[0m");
}


void grl_print_menu (char *buf) {
	system("clear");
	red();
	printf("|-------------------------NOTE----------------------|\n");
	printf("| Please ensure you have loaded specific jic/sof    |\n");
	printf("| file to test specific periferal as below          |\n");
	printf("| --> PCIe x8 	: PCIe_gen3_x8.jic                  |\n");
	printf("| --> PCIe x4 	: PCIe_gen3_x4.jic                  |\n");
	printf("| --> PCIe UART : Uart.sof                          |\n");
	printf("| --> PCIe SPI 	: Spi.sof                           |\n");
	printf("|                                                   |\n");
	printf("|UART Max speed : 12 MbPS                           |\n");
	printf("|SPI max speed : 50 MbPS                            |\n");
	printf("|---------------------------------------------------|\n\n");
	reset();
	printf("\n**********************************************\n");
	printf("** ALTERA 256b DMA driver                   **\n");
	printf("** version %s                             **\n", ALTERA_DMA_DRIVER_VERSION);
	printf("** %d) start DMA                             **\n", ALTERA_CMD_START_DMA                         );
	printf("** %d) exit                                 **\n", ALTERA_EXIT                                  );
	printf("**********************************************\n");
	
	//printf("Random			? %d\n", ((struct dma_status *)buf) -> rand);
	printf("\n\n");
	if(inval_flag) {
		printf("invalid option selested\n");
		inval_flag = 0;
	}
	if(menu_flag) {
		if(test_input == 1) {
			printf("Access On Chip RAM	? %d\n\n", ((struct dma_status *)buf)->onchip);
			printf("PCI x8 Read Time               : %ld s and %ld us\n", ((struct dma_status *)buf)->read_time.tv_sec, ((struct dma_status *)buf)->read_time.tv_usec);
			printf("PCI x8 read Throughput         : %f GB/S\n", (((struct dma_status *)buf)->length_transfer*0.954)/(((struct dma_status *)buf)->read_time.tv_usec + 1000000*((struct dma_status *)buf)->read_time.tv_sec ));
			printf("PCI x8 Write Time              : %ld s and %ld us\n", ((struct dma_status *)buf)->write_time.tv_sec, ((struct dma_status *)buf)->write_time.tv_usec);
			printf("PCI x8 Write Throughput        : %f GB/S\n\n", (((struct dma_status *)buf)->length_transfer*0.954)/(((struct dma_status *)buf)->write_time.tv_usec + 1000000*((struct dma_status *)buf)->write_time.tv_sec ));
		} else if(test_input == 2) {
			printf("Access On Chip RAM	? %d\n\n", ((struct dma_status *)buf)->onchip);
			printf("PCI x4 Read Time               : %ld s and %ld us\n", ((struct dma_status *)buf)->read_time.tv_sec, ((struct dma_status *)buf)->read_time.tv_usec);
			printf("PCI x4 read Throughput         : %f GB/S\n", (((struct dma_status *)buf)->length_transfer*0.954)/(((struct dma_status *)buf)->read_time.tv_usec + 1000000*((struct dma_status *)buf)->read_time.tv_sec ));
			printf("PCI x4 Write Time              : %ld s and %ld us\n", ((struct dma_status *)buf)->write_time.tv_sec, ((struct dma_status *)buf)->write_time.tv_usec);
			printf("PCI x4 Write Throughput        : %f GB/S\n\n", (((struct dma_status *)buf)->length_transfer*0.954)/(((struct dma_status *)buf)->write_time.tv_usec + 1000000*((struct dma_status *)buf)->write_time.tv_sec ));
		} else if(test_input == 3) {
			elapsed_rw = 1000000000 * (stop_uart_r.tv_sec - start_uart_w.tv_sec) + stop_uart_r.tv_nsec - start_uart_w.tv_nsec;
			elapsed_r = 1000000000 * (stop_uart_r.tv_sec - start_uart_r.tv_sec) + stop_uart_r.tv_nsec - start_uart_r.tv_nsec;
			elapsed_w = 1000000000 * (stop_uart_w.tv_sec - start_uart_w.tv_sec) + stop_uart_w.tv_nsec - start_uart_w.tv_nsec;

			if(status_flag)
				printf("Data Does not Match!\n");
			else
				printf("Data Matched!\n");

			status_flag = 0;

			printf("UART read time       :   %ld   ns\n", elapsed_r);
			printf("UART write time      :   %ld   ns\n", elapsed_w);
			printf("UART read-write time :   %ld   ns\n\n", elapsed_rw);
		} else if(test_input == 4) {
			elapsed = 1000000000 * (stop.tv_sec - start.tv_sec) + stop.tv_nsec - start.tv_nsec;
			if(status_flag)
				printf("Data Does not Match!\n");
			else
				printf("Data Matched!\n");
			status_flag = 0;

			printf("SPI read-write time :   %ld  ns\n\n", elapsed);
		} else if(test_input == 5) {
			printf("I2C read time       :     us\n");
			printf("I2C write time      :     us\n");
			printf("I2C read-write time :     us\n\n");
		} else {
			if(status_flag) {
				printf("GPIO pin test : failed\n");
			} else {
				printf("GPIO pin test : passed\n");
			}
		}
		menu_flag = 0;
	} 
	printf("# ");

}

int test_dma_pcie_x8(char *buf) {
	struct dma_cmd cmd;
	int num;
	cmd.usr_buf_size = sizeof(struct dma_status);

	ssize_t fd = open (pcie_device_x8, O_RDWR);
	if (fd == -1) {
		printf ("Couldn't open the device.\n");
		return 0;
	} else {
		printf ("Opened the device: file handle #%lu!\n", (long unsigned int)fd);
	}

	printf("press 1: autofill\n");
	printf("      0: self writing\n");
	scanf("%d", &num);
	if(num) {		//to test with autofill data with FPGA
        cmd.cmd = ALTERA_CMD_ENA_DIS_READ;
        cmd.buf = buf;
        write (fd, &cmd, 0);
	}

	cmd.cmd = ALTERA_CMD_READ_STATUS;
	cmd.buf = buf;
	write (fd, &cmd, 0);

	ioctl(fd, ALTERA_IOCX_START);
	ioctl(fd, ALTERA_IOCX_WAIT);
	cmd.cmd = ALTERA_CMD_READ_STATUS;
	cmd.buf = buf;
	write (fd, &cmd, 0);
	menu_flag = 1;

	if(num) {	// to clear autofill test case
        cmd.cmd = ALTERA_CMD_ENA_DIS_READ;
        cmd.buf = buf;
        write (fd, &cmd, 0);
	}

	grl_print_menu(buf);
	close(fd);
	return 0;
}

int test_dma_pcie_x4(char *buf) {
	struct dma_cmd cmd;
	cmd.usr_buf_size = sizeof(struct dma_status);

	ssize_t fd = open (pcie_device_x4, O_RDWR);
	if (fd == -1) {
		printf ("Couldn't open the device.\n");
		return 0;
	} else {
		printf ("Opened the device: file handle #%lu!\n", (long unsigned int)fd);
	}

	// cmd.cmd = ALTERA_CMD_READ_STATUS;
	// cmd.buf = buf;
	// write (fd, &cmd, 0);

	ioctl(fd, ALTERA_IOCX_START);
	ioctl(fd, ALTERA_IOCX_WAIT);
	cmd.cmd = ALTERA_CMD_READ_STATUS;
	cmd.buf = buf;
	write (fd, &cmd, 0);
	menu_flag = 1;

	grl_print_menu(buf);
	close(fd);
	return 0;
}

int test_dma_uart(char *buf) {
	int fd;

	menu_flag = 1;

	fd = open(uart_device, O_RDWR | O_NOCTTY);
	if(0 > fd)
	{
		printf("can't open device\n");
		exit(-1);
	}/*if */

	SetInterfaceAttribs(fd, uart_speed, 0, 20); /* set speed to 57600 bps, 8n1 (no parity), timeout 2 secs*/

#if 0			/*temprary fix for UART 1st boot not work */
	close(fd);

	fd = open(uart_device, O_RDWR | O_NOCTTY);
	if(0 > fd)
	{
		printf("can't open device\n");
		exit(-1);
	}/*if */
#endif
	pthread_t readThread_t, sendThread_t;  /* thread variables */

	pthread_create(&readThread_t, NULL, (void *)readThread, (void *)&fd);
	pthread_create(&sendThread_t, NULL, (void *)sendThread, (void *)&fd);

	pthread_join(readThread_t, NULL);
	pthread_join(sendThread_t, NULL);

	close(fd);

	menu_flag = 1;

	grl_print_menu(buf);
	return 0;
}

int test_dma_spi(char *buf) {
	int ret = 0, match=0;
	int fd, i;

	for(i=0; i<DATA_MAX_SIZE; i++) {
		default_tx[i] = i+1;
	}

	fd = open(spi_device, O_RDWR);
	if (fd < 0) {
		printf("can't open device\n");
		exit(-1);
	}

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printf("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		printf("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		printf("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
	if (ret == -1)
		printf("can't get max speed hz");

	//printf("spi mode: 0x%x\n", mode);
	//printf("bits per word: %d\n", bits);
	//printf("max speed: %d Hz (%d KHz)\n", spi_speed, spi_speed/1000);

	transfer(fd, default_tx, default_rx, sizeof(default_tx));

	start_time = (start.tv_sec * 1000000000) + start.tv_nsec;
	stop_time = (stop.tv_sec * 1000000000) + stop.tv_nsec;

	match = check_dump();
	if(match) 
		printf("data not Matched!\n");
	else
		printf("Data matched : %d\n", count);

	menu_flag = 1;
	close(fd);
	grl_print_menu(buf);
	return 0;
}

int test_dma_i2c(char *buf) {
	menu_flag = 1;

	grl_print_menu(buf);
	return 0;
}

int test_gpio_pins(char *buf) {
	int num, data;
    char line[BUFFER_LENGTH];

    ssize_t fd = open (gpio, O_RDWR);
    if (fd == -1) {
        printf ("Couldn't open the device.\n");
        return 0;
    } else {
        printf ("Opened the device: file handle #%lu!\n", (long unsigned int)fd);
    }
    
    printf("       1 : test gpio9 & gpio15 with 0 value\n");
    printf("       2 : test gpio9 & gpio15 with 1 value\n");
    printf("       3 : test gpio25 & gpio16 with 0 value\n");
    printf("       4 : test gpio25 & gpio16 with 1 value\n");    
    scanf("%d", &num);

    if(num == 1) {
        buf[0] = 0;
        buf[1] = 1;
        read (fd, buf, 2);
        printf("value : %d\n", buf[0]);
    } else if(num == 2) {
        buf[0] = 1;
        buf[1] = 1;
        read (fd, buf, 2);
        printf("value : %d\n", buf[0]);
    } else if(num == 3) {
        buf[0] = 0;
        buf[1] = 2;
        read (fd, buf, 2);
        printf("value : %d\n", buf[0]);
    } else if(num == 4) {
        buf[0] = 1;
        buf[1] = 2;
        read (fd, buf, 2);
        printf("value : %d\n", buf[0]);
    } else {
        printf("invalid input\n");
		return -1;
    }
	close(fd);
	status_flag = buf[0];
	menu_flag = 1;
	grl_print_menu(buf);
    return 0;
}

int test_dma_periferal(char *buf) {
	switch(test_input)
	{
		case DMA_PCIE_X8:
			test_dma_pcie_x8(buf);
			break;
		case DMA_PCIE_X4:
			test_dma_pcie_x4(buf);
			break;
		case DMA_UART:
			printf("please enter speed for test UART : ");
			scanf("%d", & uart_speed);
			test_dma_uart(buf);
			break;
		case DMA_SPI:
			printf("please enter speed for test SPI : ");
			scanf("%d", & spi_speed);
			test_dma_spi(buf);
			break;
		case DMA_I2C:
			test_dma_i2c(buf);
			break;
		case GPIO_PIN:
			test_gpio_pins(buf);
			break;
		default:
			printf("not valid test periferal\n");
			inval_flag = 1;
			grl_print_menu(buf);
	}
	return 0;
}

int main() {
	char *buf = malloc(sizeof(struct dma_status));
	struct dma_cmd cmd;
	cmd.usr_buf_size = sizeof(struct dma_status);
	char line[BUFFER_LENGTH];
	int num_input;
	int i, loop_num;
	int j=1;
	int test;

	printf ("Try to open the device...\n");
	system("clear");
	grl_print_menu(buf);
	do{
		scanf("%d", &num_input);
		switch(num_input)
		{
			case ALTERA_EXIT:
				printf("EXIT\n");
				break;

			case ALTERA_CMD_START_DMA:
				printf("enter : \n\t# 1. test PCI x8\n\t# 2. test PCI x4\n\t# 3. Test UART\n\t# 4. Test SPI\n\t# 5. Test I2C\n\t# 6. Test GPIO\n");
				scanf("%d", &test_input);
				test_dma_periferal(buf);
				break;

			default:
				printf("%d is an invalid command\n", num_input);
				inval_flag = 1;
				grl_print_menu(buf);
		}
	}while (num_input != ALTERA_EXIT);

	return 0;
}
