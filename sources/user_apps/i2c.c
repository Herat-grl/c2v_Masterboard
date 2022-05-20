/*
 * i2c.c
 *
 * @date 2019/08/09
 * @author Cosmin Tanislav
 */

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"

/*
 * Start the I2C device.
 *
 * @param dev points to the I2C device to be started, must have filename and addr populated
 *
 * @return - 0 if the starting procedure succeeded
 *         - negative if the starting procedure failed
 */
int i2c_start(struct I2cDevice* dev) {
	int fd;
	int rc;

	/*
	 * Open the given I2C bus filename.
	 */
	fd = open(dev->filename, O_RDWR);
	if (fd < 0) {
		rc = fd;
		goto fail_open;
	}

	/*
	 * Set the given I2C slave address.
	 */
	rc = ioctl(fd, I2C_SLAVE, dev->addr);
	if (rc < 0) {
		goto fail_set_i2c_slave;
	}

	dev->fd = fd;

	return 0;

fail_set_i2c_slave:
	close(fd);
fail_open:
	return rc;

}

/*
 * Read data from the I2C device.
 *
 * @param dev points to the I2C device to be read from
 * @param buf points to the start of buffer to be read into
 * @param buf_len length of the buffer to be read
 *
 * @return - number of bytes read if the read procedure succeeded
 *         - 0 if no bytes were read
 *         - negative if the read procedure failed
 */
int i2c_read(struct I2cDevice* dev, uint8_t *buf, size_t buf_len) {
	return read(dev->fd, buf, buf_len);
}

/*
 * Write data to the I2C device.
 *
 * @param dev points to the I2C device to be write to
 * @param buf points to the start of buffer to be written from
 * @param buf_len length of the buffer to be written

 * @return - number of bytes written if the write procedure succeeded
 *         - 0 if no bytes were written
 *         - negative if the read procedure failed
 */
int i2c_write(struct I2cDevice* dev, uint8_t *buf, size_t buf_len) {
	printf("buf lenth : %d\n", buf_len);
	return write(dev->fd, buf, buf_len);
}

/*
 * Read data from a register of the I2C device.
 *
 * @param dev points to the I2C device to be read from
 * @param reg the register to read from
 * @param buf points to the start of buffer to be read into
 * @param buf_len length of the buffer to be read
 *
 * @return - number of bytes read if the read procedure succeeded
 *         - 0 if no bytes were read
 *         - negative if the read procedure failed
 */
int i2c_readn_reg(struct I2cDevice* dev, uint8_t reg, uint8_t *buf, size_t buf_len) {
	int rc;

	/*
	 * Write the I2C register address.
	 */
	rc = i2c_write(dev, &reg, 1);
	if (rc <= 0) {
		printf("%s: failed to write i2c register address : %d \r\n", __func__, dev->addr);
		return rc;
	}

	/*
	 * Read the I2C register data.
	 */
	rc = i2c_read(dev, buf, buf_len);
	if (rc <= 0) {
		printf("%s: failed to read i2c register data\r\n", __func__);
		return rc;
	}

	return rc;
}

/*
 * Stop the I2C device.
 *
 * @param dev points to the I2C device to be stopped
 */
void i2c_stop(struct I2cDevice* dev) {
	/*
	 * Close the I2C bus file descriptor.
	 */
	close(dev->fd);
}
