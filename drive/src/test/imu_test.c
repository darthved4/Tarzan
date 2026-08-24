#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/cobs.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

const struct device *const lower = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));
const struct device *const upper = DEVICE_DT_GET(DT_ALIAS(imu_upper_joint));
const struct device *const end = DEVICE_DT_GET(DT_ALIAS(imu_pitch_roll));
const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(latte_panda_uart));

struct inverse_msg msg_rx;

int pos;
const int MAX_MSG_LEN = sizeof(struct inverse_msg) + 2;
uint8_t rx_buf[sizeof(struct inverse_msg) + 2];
struct k_work work;
K_MSGQ_DEFINE(msgq_rx, (sizeof(struct inverse_msg) + 2), 50, 1);

void serial_cb(const struct device *uart_dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  if (!uart_irq_update(uart_dev)) {
    return;
  }
  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }
  while (uart_fifo_read(uart_dev, &c, 1) == 1) {
    if (c == 0x00 && pos > 0) {
      rx_buf[pos] = 0;
      if (pos != (sizeof(struct inverse_msg) + 2 - 1)) {
        pos = 0;
        continue;
      }
      if (k_msgq_put(&msgq_rx, rx_buf, K_MSEC(4)) != 0) {
        printk("Error: put fail\n");
      } else {
        k_work_submit(&work);
      }
      pos = 0;
    } else if (pos < sizeof(rx_buf)) {
      rx_buf[pos++] = c;
    }
  }
}
void inverse_mssg_work_handler(struct k_work *work_ptr) {
  uint8_t buf[sizeof(struct inverse_msg) + 2];
  if (k_msgq_get(&msgq_rx, buf, K_MSEC(4)) != 0) {
    printk("error: get failed\n");
    return;
  }
  cobs_decode_result result =
      cobs_decode((void *)&msg_rx, sizeof(msg_rx), buf, MAX_MSG_LEN - 1);
  if (result.status != COBS_DECODE_OK) {
    printk(":o COBS Decode Failed %d !!!!\n", result.status);
    return;
  }
}
void send_imu_data(uint8_t buf[], struct inverse_msg *data) {
  cobs_encode_result result =
      cobs_encode(buf, MAX_MSG_LEN, (void *)data, sizeof(struct inverse_msg));
  if (result.status != COBS_ENCODE_OK) {
    printk(":o COBS Encode Failed!!!\n");
    return;
  }
  buf[MAX_MSG_LEN - 1] = 0x00;
  for (int i = 0; i < sizeof(struct inverse_msg) + 2; i++) {
    uart_poll_out(dev, buf[i]);
  }
}

int main() {
  uint32_t dtr = 0;

  struct joint lowerIMU = {0};
  struct joint upperIMU = {0};
  struct joint endIMU = {0};

  k_work_init(&work, inverse_mssg_work_handler);

  uint8_t tx_buf[sizeof(struct inverse_msg) + 2];

  struct inverse_msg msg_tx;

  if (!device_is_ready(dev)) {
    printk("Uart device not ready");
  }
  int err = uart_irq_callback_user_data_set(dev, serial_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP)
      printk("Interrupt-driver UART API support not enabled");
    else if (err == -ENOSYS)
      printk("UART device does not support interrupt-driven API");
    else
      printk("Error setting UART callback");
  }
  if (usb_enable(NULL)) {
    return 0;
  }
  /*Device checks*/
  if (!device_is_ready(lower)) {
    printk("Device %s is not ready\n", lower->name);
  }
  if (!device_is_ready(upper)) {
    printk("Device %s is not ready\n", upper->name);
  }
  if (!device_is_ready(end)) {
    printk("Device %s is not ready\n", upper->name);
  }

  /*Calibration */
  printk("Calibrating IMU %s\n", lower->name);
  if (calibration(lower, &lowerIMU)) {
    printk("Calibration failed for device %s\n", lower->name);
  }
  printk("Calibrating IMU %s\n", upper->name);
  if (calibration(upper, &upperIMU)) {
    printk("Calibration failed for device %s\n", upper->name);
  }
  printk("Calibrating IMU %s\n", end->name);
  if (calibration(end, &endIMU)) {
    printk("Calibration failed for device %s\n", end->name);
  }
  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    printk("waiting for uart line contorl\n");
    k_sleep(K_MSEC(100));
  }
  printk("Initialization completed successfully!\n");

  uart_irq_rx_enable(dev);

  while (true) {
    process_mpu6050(lower, &lowerIMU);
    process_mpu6050(upper, &upperIMU);
    // process_mpu6050(end, &endIMU);
    msg_tx.turn_table = 0;
    msg_tx.first_link = lowerIMU.pitch;
    msg_tx.second_link = upperIMU.pitch;
    msg_tx.pitch = 0; // endIMU.pitch;
    msg_tx.roll = 0;  // endIMU.roll;
    msg_tx.x = 0;
    msg_tx.y = 0;
    msg_tx.z = 0;
    // printk("%f %f\n", lowerIMU.pitch, upperIMU.pitch);
    send_imu_data(tx_buf, &msg_tx);
    k_sleep(K_MSEC(10));
  }
}
