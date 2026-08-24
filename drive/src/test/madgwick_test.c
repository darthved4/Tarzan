#include <Tarzan/lib/arm.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define IMU_NODE DT_NODELABEL(imu0)
#define MAG_NODE DT_NODELABEL(mag0)

static const struct device *const imu_dev = DEVICE_DT_GET(IMU_NODE);
static const struct device *const mag_dev = DEVICE_DT_GET(MAG_NODE);

int main(void) {
  printk("\n=== IMU + MAG Madgwick test ===\n");

  if (!device_is_ready(imu_dev)) {
    printk("IMU device not ready\n");
    return 1;
  }
  if (!device_is_ready(mag_dev)) {
    printk("Magnetometer device not ready\n");
    return 1;
  }

  struct joint sensor;
  joint_init(&sensor);

  printk("Keep the IMU still while the gyroscope is calibrated.\n");
  if (calibrate_gyro(imu_dev, &sensor) != 0) {
    printk("Gyroscope calibration failed.\n");
    return 1;
  }

  printk("Rotate the magnetometer through every axis for calibration.\n");
  if (calibrate_magnetometer(mag_dev, &sensor) != 0) {
    printk("Magnetometer calibration failed.\n");
    return 1;
  }

  printk("Calibration complete; starting orientation updates.\n");

  unsigned int print_divider = 0U;
  while (true) {
    if (madgwick_filter(mag_dev, imu_dev, &sensor) == 0) {
      print_divider++;
      if (print_divider == 5U) {
        print_divider = 0U;
        printk("R=%7.2f P=%7.2f Y=%7.2f | "
               "G=[%.4f %.4f %.4f] | M=[%.3f %.3f %.3f]\n",
               (double)sensor.roll, (double)sensor.pitch,
               (double)sensor.yaw, (double)sensor.gyro[0],
               (double)sensor.gyro[1], (double)sensor.gyro[2],
               (double)sensor.mag[0], (double)sensor.mag[1],
               (double)sensor.mag[2]);
      }
    }

    k_sleep(K_MSEC(20));
  }

  return 0;
}
