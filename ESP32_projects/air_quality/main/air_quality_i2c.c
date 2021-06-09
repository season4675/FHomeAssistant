
#include "air_quality_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char *TAG = "air_tvoc";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS                                            \
  1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO                                                       \
  CONFIG_I2C_SLAVE_SCL /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO                                                       \
  CONFIG_I2C_SLAVE_SDA /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM                                                          \
  I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size   \
                                                */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size   \
                                                */

#define I2C_MASTER_SCL_IO GPIO_NUM_23 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_18 /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                                                         \
  I2C_NUMBER(1)                     /*!< I2C port number for master dev        \
                                     */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define CS811_SENSOR_ADDR                                                      \
  0x23 /*!< slave address for CS811 sensor */
#define CS811_CMD_START 0x01 /*!< Operation mode */
#define ESP_SLAVE_ADDR                                                         \
  0x23 /*!< ESP32 slave address, you can set any 7bit      \
                              value */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

static SemaphoreHandle_t i2c_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read
 * them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte +
 * nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @note cannot use master read slave on esp32c3 because there is only one i2c
 * controller on esp32c3
 */
static esp_err_t __attribute__((unused))
i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 * @note cannot use master write slave on esp32c3 because there is only one i2c
 * controller on esp32c3
 */
static esp_err_t __attribute__((unused))
i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len) {
  int i;
  for (i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h,
                                        uint8_t *data_l) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, CS811_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, CS811_CMD_START, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, CS811_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data_h, ACK_VAL);
  i2c_master_read_byte(cmd, data_l, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void i2c_tvoc_task(void *arg) {
  int ret;
  uint32_t task_idx = (uint32_t)arg;
#if !CONFIG_IDF_TARGET_ESP32C3
  int i = 0;
  uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
#endif //! CONFIG_IDF_TARGET_ESP32C3
  uint8_t sensor_data_h, sensor_data_l;
  int cnt = 0;
  while (1) {
    ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
    ret =
        i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
    xSemaphoreTake(i2c_mux, portMAX_DELAY);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      printf("*******************\n");
      printf("TASK[%d]  MASTER READ SENSOR( CS811 )\n", task_idx);
      printf("*******************\n");
      printf("data_h: %02x\n", sensor_data_h);
      printf("data_l: %02x\n", sensor_data_l);
      printf("sensor val: %.02f [Lux]\n",
             (sensor_data_h << 8 | sensor_data_l) / 1.2);
    } else {
      ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...",
               esp_err_to_name(ret));
    }
    xSemaphoreGive(i2c_mux);
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) /
               portTICK_RATE_MS);
    //---------------------------------------------------
  }
  vSemaphoreDelete(i2c_mux);
  vTaskDelete(NULL);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
      // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_*
      // flags to choose i2c source clock here. */
  };
  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
  if (err != ESP_OK) {
    return err;
  }
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

int start_tvoc(void) {
  ESP_LOGI(TAG, "%s line:%d start -->", __func__, __LINE__);

  i2c_mux = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(i2c_master_init());
  xTaskCreate(i2c_tvoc_task, "i2c_tvoc_task_0", 1024 * 2, (void *)0, 10, NULL);

  ESP_LOGI(TAG, "%s line:%d done", __func__, __LINE__);

  return 0;
}