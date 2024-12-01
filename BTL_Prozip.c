#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "unistd.h"
#include <stdio.h>


#define DHT_PIN GPIO_NUM_4
#define MQ2_PIN GPIO_NUM_18
#define FLAME_PIN GPIO_NUM_19
#define FAN_PIN GPIO_NUM_17  //Relay high level
#define BUZZER_PIN GPIO_NUM_23

#define TAG "MULTI_SENSOR"
#define TIMEOUT 1000 // 1ms timeout

#define SLAVE_ADDRESS_LCD 0x27 // change this according to ur setup

esp_err_t err;

#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO           GPIO_NUM_22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

char buffer[10];

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  
	data_t[1] = data_u|0x08;  
	data_t[2] = data_l|0x0C;  
	data_t[3] = data_l|0x08;  
	err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
	if (err!=0) ESP_LOGI(TAG, "Error in sending command");
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  
	data_t[1] = data_u|0x09;  
	data_t[2] = data_l|0x0D;  
	data_t[3] = data_l|0x09;  
	err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
	if (err!=0) ESP_LOGI(TAG, "Error in sending data");
}

void lcd_clear (void)
{
	lcd_send_cmd (0x01);
	usleep(5000);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	
	usleep(50000);  
	lcd_send_cmd (0x30);
	usleep(5000);  
	lcd_send_cmd (0x30);
	usleep(200);  
	lcd_send_cmd (0x30);
	usleep(10000);
	lcd_send_cmd (0x20);  
	usleep(10000);

 
	lcd_send_cmd (0x28); 
	usleep(1000);
	lcd_send_cmd (0x08); 
	usleep(1000);
	lcd_send_cmd (0x01);  
	usleep(1000);
	usleep(1000);
	lcd_send_cmd (0x06); 
	usleep(1000);
	lcd_send_cmd (0x0C); 
	usleep(1000);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}


void delay_us(uint32_t us) {
    uint32_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // Busy wait
    }
}


bool wait_for_level(int level, uint32_t timeout_us) {
    uint32_t start = esp_timer_get_time();
    while (gpio_get_level(DHT_PIN) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return false;
        }
    }
    return true;
}


void read_dht11_task(void *pvParameter) {
    while (1) {
        int16_t temperature = 0;
        int16_t humidity = 0;
        uint8_t data[5] = {0};
        char bu [3];

        
        gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DHT_PIN, 0);
        delay_us(20000); // 18ms for DHT11
        gpio_set_level(DHT_PIN, 1);
        delay_us(40); // 40us
        gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

        
        if (!wait_for_level(0, TIMEOUT) || !wait_for_level(1, TIMEOUT)) {
            ESP_LOGE(TAG, "Failed to get response from DHT11");
        } else {
           
            while(!wait_for_level(0, TIMEOUT));
            for (int i = 0; i < 40; i++) {
                if (!wait_for_level(1, TIMEOUT)) {
                    ESP_LOGE(TAG, "Data read timeout");
                    break;
                }
                delay_us(28); // Sampling point
                if (gpio_get_level(DHT_PIN) == 1) {
                    data[i / 8] |= (1 << (7 - (i % 8)));
                }
                if (!wait_for_level(0, TIMEOUT)) {
                    ESP_LOGE(TAG, "Data read timeout");
                    break;
                }
            }

            
            if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
                humidity = data[0];
                temperature = data[2];
                printf("DHT11 -> Humidity: %d%%, Temperature: %d\n", humidity, temperature);
                lcd_put_cur(0, 0);
                lcd_send_string("Temp: "); itoa(temperature, bu, 10);
                lcd_send_string(bu);
                lcd_send_string("C");
                lcd_put_cur(1, 0);
                lcd_send_string("Hum: "); itoa(humidity, bu, 10);
                lcd_send_string(bu);
                lcd_send_string("%");
            } else {
                ESP_LOGE(TAG, "DHT11 checksum failed");
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); 
    }
}


void sensor_control_task(void *pvParameter) {
    gpio_set_direction(MQ2_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FLAME_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        int mq2_value = gpio_get_level(MQ2_PIN); // 1: No smoke, 0: Smoke detected
        int flame_value = gpio_get_level(FLAME_PIN); // 1: No fire, 0: Fire detected

        if (mq2_value == 0) { 
            gpio_set_level(FAN_PIN, 1); 
            printf("MQ-2 -> Smoke detected! Fan ON.\n");
        } else {
            gpio_set_level(FAN_PIN, 0); 
            printf("MQ-2 -> No smoke. Fan OFF.\n");
        }

        if (flame_value == 0) { 
            gpio_set_level(FAN_PIN, 1); 
            gpio_set_level(BUZZER_PIN, 1);
            printf("Flame Sensor -> Fire detected! Fan ON, Buzzer ON.\n");
        } else {
            gpio_set_level(BUZZER_PIN, 0); 
            printf("Flame Sensor -> No fire. Buzzer OFF.\n");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
}

void app_main() {
    i2c_master_init();
    lcd_init();

    xTaskCreate(read_dht11_task, "read_dht11_task", 2048, NULL, 4, NULL);
    xTaskCreate(sensor_control_task, "sensor_control_task", 2048, NULL, 5, NULL);
}
