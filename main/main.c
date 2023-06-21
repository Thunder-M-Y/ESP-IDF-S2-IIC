#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_SDA_IO 4

#define MPU_ADDR 0x68             // MPU6050 设备地址
#define MPU_CMD_WHO_AM_I 0x75     // MPU6050 设备确认寄存器
#define MPU_CMD_PWR_MGMT_1 0x6B   // 电源管理寄存器
#define MPU_CMD_SMPLRT_DIV 0x19   // 陀螺仪采样率分频器寄存器
#define MPU_CMD_CONFIG 0x1A       // 数字低通滤波器配置寄存器
#define MPU_CMD_GYRO_CONFIG 0x1B  // 陀螺仪配置寄存器
#define MPU_CMD_ACCEL_CONFIG 0x1C // 加速度传感器配置寄存器
#define MPU_CMD_ACCEL_XOUT_H 0x3B // 加速计X轴高字节数据寄存器

static const char *TAG = "MPU6050";

/**
 * @brief 初始化 I2C 总线
 */
void i2c_init()
{
    // 配置 IIC 总线参数
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // 主机方式启动IIC
        .sda_io_num = I2C_MASTER_SDA_IO,       // 设置数据引脚编号
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // 使能 SDA 上拉电阻
        .scl_io_num = I2C_MASTER_SCL_IO,       // 设置始终引脚编号
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // 使能 SCL 上拉电阻
        .master.clk_speed = I2C_MASTER_FREQ_HZ // 设置主频
    };

    // 初始化 IIC 总线
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    // 安装 IIC 设备，因为 slv_rx_buf_len 和 slv_tx_buf_len 中主机模式下会忽略，所以不配置，中断标志位也不做分配
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "IIC 初始化完毕!");
}

/**
 * @brief 初始化 MPU6050
 */

void MPU6050_init()
{
    uint8_t check;

    /* 创建设备检测命令链，查看 0x75 寄存器返回的数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬─┬─────────────┬─┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│   WHO_AM_I    │A│S│  DEV-ADDR   │R│A│    RES DATA   │N│P│
     * └─┴─────────────┴─┴─┴───────────────┴─┴─┴─────────────┴─┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1 1       7       1 1        8        1 1
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送地址，以及写指令，命令之后需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_WHO_AM_I, true);                   // 发送 WHO_MI_I 寄存器地址 0x75
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &check, I2C_MASTER_LAST_NACK);              // 读取数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);                                             // 删除指令

    if (check != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }

    ESP_LOGI(TAG, "MPU6050 检测到在线，开始初始化...");

    /* 创建电源管理控制命令链，写入数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│  PWR_MGMT_1   │A│   SEND DATA   │A│P│
     * └─┴─────────────┴─┴─┴───────────────┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1        8        1 1
     */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_PWR_MGMT_1, true);                 // 写入电源管理和复位控制
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据，将值设置为0x00，表示不进入休眠模式、开启循环测量、开启温度传感器。
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);

    // 初始化默认参数（设置陀螺仪采样率分频器）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送设备地址和写指令，需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_SMPLRT_DIV, true);                 // 发送陀螺仪采样率分频器寄存器地址
    i2c_master_write_byte(cmd, 0x07, true);                               // 设置陀螺仪采样率分频器的值为0x07
    i2c_master_stop(cmd);                                                 // 发送停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 执行I2C命令，超时时间为1秒
    i2c_cmd_link_delete(cmd);                                             // 删除命令链

    // 初始化默认参数（数字低通滤波器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_CONFIG, true);                     // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope：260Hz 0ms，Accelerometer：256Hz 0.98ms 8Khz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);

    // 初始化默认参数（陀螺仪配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_CONFIG, true);                // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope: +/- 250 dps
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);

    // 初始化默认参数（加速度传感器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_CONFIG, true);               // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Accelerometer: +/- 2g
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "MPU6050 初始化完毕!");
}

/**
 * @brief 获得 X 轴加速度
 * @return 返回 X 轴加速度
 */
int16_t get_accel_x()
{
    union
    {
        uint8_t bytes[4];
        int16_t value;
    } data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}
float get_tem()
{
    union
    {
        uint8_t bytes[4];
        int16_t value;
    } data;
    double tem=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, 0x41, true);                               // 写入寄存器地址，这个寄存器是温度的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    tem=(double)data.value/340+36.53;
    return (float)tem;
}

void app_main(void)
{

    i2c_init();     // 初始化 IIC
    MPU6050_init(); // 初始化 MPU6050
    ESP_LOGI(TAG, "准备采集 数据:");
    vTaskDelay(100); // 等待一秒钟开始获取
    while (1)
    {
        printf("%d\n", get_accel_x());
        printf("温度%.2f\n", get_tem());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}
