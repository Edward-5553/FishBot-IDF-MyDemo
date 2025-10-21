#include "myi2c.h"
#include "driver/i2c.h"  /* 使用 i2c_master_probe 进行地址阶段 ACK 探测 */
static const char *TAG = "IIC";

i2c_master_bus_handle_t bus_handle;     /* 总线句柄 */

/**
 * @brief       初始化MYIIC
 * @param       无
 * @retval      ESP_OK:初始化成功
 */
esp_err_t myiic_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source                     = I2C_CLK_SRC_DEFAULT,  /* 时钟源 */
        .i2c_port                       = IIC_NUM_PORT,         /* I2C端口 */
        .scl_io_num                     = IIC_SCL_GPIO_PIN,     /* SCL管脚 */
        .sda_io_num                     = IIC_SDA_GPIO_PIN,     /* SDA管脚 */
        .glitch_ignore_cnt              = 7,                    /* 故障周期 */
        .flags.enable_internal_pullup   = true,                 /* 内部上拉 */
    };

    ESP_LOGD(TAG, "I2C 总线配置: port=%d, SDA=%d, SCL=%d, glitch_ignore=%d, pullup=%d",
             (int)IIC_NUM_PORT, (int)IIC_SDA_GPIO_PIN, (int)IIC_SCL_GPIO_PIN,
             i2c_bus_config.glitch_ignore_cnt, i2c_bus_config.flags.enable_internal_pullup);
    ESP_LOGD(TAG, "I2C 目标速率(设备级设置): %d Hz", IIC_SPEED_CLK);

    /* 新建I2C总线 */
    ESP_LOGI(TAG, "创建 I2C 主总线...");
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C 主总线创建成功, handle=%p", (void*)bus_handle);
    } else {
        ESP_LOGE(TAG, "I2C 主总线创建失败: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    return ESP_OK;
}


esp_err_t myiic_scan(void)
{
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C 总线未初始化，无法扫描");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "开始 I2C 扫描 (0x08..0x77, 地址阶段 ACK 探测)...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        /* 直接用端口级 API 探测地址阶段 ACK，不需要先 add_device */
        esp_err_t probe = i2c_master_probe(IIC_NUM_PORT, addr, 100);
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "发现设备: 0x%02X", addr);
            found++;
        } else {
            ESP_LOGV(TAG, "[0x%02X] NACK: %s", addr, esp_err_to_name(probe));
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "扫描结束，未发现设备");
    } else {
        ESP_LOGI(TAG, "扫描结束，共发现 %d 个设备", found);
    }
    return ESP_OK;
}