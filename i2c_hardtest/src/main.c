#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* I2Cデバイスノード（TWIM21を使用） */
#define I2C_DEV_NODE DT_NODELABEL(i2c21)

/* スレーブアドレス（7bit） */
#define I2C_ADDR_7BIT  0x50

/* 送信データ（0～255） */
static uint8_t tx_buf[256];

int main(void)
{
    const struct device *i2c_dev;
    int ret;
    uint32_t success_count = 0;
    uint32_t error_count = 0;

    printk("=== nRF54L15 I2C Test (TWIM21) ===\n");

    /* 送信データ初期化 */
    for (uint32_t i = 0; i < sizeof(tx_buf); i++) {
        tx_buf[i] = (uint8_t)i;
    }

    /* I2Cデバイス取得 */
    i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
    
    if (!device_is_ready(i2c_dev)) {
        printk("ERROR: I2C device not ready\n");
        return -1;
    }

    printk("I2C device ready: %s\n", i2c_dev->name);

    /* I2C設定を表示 */
    uint32_t i2c_cfg;
    ret = i2c_get_config(i2c_dev, &i2c_cfg);
    if (ret == 0) {
        uint32_t speed = I2C_SPEED_GET(i2c_cfg);
        const char *speed_str;
        switch (speed) {
            case I2C_SPEED_STANDARD:   speed_str = "100kHz"; break;
            case I2C_SPEED_FAST:       speed_str = "400kHz"; break;
            case I2C_SPEED_FAST_PLUS:  speed_str = "1MHz"; break;
            default:                   speed_str = "unknown"; break;
        }
        printk("I2C speed: %s\n", speed_str);
    }


    printk("Starting I2C transmission to addr 0x%02X...\n", I2C_ADDR_7BIT);

    /* 連続送信 */
    while (1) {


        for(int i=0;i!=sizeof(tx_buf);i++){
            ret = i2c_write(i2c_dev, &tx_buf[i], 1, I2C_ADDR_7BIT);
            
            if (ret == 0) {
                success_count++;
                if (success_count % 100 == 0) {
                    printk("OK: %u, ERR: %u\n", success_count, error_count);
                }
            } else {
                error_count++;
                if (error_count % 10 == 0) {
                    printk("I2C error: %d (total: %u)\n", ret, error_count);
                }
                //k_msleep(10);
            }
            
        }
        
        /*
        
        ret = i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), I2C_ADDR_7BIT);

        if (ret == 0) {
            success_count++;
            if (success_count % 100 == 0) {
                printk("OK: %u, ERR: %u\n", success_count, error_count);
            }
        } else {
            error_count++;
            if (error_count % 10 == 0) {
                printk("I2C error: %d (total: %u)\n", ret, error_count);
            }
            k_msleep(10);
        }
            */
        
        k_yield();
    }

    return 0;
}
