#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_bitbang, LOG_LEVEL_INF);

/* ========================================
 * ユーザー設定エリア
 * ======================================== */

/* I2Cピン設定 */
#define I2C_SCL_PORT    1      // P1.11のポート番号
#define I2C_SCL_PIN     11     // P1.11のピン番号
#define I2C_SDA_PORT    1      // P1.10のポート番号
#define I2C_SDA_PIN     10     // P1.10のピン番号

/* I2Cスレーブアドレス（7ビット） */
#define I2C_SLAVE_ADDR      0x50  // お使いのデバイスに合わせて変更

/* I2Cクロック設定 */
#define I2C_DELAY_US        1.25     // 5us = 約100kHz (調整可能)

/* 送信データ設定 */
#define SEND_DATA_MIN       0
#define SEND_DATA_MAX       255
#define SEND_INTERVAL_MS    10

/* ========================================
 * GPIO設定
 * ======================================== */

static const struct device *scl_dev;
static const struct device *sda_dev;

/* ========================================
 * ソフトウェアI2C実装
 * ======================================== */

/* 遅延関数 */
static inline void i2c_delay(void) {
    k_busy_wait(I2C_DELAY_US);
}

/* SDA設定 */
static void sda_high(void) {
    gpio_pin_configure(sda_dev, I2C_SDA_PIN, GPIO_INPUT | GPIO_PULL_UP);
    i2c_delay();
}

static void sda_low(void) {
    gpio_pin_configure(sda_dev, I2C_SDA_PIN, GPIO_OUTPUT_LOW);
    i2c_delay();
}

static int sda_read(void) {
    int val;
    gpio_pin_configure(sda_dev, I2C_SDA_PIN, GPIO_INPUT | GPIO_PULL_UP);
    val = gpio_pin_get(sda_dev, I2C_SDA_PIN);
    return val;
}

/* SCL設定 */
static void scl_high(void) {
    gpio_pin_configure(scl_dev, I2C_SCL_PIN, GPIO_INPUT | GPIO_PULL_UP);
    i2c_delay();
    
    /* クロックストレッチング対応（オプション） */
    int timeout = 100;
    while (gpio_pin_get(scl_dev, I2C_SCL_PIN) == 0 && timeout-- > 0) {
        k_busy_wait(1);
    }
}

static void scl_low(void) {
    gpio_pin_configure(scl_dev, I2C_SCL_PIN, GPIO_OUTPUT_LOW);
    i2c_delay();
}

/* I2Cスタートコンディション */
static void i2c_start(void) {
    sda_high();
    scl_high();
    sda_low();
    scl_low();
}

/* I2Cストップコンディション */
static void i2c_stop(void) {
    sda_low();
    scl_high();
    sda_high();
}

/* 1バイト書き込み */
static int i2c_write_byte(uint8_t byte) {
    int i;
    int ack;

    /* 8ビット送信（MSBファースト） */
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sda_high();
        } else {
            sda_low();
        }
        scl_high();
        scl_low();
        byte <<= 1;
    }

    /* ACK読み取り */
    sda_high();
    scl_high();
    ack = sda_read();
    scl_low();

    return (ack == 0) ? 0 : -1;  // ACK=0なら成功
}

/* 1バイト読み込み */
static uint8_t i2c_read_byte(int send_ack) {
    int i;
    uint8_t byte = 0;

    sda_high();

    /* 8ビット受信（MSBファースト） */
    for (i = 0; i < 8; i++) {
        scl_high();
        byte <<= 1;
        if (sda_read()) {
            byte |= 0x01;
        }
        scl_low();
    }

    /* ACK/NACK送信 */
    if (send_ack) {
        sda_low();   // ACK
    } else {
        sda_high();  // NACK
    }
    scl_high();
    scl_low();
    sda_high();

    return byte;
}

/* ========================================
 * I2C通信関数
 * ======================================== */

/**
 * @brief I2Cデータ送信
 */
static int i2c_send_data(uint8_t addr, uint8_t *data, uint8_t len) {
    int ret;
    int i;

    i2c_start();

    /* スレーブアドレス + Write bit (0) */
    ret = i2c_write_byte((addr << 1) | 0x00);
    if (ret != 0) {
        LOG_ERR("Slave address NACK");
        i2c_stop();
        return -1;
    }

    /* データ送信 */
    for (i = 0; i < len; i++) {
        ret = i2c_write_byte(data[i]);
        if (ret != 0) {
            LOG_ERR("Data byte %d NACK", i);
            i2c_stop();
            return -1;
        }
    }

    i2c_stop();
    return 0;
}

/**
 * @brief I2Cデータ受信
 */
static int i2c_receive_data(uint8_t addr, uint8_t *data, uint8_t len) {
    int ret;
    int i;

    i2c_start();

    /* スレーブアドレス + Read bit (1) */
    ret = i2c_write_byte((addr << 1) | 0x01);
    if (ret != 0) {
        LOG_ERR("Slave address NACK (read)");
        i2c_stop();
        return -1;
    }

    /* データ受信 */
    for (i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i < (len - 1));  // 最後以外はACK
    }

    i2c_stop();
    return 0;
}

/**
 * @brief レジスタ書き込み
 */
static int i2c_write_register(uint8_t addr, uint8_t reg_addr, uint8_t value) {
    uint8_t data[2];
    data[0] = reg_addr;
    data[1] = value;
    
    return i2c_send_data(addr, data, 2);
}

/**
 * @brief レジスタ読み込み
 */
static int i2c_read_register(uint8_t addr, uint8_t reg_addr, uint8_t *value) {
    int ret;

    /* レジスタアドレス送信 */
    ret = i2c_send_data(addr, &reg_addr, 1);
    if (ret != 0) {
        return ret;
    }

    /* データ受信 */
    return i2c_receive_data(addr, value, 1);
}

/**
 * @brief GPIOデバイス取得
 */
static const struct device* get_gpio_device(int port) {
    switch (port) {
        case 0:
            return DEVICE_DT_GET(DT_NODELABEL(gpio0));
        case 1:
            return DEVICE_DT_GET(DT_NODELABEL(gpio1));
        case 2:
            return DEVICE_DT_GET(DT_NODELABEL(gpio2));
        default:
            return NULL;
    }
}

/**
 * @brief I2C初期化
 */
static int i2c_init(void) {
    /* SCL用GPIOデバイス取得 */
    scl_dev = get_gpio_device(I2C_SCL_PORT);
    if (scl_dev == NULL || !device_is_ready(scl_dev)) {
        LOG_ERR("SCL GPIO device (port %d) not ready", I2C_SCL_PORT);
        return -1;
    }

    /* SDA用GPIOデバイス取得 */
    sda_dev = get_gpio_device(I2C_SDA_PORT);
    if (sda_dev == NULL || !device_is_ready(sda_dev)) {
        LOG_ERR("SDA GPIO device (port %d) not ready", I2C_SDA_PORT);
        return -1;
    }

    LOG_INF("GPIO devices ready");
    LOG_INF("SCL: P%d.%d, SDA: P%d.%d", 
            I2C_SCL_PORT, I2C_SCL_PIN, I2C_SDA_PORT, I2C_SDA_PIN);

    /* ピンをアイドル状態に設定 */
    sda_high();
    scl_high();

    LOG_INF("Software I2C initialized");
    
    return 0;
}

/* ========================================
 * メイン関数
 * ======================================== */

int main(void) {
    int ret;
    uint8_t data = SEND_DATA_MIN;
    uint8_t tx_data[1];

    LOG_INF("========================================");
    LOG_INF("nRF54L15-DK Software I2C Master");
    LOG_INF("========================================");
    LOG_INF("SCL: P%d.%d, SDA: P%d.%d", 
            I2C_SCL_PORT, I2C_SCL_PIN, I2C_SDA_PORT, I2C_SDA_PIN);
    LOG_INF("Slave Address: 0x%02X", I2C_SLAVE_ADDR);
    LOG_INF("Clock: ~%d kHz", 1000 / (I2C_DELAY_US * 2));
    LOG_INF("========================================");

    /* I2C初期化 */
    ret = i2c_init();
    if (ret != 0) {
        LOG_ERR("I2C initialization failed!");
        return -1;
    }

    /* 初期化後の待機 */
    k_msleep(100);

    /* メインループ */
    while (1) {
        tx_data[0] = data;

        /* データ送信 */
        ret = i2c_send_data(I2C_SLAVE_ADDR, tx_data, 1);
        
        if (ret == 0) {
            LOG_INF("Sent: 0x%02X (%d) to address 0x%02X", 
                    data, data, I2C_SLAVE_ADDR);
        } else {
            LOG_ERR("Send failed for data: 0x%02X", data);
        }

        /* レジスタ書き込み例（必要に応じて使用） */
        // ret = i2c_write_register(I2C_SLAVE_ADDR, 0x00, data);
        // if (ret == 0) {
        //     LOG_INF("Register write OK: reg=0x00, data=0x%02X", data);
        // }

        /* レジスタ読み込み例（必要に応じて使用） */
        // uint8_t rx_data;
        // ret = i2c_read_register(I2C_SLAVE_ADDR, 0x00, &rx_data);
        // if (ret == 0) {
        //     LOG_INF("Read: 0x%02X", rx_data);
        // }

        /* データインクリメント */
        data++;
        if (data > SEND_DATA_MAX) {
            data = SEND_DATA_MIN;
            LOG_INF("Counter reset to %d", SEND_DATA_MIN);
        }

        k_msleep(SEND_INTERVAL_MS);
    }

    return 0;
}
