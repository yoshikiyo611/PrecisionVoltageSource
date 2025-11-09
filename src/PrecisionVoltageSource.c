//*****************************************************************************
// ファイル名:       PrecisionVoltageSource.c
// 対象マイコン:     RP2040 (Raspberry Pi Pico)
// 説明:            Precision Voltage Source
// 制作者:          yoshikiyo
// 最終更新日:      2025年3月13日
//*****************************************************************************

//=============================================================================
// インクルードファイル
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/float.h"
#include "pico/flash.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"

//=============================================================================
// 定数定義
//=============================================================================
#define SPI_PORT            spi0
#define PIN_SPI_SCK         2
#define PIN_SPI_MOSI        3
#define PIN_SPI_CS          5

#define AD5791_REG_DAC      0x01
#define AD5791_REG_CTRL     0x02
#define AD5791_REG_SCTRL    0x04
#define AD5791_SCTRL_RESET  0x04

#define DAC_RESOLUTION      1048575        // DAC分解能
#define DAC_VREF            13.799         // DAC基準電圧
#define DAC_RES_PER_VOLT    (DAC_RESOLUTION / DAC_VREF) // 1Vあたりの分解能

#define VOLTAGE_MIN         -12.0          // 最小電圧
#define VOLTAGE_MAX         12.0           // 最大電圧

#define PIN_RELAY           1              // リレーピン
#define PIN_LED_RED         27             // 赤LEDピン
#define PIN_LED_BLUE        28             // 青LEDピン
#define PIN_LED_ONBOARD     25             // オンボードLEDピン
#define PIN_BUZZER          22             // ブザーピン

#define BUFFER_SIZE         64             // コマンドバッファサイズ

// デフォルトのキャリブレーション値（10Vレンジ：リレーオフ、20Vレンジ：リレーオン）
#define DEFAULT_SLOPE_10V_RANGE     -0.5
#define DEFAULT_INTERCEPT_10V_RANGE  0.0
#define DEFAULT_SLOPE_20V_RANGE     -0.25
#define DEFAULT_INTERCEPT_20V_RANGE  0.0

// フラッシュメモリ関連定数
#define FLASH_TARGET_OFFSET (1536 * 1024) // フラッシュの保存位置 (1.5MBオフセット)
#define FLASH_SECTOR_SIZE   (1u << 12)    // 4KBセクター
#define FLASH_DATA_SIZE     (sizeof(double) * 4) // 4つのdouble値

// ブザー関連定数
#define BUZZER_FREQ         4000    // ブザー周波数 (4kHz)
#define BUZZER_WRAP         31249   // PWMラップ値 (125MHz / 4kHz - 1)
#define BUZZER_DUTY_50      15625   // 50%デューティ比

//=============================================================================
// グローバル変数
//=============================================================================
static double set_voltage = 0.0;           // 設定電圧
static double set_voltage_cal = 0.0;       // キャリブレーション後の電圧
static char cmd_buffer[BUFFER_SIZE];       // コマンドバッファ
static uint32_t buffer_idx = 0;            // バッファインデックス

// キャリブレーション値（フラッシュから読み書き）
static double slope_10v_range;       // 10Vレンジ（リレーオフ時）の傾き
static double intercept_10v_range;   // 10Vレンジ（リレーオフ時）の切片
static double slope_20v_range;       // 20Vレンジ（リレーオン時）の傾き
static double intercept_20v_range;   // 20Vレンジ（リレーオン時）の切片

// ブザー変数
static uint32_t beep_timer = 0;     // ブザータイマー
static uint16_t beep_pattern = 0x0000; // ブザーパターン
static int8_t beep_mode = 0;        // ブザーモード
static uint pwm_slice_num;          // PWMスライス番号

// フラッシュに保存するデフォルトデータ構造
static const double flash_default_data[4] = {
    DEFAULT_SLOPE_10V_RANGE,
    DEFAULT_INTERCEPT_10V_RANGE,
    DEFAULT_SLOPE_20V_RANGE,
    DEFAULT_INTERCEPT_20V_RANGE
};

//=============================================================================
// 関数プロトタイプ宣言
//=============================================================================
static bool timer_callback(repeating_timer_t *rt);
static void init_rp2040(void);
static void core1_main(void);
static void ad5791_write(uint8_t reg, uint32_t data);
static void ad5791_set_dac(int32_t value);
static void ad5791_init(void);
static void process_command(const char *cmd);
static void update_voltage(double voltage);
static void handle_usb_input(void);
static void init_pwm(void);
void init_beep(void);
void beep_out(uint16_t f);
void set_beep_pattern(uint16_t data);
void beep_process(void);
static void load_calibration_from_flash(void);
static void save_calibration_to_flash(void);
static void initialize_flash(void);

//=============================================================================
// コマンドツリー構造体
//=============================================================================
typedef struct {
    const char *command;            // コマンド名
    void (*function)(const char *arg); // コマンドハンドラ
    const char *description;        // 説明
} CommandNode;

static void cmd_set_voltage(const char *arg);
static void cmd_help(const char *arg);
static void cmd_reset_calibration(const char *arg);
static void cmd_apply_calibration(const char *arg);
static void cmd_idn(const char *arg);
static void cmd_set_calibration(const char *arg);

static const CommandNode command_tree[] = {
    {"volt", cmd_set_voltage, "Set voltage (e.g., 'volt 5.0')"},
    {"help?", cmd_help, "Show available commands"},
    {"resetcal", cmd_reset_calibration, "Reset calibration to default values"},
    {"applycal", cmd_apply_calibration, "Apply calibrated values"},
    {"*idn?", cmd_idn, "Return device identification"},
    {"setcal", cmd_set_calibration, "Set calibration from voltages (e.g., 'setcal 10.1 -9.9 12.2 -11.8')"},
    {NULL, NULL, NULL}
};

//=============================================================================
// メイン関数 (Core0)
//=============================================================================
int main(void) {
    init_rp2040();              // RP2040を初期化
    set_beep_pattern(0x0A);     // 初期ビープパターン設定
    sleep_ms(1000);             // 1秒待機

    while (true) {
        handle_usb_input();     // USB入力を処理
    }
    return 0;
}

//=============================================================================
// Core1メイン (将来の拡張用)
//=============================================================================
static void core1_main(void) {
    while (true) {
        sleep_ms(1000);         // 1秒ごとに待機
    }
}

//=============================================================================
// RP2040初期化
//=============================================================================
static void init_rp2040(void) {
    stdio_usb_init();           // USBシリアルを初期化
    multicore_launch_core1(core1_main); // Core1を起動

    // GPIOピンを初期化
    const uint8_t output_pins[] = {PIN_RELAY, PIN_LED_RED, PIN_LED_BLUE, PIN_LED_ONBOARD};
    for (size_t i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
        gpio_init(output_pins[i]);
        gpio_set_dir(output_pins[i], GPIO_OUT);
        gpio_put(output_pins[i], 0);
    }

    // 1ms周期のタイマーを設定
    static repeating_timer_t timer;
    add_repeating_timer_ms(-1, timer_callback, NULL, &timer);

    // 周辺機器を初期化
    ad5791_init();              // AD5791 DACを初期化
    init_beep();                // ブザーを初期化
    initialize_flash();         // フラッシュを初期化（初回起動時にデフォルト値を保存）
    load_calibration_from_flash(); // フラッシュからキャリブレーション値を読み込み
}

//=============================================================================
// タイマーコールバック (1ms間隔)
//=============================================================================
static bool timer_callback(repeating_timer_t *rt) {
    beep_process();             // ブザーパターンを処理
    return true;                // タイマーを継続
}

//=============================================================================
// USB入力処理
//=============================================================================
static void handle_usb_input(void) {
    int c = getchar_timeout_us(0); // 非ブロッキングで文字を取得
    if (c != PICO_ERROR_TIMEOUT) {
        if (buffer_idx < BUFFER_SIZE - 1) {
            cmd_buffer[buffer_idx++] = (char)c;
        }
        // CRLF (\r\n) を検出
        if (buffer_idx >= 2 && cmd_buffer[buffer_idx - 2] == '\r' && cmd_buffer[buffer_idx - 1] == '\n') {
            cmd_buffer[buffer_idx - 2] = '\0';
            process_command(cmd_buffer); // コマンドを処理
            buffer_idx = 0;
            memset(cmd_buffer, 0, BUFFER_SIZE);
        }
    }
}

//=============================================================================
// コマンド処理
//=============================================================================
static void process_command(const char *cmd) {
    char command[16];
    char arg[64]; // バッファサイズを64に増量
    
    if (sscanf(cmd, "%15s %63[^\r\n]", command, arg) >= 1) {
        bool valid_command = false;
        for (int i = 0; command_tree[i].command != NULL; i++) {
            if (strcasecmp(command, command_tree[i].command) == 0) {
                command_tree[i].function(arg);
                valid_command = true;
                gpio_put(PIN_LED_BLUE, 0); // 有効なコマンドで青LED点灯
                gpio_put(PIN_LED_RED, 1);  // 赤LED消灯
                break;
            }
        }
        if (!valid_command) {
            printf("Unknown command: %s\r\n", command);
            gpio_put(PIN_LED_RED, 0);  // 無効なコマンドで赤LED点灯
            gpio_put(PIN_LED_BLUE, 1); // 青LED消灯
        }
    }
}

//=============================================================================
// コマンドハンドラ
//=============================================================================
static void cmd_set_voltage(const char *arg) {
    double voltage;
    if (sscanf(arg, "%lf", &voltage) == 1) {
        update_voltage(voltage);    // 電圧を更新
        printf("Voltage set to %.4fV\r\n", voltage);
    } else {
        printf("Invalid voltage value\r\n");
    }
}

static void cmd_help(const char *arg) {
    printf("Available commands:\r\n");
    for (int i = 0; command_tree[i].command != NULL; i++) {
        printf("  %s - %s\r\n", command_tree[i].command, command_tree[i].description);
    }
}

static void cmd_reset_calibration(const char *arg) {
    slope_10v_range = DEFAULT_SLOPE_10V_RANGE;
    intercept_10v_range = DEFAULT_INTERCEPT_10V_RANGE;
    slope_20v_range = DEFAULT_SLOPE_20V_RANGE;
    intercept_20v_range = DEFAULT_INTERCEPT_20V_RANGE;
    update_voltage(set_voltage);
    printf("Calibration reset to default values\r\n");
}

static void cmd_apply_calibration(const char *arg) {
    load_calibration_from_flash(); // フラッシュからキャリブレーション値を再読み込み
    update_voltage(set_voltage);
    printf("Calibrated values applied\r\n");
}

static void cmd_idn(const char *arg) {
    printf("Precision Voltage Source\r\n");
}

static void cmd_set_calibration(const char *arg) {
    double v_10v_plus, v_10v_minus, v_20v_plus, v_20v_minus;

    // 引数が空の場合はエラーメッセージを表示
    if (!arg || arg[0] == '\0') {
        printf("No arguments provided. Use: setcal <10V_plus> <10V_minus> <20V_plus> <20V_minus>\r\n");
        return;
    }

    // 測定した電圧値を入力
    if (sscanf(arg, "%lf %lf %lf %lf", &v_10v_plus, &v_10v_minus, &v_20v_plus, &v_20v_minus) == 4) {
        // 10Vレンジの傾きと切片を計算
        // 理論値: プラスフルスケール -5.0V、マイナスフルスケール 5.0V
        // 測定値を x、理論値を y とする
        double x1_10v = v_10v_plus;   // 測定値（プラスフルスケール）
        double x2_10v = v_10v_minus;  // 測定値（マイナスフルスケール）
        double y1_10v = -5.0;         // 理論値（プラスフルスケール）
        double y2_10v = 5.0;          // 理論値（マイナスフルスケール）

        slope_10v_range = (y2_10v - y1_10v) / (x2_10v - x1_10v); // 傾き m = (y2 - y1) / (x2 - x1)
        intercept_10v_range = y1_10v - slope_10v_range * x1_10v; // 切片 b = y1 - m * x1

        // 20Vレンジの傾きと切片を計算
        // 理論値: プラスフルスケール -5.0V、マイナスフルスケール 5.0V
        // 測定値を x、理論値を y とする
        double x1_20v = v_20v_plus;   // 測定値（プラスフルスケール）
        double x2_20v = v_20v_minus;  // 測定値（マイナスフルスケール）
        double y1_20v = -5.0;         // 理論値（プラスフルスケール）
        double y2_20v = 5.0;          // 理論値（マイナスフルスケール）

        slope_20v_range = (y2_20v - y1_20v) / (x2_20v - x1_20v); // 傾き m = (y2 - y1) / (x2 - x1)
        intercept_20v_range = y1_20v - slope_20v_range * x1_20v; // 切片 b = y1 - m * x1

        // フラッシュに保存
        save_calibration_to_flash();

        // 現在の設定電圧に適用
        update_voltage(set_voltage);

        // 計算結果を表示
        printf("Calibration set:\r\n");
        printf("  10V Range - Slope: %.9f, Intercept: %.9f\r\n", slope_10v_range, intercept_10v_range);
        printf("  20V Range - Slope: %.9f, Intercept: %.9f\r\n", slope_20v_range, intercept_20v_range);
    } else {
        printf("Invalid values. Use: setcal <10V_plus> <10V_minus> <20V_plus> <20V_minus>\r\n");
    }
}

//=============================================================================
// フラッシュ操作
//=============================================================================
// フラッシュを初期化（初回起動時にデフォルト値を保存）
static void initialize_flash(void) {
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    
    // フラッシュが不定値か確認（簡易的に最初のバイトをチェック）
    if (flash_data[0] == 0xFF) { // 消去状態（0xFF）の場合、デフォルト値を保存
        slope_10v_range = DEFAULT_SLOPE_10V_RANGE;
        intercept_10v_range = DEFAULT_INTERCEPT_10V_RANGE;
        slope_20v_range = DEFAULT_SLOPE_20V_RANGE;
        intercept_20v_range = DEFAULT_INTERCEPT_20V_RANGE;
        save_calibration_to_flash();
    }
}

// フラッシュからキャリブレーション値を読み込む
static void load_calibration_from_flash(void) {
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    double *data = (double *)flash_data;

    // フラッシュから直接値を読み込む
    slope_10v_range = data[0];
    intercept_10v_range = data[1];
    slope_20v_range = data[2];
    intercept_20v_range = data[3];
}

// フラッシュにキャリブレーション値を保存
static void save_calibration_to_flash(void) {
    double flash_data[4] = {
        slope_10v_range,
        intercept_10v_range,
        slope_20v_range,
        intercept_20v_range
    };

    // 割り込みを無効化してフラッシュを安全に書き込み
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)flash_data, FLASH_DATA_SIZE);
    restore_interrupts(ints);

    // 書き込み後に簡易的な整合性チェック
    const uint8_t *flash_data_check = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    if (memcmp(flash_data, flash_data_check, FLASH_DATA_SIZE) != 0) {
        // 書き込み失敗時のエラー表示（必要に応じて）
        printf("Flash write verification failed\r\n");
    }
}

//=============================================================================
// AD5791 DAC制御
//=============================================================================
static void ad5791_init(void) {
    spi_init(SPI_PORT, 1000 * 1000); // 1MHz SPI初期化
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1);
    sleep_ms(100);
    ad5791_write(AD5791_REG_SCTRL, AD5791_SCTRL_RESET); // AD5791リセット
    sleep_ms(1);
    ad5791_write(AD5791_REG_CTRL, 0xC0); // 制御レジスタ設定
    ad5791_set_dac((int32_t)(-0.000237198 * DAC_RES_PER_VOLT)); // 初期DAC値
}

static void ad5791_write(uint8_t reg, uint32_t data) {
    uint8_t tx_buf[3];
    tx_buf[0] = (reg & 0x0F) << 4 | ((data >> 16) & 0x0F);
    tx_buf[1] = (data >> 8) & 0xFF;
    tx_buf[2] = data & 0xFF;
    gpio_put(PIN_SPI_CS, 0);
    spi_write_blocking(SPI_PORT, tx_buf, 3);
    gpio_put(PIN_SPI_CS, 1);
}

static void ad5791_set_dac(int32_t value) {
    ad5791_write(AD5791_REG_DAC, (uint32_t)value);
}

//=============================================================================
// 電圧更新
//=============================================================================
static void update_voltage(double voltage) {
    set_voltage = voltage;
    if (set_voltage >= VOLTAGE_MIN && set_voltage <= VOLTAGE_MAX) {
        gpio_put(PIN_RELAY, 0); // リレーオフ（10Vレンジ）
        set_voltage_cal = (set_voltage * slope_10v_range) + intercept_10v_range;
        ad5791_set_dac((int32_t)(set_voltage_cal * DAC_RES_PER_VOLT));
    } else {
        gpio_put(PIN_RELAY, 1); // リレーオン（20Vレンジ）
        set_voltage_cal = (set_voltage * slope_20v_range) + intercept_20v_range;
        ad5791_set_dac((int32_t)(set_voltage_cal * DAC_RES_PER_VOLT));
    }
}

//=============================================================================
// ブザー関連
//=============================================================================
static void init_pwm(void) {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM); // ブザーピンをPWMに設定
    pwm_slice_num = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_set_output_polarity(pwm_slice_num, PWM_CHAN_A, true);
    pwm_set_wrap(pwm_slice_num, BUZZER_WRAP); // 4kHz周波数設定
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0); // 初期デューティ0%
    pwm_set_enabled(pwm_slice_num, true); // PWM有効化
}

void init_beep(void) {
    init_pwm();                 // PWM初期化
    beep_timer = 0;             // タイマーリセット
    beep_pattern = 0x0000;      // パターン初期化
    beep_mode = 0;              // モード初期化
    beep_out(0);                // ブザーオフ
}

void beep_out(uint16_t f) {
    if (f) {
        pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, BUZZER_DUTY_50); // 50%デューティ
    } else {
        pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0); // オフ
    }
}

void set_beep_pattern(uint16_t data) {
    if (!beep_mode) {
        beep_pattern = data;    // パターン設定
        beep_timer = 0;         // タイマーリセット
        beep_mode = 1;          // モード開始
    }
}

void beep_process(void) {
    if (beep_mode == 1) {
        if (beep_timer % 50 == 0) {
            if (beep_pattern & 0x8) {
                beep_out(1);    // ブザーオン
            } else {
                beep_out(0);    // ブザーオフ
            }
            beep_pattern <<= 1; // パターンを左シフト
        }
        beep_timer++;
        if (beep_timer / 50 == 5) {
            beep_out(0);        // ブザー停止
            beep_mode = 0;      // モードリセット
        }
    }
}

//*****************************************************************************
// ファイル終わり
//*****************************************************************************