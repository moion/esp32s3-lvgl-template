
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_dma_utils.h"
#include "lvgl.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch_gt911.h"
#include <driver/i2c.h>
#include <lv_demos.h>


static const char *TAG = "LVGL";

//LCD
#define LCD_PIXEL_CLOCK_HZ          (30 * 1000 * 1000)
#define CONFIG_LCD_I80_BUS_WIDTH    8
#define CONFIG_LCD_TOUCH_ENABLED    1
#define LCD_BK_LIGHT_ON_LEVEL       1
#define LCD_BK_LIGHT_OFF_LEVEL      0
#define LCD_PIN_NUM_DATA0           6
#define LCD_PIN_NUM_DATA1           7
#define LCD_PIN_NUM_DATA2           8
#define LCD_PIN_NUM_DATA3           9
#define LCD_PIN_NUM_DATA4           10
#define LCD_PIN_NUM_DATA5           11
#define LCD_PIN_NUM_DATA6           12
#define LCD_PIN_NUM_DATA7           13
#if CONFIG_LCD_I80_BUS_WIDTH    >   8
#define EXAMPLE_PIN_NUM_DATA8          14
#define EXAMPLE_PIN_NUM_DATA9          15
#define EXAMPLE_PIN_NUM_DATA10         16
#define EXAMPLE_PIN_NUM_DATA11         17
#define EXAMPLE_PIN_NUM_DATA12         18
#define EXAMPLE_PIN_NUM_DATA13         19
#define EXAMPLE_PIN_NUM_DATA14         20
#define EXAMPLE_PIN_NUM_DATA15         21
#endif
#define LCD_PIN_NUM_PCLK           5
#define LCD_PIN_NUM_CS             3
#define LCD_PIN_NUM_DC             4
#define LCD_PIN_NUM_RST            2
#define LCD_PIN_NUM_BK_LIGHT       1

#define LCD_ROATATION              1

#if LCD_ROATATION == 1 | LCD_ROATATION == 3
    #define LCD_H_RES                  480
    #define LCD_V_RES                  320
#endif
#if LCD_ROATATION == 0 | LCD_ROATATION == 2
    #define LCD_H_RES                  320
    #define LCD_V_RES                  480
#endif

#define LCD_CMD_BITS                8
#define LCD_PARAM_BITS              8

// I2C number
#define TOUCH_I2C_NUM              0   
#define TOUCH_I2C_SCL              39
#define TOUCH_I2C_SDA              40

#define LVGL_TICK_PERIOD_MS         2
#define LVGL_TASK_MAX_DELAY_MS      500
#define LVGL_TASK_MIN_DELAY_MS      1
#define LVGL_TASK_STACK_SIZE        (10 * 1024)
#define LVGL_TASK_PRIORITY          2

#define PSRAM_DATA_ALIGNMENT        64

static SemaphoreHandle_t lvgl_mux = NULL;

extern void lvgl_demo_ui(lv_disp_t *disp);
extern lv_font_t song_26;
extern lv_font_t song_26_4b;
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

#if CONFIG_LCD_TOUCH_ENABLED
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        #if LCD_ROATATION == 0
            data->point.x = touchpad_x[0];
            data->point.y = touchpad_y[0];
            data->state = LV_INDEV_STATE_PRESSED;
        #endif

        #if LCD_ROATATION == 1
            data->point.x = 480 - touchpad_y[0];
            data->point.y = touchpad_x[0];
            data->state = LV_INDEV_STATE_PRESSED;
        #endif

        #if LCD_ROATATION == 2
            data->point.x = 320 - touchpad_x[0];
            data->point.y = 480 - touchpad_y[0];
            data->state = LV_INDEV_STATE_PRESSED;
        #endif

        #if LCD_ROATATION == 3
            data->point.x = touchpad_y[0];
            data->point.y = 320 - touchpad_x[0];
            data->state = LV_INDEV_STATE_PRESSED;
        #endif

    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void example_init_i80_bus(esp_lcd_panel_io_handle_t *io_handle, void *user_ctx)
{
    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .wr_gpio_num = LCD_PIN_NUM_PCLK,
        .data_gpio_nums = {
            LCD_PIN_NUM_DATA0,
            LCD_PIN_NUM_DATA1,
            LCD_PIN_NUM_DATA2,
            LCD_PIN_NUM_DATA3,
            LCD_PIN_NUM_DATA4,
            LCD_PIN_NUM_DATA5,
            LCD_PIN_NUM_DATA6,
            LCD_PIN_NUM_DATA7,
#if CONFIG_LCD_I80_BUS_WIDTH > 8
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
#endif
        },
        .bus_width = CONFIG_LCD_I80_BUS_WIDTH,
        .max_transfer_bytes = 500 * 100 * sizeof(uint16_t),
        .psram_trans_align = PSRAM_DATA_ALIGNMENT,
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            
            .swap_color_bytes = 0, 
            .pclk_idle_low = 0,
        },
        
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = user_ctx,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, io_handle));
}

void example_init_lcd_panel(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t *panel)
{


    ESP_LOGI(TAG, "Install ST7796 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,      
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle, LCD_ROATATION));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    *panel = panel_handle;

}

#if CONFIG_LCD_TOUCH_ENABLED
void example_init_lcd_touch(esp_lcd_touch_handle_t *tp_handle)
{
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = 400000,
    };

    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");

    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 480,
        .y_max = 320,
        .rst_gpio_num = -1, //42
        .int_gpio_num = -1, //41
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    *tp_handle = tp;
}
#endif // CONFIG_LCD_TOUCH_ENABLED



void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    lv_obj_t * label1 = lv_label_create(scr);
    lv_label_set_long_mode(label1, LV_LABEL_LONG_WRAP);     /*Break the long lines*/
    lv_label_set_recolor(label1, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center "
                      "and wrap long text automatically.");
    //lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_style_text_font(label1,&song_26, 0);
    
    lv_obj_t * label2 = lv_label_create(scr);
    lv_label_set_long_mode(label2, LV_LABEL_LONG_WRAP);     /*Break the long lines*/
    lv_label_set_recolor(label2, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_text(label2, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label, align the lines to the center "
                      "and wrap long text automatically.");
    //lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    lv_obj_set_style_text_align(label2, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_text_font(label2,&song_26_4b, 0);
    
    //start_animation(scr);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions


#if LCD_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
#endif // LCD_PIN_NUM_BK_LIGHT >= 0


    esp_lcd_panel_io_handle_t io_handle = NULL;
    example_init_i80_bus(&io_handle, &disp_drv);
    esp_lcd_panel_handle_t panel_handle = NULL;
    example_init_lcd_panel(io_handle, &panel_handle);

#if CONFIG_LCD_TOUCH_ENABLED
    esp_lcd_touch_handle_t tp_handle = NULL;
    example_init_lcd_touch(&tp_handle);
#endif // CONFIG_LCD_TOUCH_ENABLED


    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if LCD_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif // LCD_PIN_NUM_BK_LIGHT >= 0

    lv_init();
    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;
    uint32_t malloc_flags = 0;
#if CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
    malloc_flags |= ESP_DMA_MALLOC_FLAG_PSRAM;
#endif // CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
    //malloc_flags |= ESP_DMA_MALLOC_FLAG_PSRAM;
    ESP_ERROR_CHECK(esp_dma_malloc(500 * 100 * sizeof(lv_color_t), malloc_flags, (void *)&buf1, NULL));
    ESP_ERROR_CHECK(esp_dma_malloc(500 * 100 * sizeof(lv_color_t), malloc_flags, (void *)&buf2, NULL));
    //buf1 = (lv_color_t*) heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    //buf2 = (lv_color_t*) heap_caps_malloc(LCD_H_RES * LCD_V_RES* sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 500 * 100);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.rotated = 0;
    disp_drv.antialiasing = 1;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

#if CONFIG_LCD_TOUCH_ENABLED
    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp_handle;
    lv_indev_drv_register(&indev_drv);
#endif // CONFIG_LCD_TOUCH_ENABLED

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL animation");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        example_lvgl_demo_ui(disp);
        //lv_demo_widgets();
        // Release the mutex
        example_lvgl_unlock();
    }
}
