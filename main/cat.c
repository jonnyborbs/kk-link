#include "cat.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "radio_usb.h"

static const char *TAG = "cat";

#define LINE_QUEUE_LEN 8

typedef struct {
    char text[CAT_LINE_MAX];
} cat_line_t;

static QueueHandle_t s_lines;
static cat_line_t s_partial; /* only touched from the USB driver task (and cat_reset) */
static size_t s_partial_len;

esp_err_t cat_init(void)
{
    s_lines = xQueueCreate(LINE_QUEUE_LEN, sizeof(cat_line_t));
    return s_lines ? ESP_OK : ESP_ERR_NO_MEM;
}

void cat_feed(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        const char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (s_partial_len == 0) {
                continue; /* ignore empty lines / CRLF pairs */
            }
            s_partial.text[s_partial_len] = '\0';
            if (xQueueSend(s_lines, &s_partial, 0) != pdTRUE) {
                ESP_LOGW(TAG, "line queue full, dropped: %s", s_partial.text);
            }
            s_partial_len = 0;
        } else if (s_partial_len < CAT_LINE_MAX - 1) {
            s_partial.text[s_partial_len++] = c;
        }
        /* else: overlong line, truncate silently until the terminator */
    }
}

void cat_reset(void)
{
    s_partial_len = 0;
    if (s_lines) {
        xQueueReset(s_lines);
    }
}

esp_err_t cat_command(const char *cmd, char *reply, size_t reply_size, uint32_t timeout_ms)
{
    if (!s_lines || !cmd || !reply || reply_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Anything already queued is unsolicited output, not our reply. */
    cat_line_t line;
    while (xQueueReceive(s_lines, &line, 0) == pdTRUE) {
        ESP_LOGI(TAG, "unsolicited: %s", line.text);
    }

    char buf[CAT_LINE_MAX + 1];
    const size_t n = strlen(cmd);
    if (n >= CAT_LINE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buf, cmd, n);
    buf[n] = '\r';

    esp_err_t err = radio_usb_write((const uint8_t *)buf, n + 1, timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write '%s' failed: %s", cmd, esp_err_to_name(err));
        return err;
    }
    if (xQueueReceive(s_lines, &line, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "no reply to '%s' within %lu ms", cmd, (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    strlcpy(reply, line.text, reply_size);
    return ESP_OK;
}
