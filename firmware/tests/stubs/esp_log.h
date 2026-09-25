#pragma once

inline void test_esp_log(const char*, const char*, ...) {}
#define ESP_LOGI(...) test_esp_log(__VA_ARGS__)
#define ESP_LOGW(...) test_esp_log(__VA_ARGS__)
#define ESP_LOGE(...) test_esp_log(__VA_ARGS__)
