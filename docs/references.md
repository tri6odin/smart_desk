# Источники

## Основные

1. [FrakesH/Smart-Standing-Desk](https://github.com/FrakesH/Smart-Standing-Desk)
   — проект для Volundr X2DM-PRO: ESP32-S3, ESPHome, эмуляция ёмкостных кнопок,
   пассивное чтение ET6226M.
2. [desk.yaml](https://github.com/FrakesH/Smart-Standing-Desk/blob/master/desk.yaml)
   — логика целевой высоты и виртуальных кнопок.
3. [desk_controller.h](https://github.com/FrakesH/Smart-Standing-Desk/blob/master/desk_controller.h)
   — декодирование адресов разрядов и семисегментных кодов.
4. [dong4j/desk-gateway](https://github.com/dong4j/desk-gateway)
   — ESP32-S3 gateway с Mxtark I2C slave, BLE, panel proxy и watchdog.
5. [Описание реверса четырёхпроводной панели](https://sspai.com/post/113563)
   — измерения `3.3V/GND/CLK/DAT`, адреса, коды клавиш и тайминги.
6. [Ulanzi D100H, официальная страница](https://www.ulanzi.com/products/d100h-dial-creative-controller-i003)
   — BLE 5.0, standalone-режим, органы управления и характеристики.
7. [Официальная инструкция D100H](https://manuals.plus/m/ec60f205df9b84aaf59fc3323080ebc1ca4ce921e3417d14f5b93a2bdb36ab03.pdf)
   — сопряжение, три профиля, короткие/долгие нажатия и жесты крутилки.

## Документация компонентов

1. [ESP32-S3 Series Datasheet](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf)
   — допустимые напряжения питания и GPIO.
2. [ESP-IDF: GPIO ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)
   — конфигурация GPIO и open-drain.
3. [ESP-IDF: Bluetooth HID Host API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/bluetooth/esp_hidh.html)
   — официальный host API для подключения BLE HID-клавиатур и мышей.

## Связанные материалы

1. [LoctekMotion IoT](https://github.com/iMicknl/LoctekMotion_IoT) — пример
   другого семейства столов с UART. Не считается совместимым с X2DM-SE без
   измерений.
2. [Jarvis desk reverse engineering](https://github.com/phord/Jarvis) — пример
   перехвата интерфейса панели и основной платы. Протокол другой.

## Локальные первичные материалы

- [Основная плата, верх](../photo/IMG_3955.jpeg)
- [Основная плата, низ](../photo/IMG_3956.jpeg)
- [Плата панели](../photo/IMG_3957.jpeg)
- [Лицевая часть панели](../photo/IMG_3958.jpeg)
- [Бумажная инструкция: управление, параметры и ошибки](../photo/IMG_4035.jpeg)
- [Бумажная инструкция: сон, тепловая защита и детский замок](../photo/IMG_4036.jpeg)
- [Шильдик контроллера CL-103B-G](../photo/IMG_4037.jpeg)
- [Конспект штатной инструкции](official-manual.md)

## Лицензии внешних проектов

- FrakesH/Smart-Standing-Desk: GPL-3.0. Прямое заимствование кода создаёт
  обязательства GPL для производной работы.
- dong4j/desk-gateway: MIT.

При реализации нужно либо соблюдать лицензию выбранной кодовой базы, либо
использовать результаты измерений и написать собственную независимую
реализацию протокола.
