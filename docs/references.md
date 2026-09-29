# Источники

## Первичные данные проекта

- [Захваты UART](../captures/README.md) и [измерения](measurements.md).
- [Основная плата сверху](../photo/IMG_3955.jpeg) и [снизу](../photo/IMG_3956.jpeg).
- [Плата панели](../photo/IMG_3957.jpeg) и [внешний вид](../photo/IMG_3958.jpeg).
- [Заводская инструкция: управление и ошибки](../photo/IMG_4035.jpeg).
- [Заводская инструкция: сон, защита и замок](../photo/IMG_4036.jpeg).
- [Шильдик CL-103B-G](../photo/IMG_4037.jpeg).
- [ESP: лицевая сторона](../photo/YD-ESP32-23_front.jpeg) и [обратная](../photo/YD-ESP32-23_back.jpeg).

## Документация компонентов

- [ESP-IDF 6.1 для ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/index.html).
- [ESP32-S3 datasheet](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf).
- [Производитель YD-ESP32-S3](https://github.com/vcc-gnd/YD-ESP32-S3).
- [Ulanzi D100H](https://www.ulanzi.com/products/d100h-dial-creative-controller-i003).

## История исследования

[FrakesH/Smart-Standing-Desk](https://github.com/FrakesH/Smart-Standing-Desk)
и [dong4j/desk-gateway](https://github.com/dong4j/desk-gateway) использовались
как ориентиры до измерений. Их интерфейсы и коды не являются спецификацией
X2DM-SE. Рабочий протокол этого проекта подтверждён собственными захватами.

Старые планы, аудиты и отчёты о промежуточных рефакторингах доступны в истории
Git до обновления документации. Текущие правила собраны в
[архитектуре](architecture.md), [управлении](ulanzi-d100h.md) и
[плане проверки](verification-plan.md).
