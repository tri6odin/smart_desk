# Источники

## Данные проекта

[Захваты UART](../captures/README.md) и [история измерений](measurements.md) подтверждают поведение исследованного экземпляра.
Для вывода о протоколе используйте полный кадр и условия опыта.

| Материал | Источник |
|---|---|
| Основная плата | [Сверху](../photo/IMG_3955.jpeg), [снизу](../photo/IMG_3956.jpeg) |
| Штатная панель | [Плата](../photo/IMG_3957.jpeg), [корпус](../photo/IMG_3958.jpeg) |
| Инструкция производителя | [Управление и ошибки](../photo/IMG_4035.jpeg), [сон и защита](../photo/IMG_4036.jpeg) |
| Шильдик | [CL-103B-G](../photo/IMG_4037.jpeg) |
| ESP прототипа | [Лицевая сторона](../photo/YD-ESP32-23_front.jpeg), [обратная сторона](../photo/YD-ESP32-23_back.jpeg) |

Конспект инструкции не заменяет исходные фотографии.
Фотографии документа производителя не являются оригинальной разработкой этого проекта.

## Компоненты

- [ESP-IDF 6.1 для ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/index.html).
- [ESP32-S3 datasheet](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf).
- [YD-ESP32-S3: материалы производителя](https://github.com/vcc-gnd/YD-ESP32-S3).
- [Ulanzi D100H: страница производителя](https://www.ulanzi.com/products/d100h-dial-creative-controller-i003).

Сверяйте сведения с конкретной платой и версией ESP-IDF.
Товарная надпись на фотографии не подтверждает комплектацию flash или PSRAM.

## Стиль документации

- [ASD-STE100: FAQ](https://www.asd-ste100.org/STE_faq.html): управляемый английский язык.
- [ASD-STE100, issue 9](https://www.asd-ste100.org/assets/files/ASD-STE100_ISSUE9.pdf): правила языка и словарь.
- [Google: процедуры](https://developers.google.com/style/procedures): шаги, действия и структура инструкции.
- [Google: технический стиль](https://developers.google.com/style): ясный язык и постоянные термины.

Русские тексты проекта применяют принципы краткости и однозначности.
Они не заявляют формальное соответствие английскому ASD-STE100.
Ссылки на Google описывают подход к документации, а не принадлежность проекта Google.

## Исследовательские ориентиры

[FrakesH/Smart-Standing-Desk](https://github.com/FrakesH/Smart-Standing-Desk)
и [dong4j/desk-gateway](https://github.com/dong4j/desk-gateway) использовались до собственных измерений.
Их коды не являются спецификацией X2DM-SE.
Старые планы и промежуточные аудиты доступны в истории Git.
Текущие инструкции собраны в [карте документации](README.md).
