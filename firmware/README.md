# Сборка и прошивка

Рабочая цель: ESP32-S3-WROOM-1-N16R8, ESP-IDF 6.1.
Прошивка использует 16 МБ flash и не использует PSRAM.
Команды не зависят от имени пользователя или каталога его ESP-IDF.

## Требования

| Задача | Инструменты |
|---|---|
| Проверки на компьютере | Python 3, CMake, компилятор C++17 |
| Сборка ESP | ESP-IDF 6.1 для ESP32-S3 |
| Запись и чтение flash | esptool из окружения ESP-IDF, USB-доступ к плате |
| Сборка всех режимов скриптом проекта | ESP-IDF Installation Manager с командой eim и окружением v6.1 |

Установите ESP-IDF по [инструкции Espressif](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/get-started/index.html).
Файл eim_config.toml относится к конкретному компьютеру.
Он не хранится в Git.

## Сборка рабочего режима

Откройте терминал с активированным ESP-IDF 6.1.
Из корня репозитория выполните:

```sh
cd firmware
idf.py -B build-offline \
  -D IDF_TARGET=esp32s3 \
  -D SDKCONFIG=sdkconfig.offline \
  -D SDKCONFIG_DEFAULTS='sdkconfig.defaults;sdkconfig.ble-controller.defaults' \
  build
```

Если установлен EIM, используйте эквивалентную команду:

```sh
cd firmware
eim run 'idf.py -B build-offline -D IDF_TARGET=esp32s3 -D SDKCONFIG=sdkconfig.offline -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ble-controller.defaults" build' v6.1
```

Созданный sdkconfig имеет приоритет над defaults.
Для перехода со старой конфигурации используйте новое имя SDKCONFIG.
Не ожидайте автоматической замены сохранённых настроек.
CMake требует partitions.journal.csv, отключённую консоль и нулевой уровень логов.
Неподходящая конфигурация завершает сборку с ошибкой.

## Запись прошивки

1. Остановите стол.
2. Отсоедините J8-1 → ESP 3V3 по [инструкции подключения](../docs/esp32-board.md#подключение-для-usb).
3. Проверьте отсутствие штатной панели на командной линии TX ESP.
4. Подключите USB.
5. Определите serial port платы.
6. Из каталога firmware выполните команду ниже.

Замените `/dev/cu.usbmodemXXXX` своим портом.
На Linux имя обычно имеет вид `/dev/ttyACM0` или `/dev/ttyUSB0`.
На Windows используется имя COM-порта.

```sh
idf.py -B build-offline \
  -D IDF_TARGET=esp32s3 \
  -D SDKCONFIG=sdkconfig.offline \
  -D SDKCONFIG_DEFAULTS='sdkconfig.defaults;sdkconfig.ble-controller.defaults' \
  -p /dev/cu.usbmodemXXXX -b 115200 flash
```

Команда flash записывает загрузчик, таблицу разделов и приложение.
App-flash не обновляет таблицу разделов.
Обычный flash не содержит образов NVS или desk_log.
Завершённые записи, пресеты и привязка сохраняются.
Erase-flash удаляет эти данные.

Esptool должен подтвердить проверку хэшей записанных образов.
После перезапуска прочитайте журнал.
Проверьте BOOT с идентификатором нового ELF и PRESETS с прежними настройками.
Чтение описано в [инструкции диагностики](../docs/persistent-journal.md#чтение-через-usb).
Для аппаратной проверки выполните [один прогон](../docs/single-run-test.md).

## Консоль

Рабочая прошивка не выводит UART-кадры, HID-события или переходы состояний в USB.
RTC-диагностика удалена.
Логи ESP-IDF и загрузчика отключены конфигурацией.
ROM-сообщения и сообщения фатальной ошибки платформы не являются live-журналом приложения.
Лабораторный HID-вывод компилируется только для ble-logger.

## Проверки на компьютере

Из корня репозитория выполните:

```sh
python3 tools/check_firmware.py --sanitizers undefined
```

Скрипт выполняет шесть наборов: protocol, controller, inputs, BLE bonds, journal и decoder.
Он не прошивает устройство.
Полное сравнение сценариев с требованиями описано в [плане проверки](../docs/verification-plan.md).
Прошедшие тесты не доказывают полноту журнала или безопасность механизма.

Для сборки всех режимов через EIM выполните:

```sh
python3 tools/check_firmware.py --all-modes --fresh-configs --sanitizers undefined
```

Fresh-configs использует собственные sdkconfig.check-* и build-check-*.
Настройки других локальных сборок не влияют на результат.
CI проверяет host-наборы и рабочую сборку ESP32-S3.

## Лабораторные режимы

Kconfig задаёт режим сборки.
Имя каталога build само по себе режим не выбирает.
Базовый sdkconfig.defaults выбирает read-only.
Для остальных режимов добавляется соответствующий sdkconfig.<режим>.defaults.

| Режим | Назначение |
|---|---|
| ble-controller | Рабочее управление и постоянный журнал |
| read-only | GPIO17 и GPIO18 работают только как RX |
| idle | Передача только IDLE |
| step-down | Один DOWN_STEP по BOOT при свежей высоте 110–120 см |
| hold-down | Ограниченный защитами DOWN_HOLD по BOOT |
| ble-logger | Вывод HID-данных; на UART только IDLE |
| ble-one-shot | Один BLE-жест через ограниченный HOLD за запуск |
| ble-calibration | Лабораторные STEP и HOLD примерно на 3 см для измерения выбега |

Все режимы, кроме read-only, включают TX.
Отсоедините штатную панель перед их использованием.
Ble-calibration измеряет выбег и не выполняет рабочую процедуру RST.
Однократные тесты блокируют повторный запуск до перезапуска ESP.

## Учёт версии

Сохраняйте commit, версию ESP-IDF и полный SHA-256 ELF для каждого аппаратного прогона.
BOOT содержит только первые восемь байтов хэша.
Записывайте результат прошивки отдельно от результата движения.
Датированные установки приведены в [истории измерений](../docs/measurements.md).
