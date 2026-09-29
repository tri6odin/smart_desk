# Сборка и прошивка

ESP-IDF **6.1**, target **esp32s3**, модуль **N16R8**. Команды ниже используют
установленный ESP-IDF Installation Manager (`eim`) с окружением `v6.1`.
Host-тестам нужны Python 3, CMake и C++-компилятор; чтению ESP — пакет esptool
из окружения IDF.

## Production

Из корня репозитория:

```sh
cd firmware
eim run 'idf.py -B build-journal -D IDF_TARGET=esp32s3 -D SDKCONFIG=sdkconfig.journal -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ble-controller.defaults" build' v6.1
```

`build-journal` и `sdkconfig.journal` отделены от старых сборок. Сохранённый
sdkconfig имеет приоритет над defaults: при переходе со старой конфигурации
использовать новое имя, а не рассчитывать на автоматическую замену настроек.
Production требует `partitions.journal.csv`; прежняя таблица вызовет ошибку CMake.

Перед прошивкой отключить **J8-1 → ESP 3V3**, подключить USB и убедиться, что
штатная панель не подключена к TX ESP. Подставить текущий serial port:

```sh
eim run 'idf.py -B build-journal -D IDF_TARGET=esp32s3 -D SDKCONFIG=sdkconfig.journal -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ble-controller.defaults" -p /dev/cu.usbmodemXXXX flash' v6.1
```

Первое обновление с журналом требует полного `flash`, включая таблицу разделов,
а не `app-flash`. Обычный flash сохраняет NVS и журнал; **`erase-flash`
удаляет журнал, пресеты и BLE bond**. После загрузки проверить отсутствие
`persistent log unavailable`, подключение D100H и сохранённые цели.

USB-monitor с теми же параметрами сборки:

```sh
eim run 'idf.py -B build-journal -D SDKCONFIG=sdkconfig.journal -p /dev/cu.usbmodemXXXX monitor' v6.1
```

Чтение истории после ошибки — [отдельная инструкция](../docs/persistent-journal.md).
Новая версия с журналом пока имеет только программную проверку;
[аппаратный прогон](../docs/verification-plan.md) ещё нужен.

## Автоматическая проверка

Из корня репозитория:

```sh
python3 tools/check_firmware.py --sanitizers undefined
python3 tools/check_firmware.py --all-modes --fresh-configs --sanitizers undefined
```

Первая команда выполняет шесть наборов: protocol, controller, inputs,
BLE bonds, journal и Python-декодер. Вторая также собирает восемь режимов
через EIM. `--fresh-configs` использует собственные `sdkconfig.check-*` и
`build-check-*`, чтобы старые локальные настройки не влияли на результат.
Скрипт ничего не прошивает. CI в `.github/workflows/firmware.yml` проверяет
host-тесты и production-сборку.

## Режимы

Режим задаётся Kconfig; имя каталога сборки само по себе его не выбирает.
Базовый `sdkconfig.defaults` включает пассивный `read-only`. Для остальных
добавляется `sdkconfig.<режим>.defaults`.

| Режим | Назначение и выход UART |
|---|---|
| `ble-controller` | Рабочее управление и постоянный журнал |
| `read-only` | GPIO17/18 только RX; пассивное наблюдение обоих направлений |
| `idle` | Только IDLE, проверка замены панели |
| `step-down` | Один DOWN_STEP по BOOT при свежей высоте 110–120 см, затем блокировка до reset |
| `hold-down` | Ограниченный защитами DOWN_HOLD по BOOT, затем блокировка до reset |
| `ble-logger` | HID-диагностика, на UART только IDLE |
| `ble-one-shot` | Один медленный BLE-жест через ограниченный HOLD за запуск |
| `ble-calibration` | Лабораторные STEP и HOLD примерно на 3 см для измерения выбега |

`ble-calibration` — измерительный режим, **не** production-процедура RST.
Все режимы, кроме `read-only`, включают TX: штатную панель нужно отсоединить.
Лабораторные режимы не предназначены для повседневной эксплуатации.

Код и зависимости модулей описаны в [архитектуре](../docs/architecture.md),
параметры движения — в [управлении](../docs/ulanzi-d100h.md).
