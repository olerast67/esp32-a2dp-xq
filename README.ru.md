# esp32-a2dp-xq

Библиотека для ESP32, которая передаёт звук в Bluetooth-наушники по A2DP с SBC-XQ: SBC в режиме Dual Channel с битпулом 38, 452 кбит/с при 44,1 кГц. Если наушники XQ не принимают, библиотека переходит на Joint Stereo с битпулом 53 (328 кбит/с).

[![CI](https://github.com/olerast67/esp32-a2dp-xq/actions/workflows/ci.yml/badge.svg)](https://github.com/olerast67/esp32-a2dp-xq/actions/workflows/ci.yml)
[![Component Registry](https://components.espressif.com/components/olerast67/esp32-a2dp-xq/badge.svg)](https://components.espressif.com/components/olerast67/esp32-a2dp-xq)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

[English](README.md) · [Справочник API](https://olerast67.github.io/esp32-a2dp-xq/) · [История изменений](CHANGELOG.md)

![Как библиотека договаривается с наушниками о SBC-XQ](docs/negotiation.svg)

## Состояние

Версию 0.1.0 я ещё не запускал на железе. Первую проверку с наушниками я запишу в историю изменений вместе с моделями наушников.

Что проверено:

- В тестах на компьютере проходят 1315 проверок: длина кадра и битрейт SBC, подбор битпула кодером Bluedroid, решение о XQ по возможностям наушников, PCM-буфер с опустошением и сбросом, шкалы громкости, файл памяти устройств и фаззинг парсера.
- Примеры для ESP-IDF собираются под ESP32 на ESP-IDF 6.1 без предупреждений, с включённым Bluetooth Classic; `esp_a2d_source_set_pref_mcc` попадает в сборку примера с синусом.
- Примеры для Arduino собираются на ядре Arduino 3.3.12 (ESP-IDF 5.5) с `-Wall -Wextra` без предупреждений.

## Сравнение

| | esp32-a2dp-xq | [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) |
|---|---|---|
| SBC-XQ, Dual Channel, битпул 38, 452 кбит/с | Да, ESP-IDF 6.0+ | Нет |
| Обычный SBC, Joint Stereo, битпул 53 | Да | Да |
| Вход | 16 бит, или 32 бита с TPDF-дизерингом до 16 | 16 бит |
| Память XQ по устройствам и чёрный список | Да | Нет |
| ESP32 в роли Bluetooth-колонки (A2DP sink) | Нет | Да |
| `Stream` из Arduino и AudioTools | Нет | Да |
| Проверено на железе | Пока нет | Да |

Для Bluetooth-колонки на ESP32 или для быстрого старта с AudioTools подходит ESP32-A2DP. Эта библиотека работает только как источник звука и добавляет более высокий битрейт SBC.

## Быстрый старт

```c
#include "a2dp_xq.h"

void app_main(void) {
    nvs_flash_init();                                   // ключи сопряжения хранятся в NVS
    a2dp_xq_config_t cfg = A2DP_XQ_CONFIG_DEFAULT();
    cfg.device_name = "My ESP32";
    a2dp_xq_init(&cfg);
    a2dp_xq_connect("aa:bb:cc:dd:ee:ff");               // или a2dp_xq_scan(true)
    a2dp_xq_start();
    static int16_t pcm[256 * 2];                        // стерео, 44,1 кГц
    for (;;) a2dp_xq_write_s16(pcm, 256, portMAX_DELAY);  // ждёт, пока в буфере есть место
}
```

## Установка

ESP-IDF 5.3 и новее:

```bash
idf.py add-dependency "olerast67/esp32-a2dp-xq^0.1.0"
```

PlatformIO (`platformio.ini`):

```ini
lib_deps = https://github.com/olerast67/esp32-a2dp-xq.git#v0.1.0
```

Arduino IDE с ядром ESP32 3.x: скачайте `esp32-a2dp-xq-0.1.0.zip` со страницы [Releases](https://github.com/olerast67/esp32-a2dp-xq/releases) и добавьте через «Скетч > Подключить библиотеку > Добавить .ZIP библиотеку». Пример с синусом занимает 1,08 МБ из 1,25 МБ раздела приложения по умолчанию. Если рядом есть другой код (esp32-audio-player, Wi-Fi), выберите «Инструменты > Partition Scheme > Huge APP».

В sdkconfig проекта на ESP-IDF нужен Bluetooth Classic с A2DP. В примерах эти строки лежат в `sdkconfig.defaults`:

```
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_AVRCP_ENABLED=y
```

## Как выбирается SBC-XQ

У SBC здесь важны два стереорежима. В Joint Stereo оба канала делят один битпул, и кодер Bluedroid в ESP-IDF держит 328 кбит/с, это битпул 53. В Dual Channel у каждого канала свой битпул, и при битпуле 38 на канал поток занимает 452 кбит/с. Спецификация A2DP 1.3 (раздел 4.3.2) обязывает каждый приёмник декодировать Dual Channel. В PipeWire такая настройка называется кодеком `sbc_xq`.

После подключения библиотека до 2 секунд ждёт от наушников описание их возможностей SBC (`ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT`). XQ запрашивается, если выполнены три условия:

1. XQ разрешён в настройках (`allow_sbc_xq`).
2. Наушники поддерживают 44,1 кГц, Dual Channel, 16 блоков, 8 поддиапазонов, распределение loudness, и их диапазон битпула включает 38.
3. В памяти устройств для этих наушников нет отметки о сбое, и имени нет в чёрном списке.

Запрос уходит через `esp_a2d_source_set_pref_mcc()`: 44,1 кГц, Dual Channel, битпул 38..38. После этого кодер Bluedroid поднимает целевой битрейт шагами по 5 кбит/с, пока не получится битпул 38. Если наушники отказали, не ответили за 4 секунды или не прислали свои возможности, поток идёт в Joint Stereo с диапазоном битпула от стека.

SBC-XQ требует ESP-IDF 6.0, где появилась `esp_a2d_source_set_pref_mcc()`. На старых версиях, в том числе в ядре Arduino 3.x, библиотека пишет в лог «SBC-XQ needs ESP-IDF 6.0 or newer» и передаёт Joint Stereo. Функция `a2dp_xq_supported()` сообщает, какой из двух вариантов в сборке.

## Память устройств

Результат XQ для каждых наушников хранится в файле `<state_dir>/a2dp_xq_devices.txt`, по строке на устройство:

```
aa:bb:cc:dd:ee:ff	xq=ok	fails=0	noxq=0	seen=12	name=WH-1000XM4
```

Если поток XQ держится 30 секунд, устройство получает отметку `xq=ok`. Обрыв связи в первые 20 секунд после старта потока XQ считается одним сбоем, после двух сбоев подряд ставится `xq=fail`. Если наушники приняли настройку XQ, а потом дважды отказались запускать поток, библиотека сразу ставит `xq=fail` и переподключается в Joint Stereo. Через `a2dp_xq_set_xq_allowed()` пользователь может выключить или включить XQ для одного устройства, включение стирает прошлую отметку о сбое. В файле помещается 16 устройств; при переполнении удаляется то, которым пользовались давнее всех.

Встроенный чёрный список ищет в имени устройства без учёта регистра целые слова: Soundcore 2, CMF Buds 2a, DC800, S305, PMK TWS, Phonak. Я собрал их из списка особенностей Bluetooth-устройств в PipeWire и из баг-репортов других открытых плееров.

Без `state_dir` память хранится в RAM и стирается при перезагрузке.

## Звук и громкость

Приложение пишет стереокадры с частотой 44,1 кГц функциями `a2dp_xq_write_s16()` или `a2dp_xq_write_q31()`. Кадры попадают в буфер на 16384 кадра, который заполняется до 300 мс; память под него берётся из PSRAM, если она есть на плате. 32-битный вход переводится в 16 бит с TPDF-дизерингом, 16-битный вход на полной громкости копируется без изменений. После опустошения буфера кодер ждёт 100 мс звука и до этого отправляет тишину.

`a2dp_xq_set_volume_db()` передаёт громкость наушникам с абсолютной громкостью AVRCP (0..127, 127 соответствует 0 дБ, один шаг равен 60/127 дБ), и сам поток идёт на 0 дБ. Для наушников без абсолютной громкости, а также пока наушники её не подтвердили, библиотека ослабляет сигнал при переводе в 16 бит. Изменения громкости кнопками наушников приходят в `remote_cb` как `A2DP_XQ_REMOTE_VOLUME_ABS`.

Кнопки наушников (воспроизведение, пауза, стоп, следующий, предыдущий, громкость) приходят в `remote_cb`. Событие `A2DP_XQ_REMOTE_LINK_DOWN` приходит, когда звук запущен, наушники не подключены и переподключение не ожидается.

## Примеры

| Пример | ESP-IDF | Arduino |
|---|---|---|
| Синус 440 Гц в наушники | [sine_to_headphones](examples/idf/sine_to_headphones) | [SineToHeadphones](examples/arduino/SineToHeadphones) |
| Выбранный кодек, битпул, битрейт и статистика связи | [codec_report](examples/idf/codec_report) | [CodecReport](examples/arduino/CodecReport) |
| WAV-файл с SD-карты | | [WavFileToHeadphones](examples/arduino/WavFileToHeadphones) |

FLAC, MP3, Vorbis и альбомы без пауз между треками с SD-карты в наушники воспроизводит пример `bluetooth_headphones` из [esp32-audio-player](https://github.com/olerast67/esp32-audio-player). Для вывода в Bluetooth он использует эту библиотеку.

## Требования

- Классический ESP32 (ESP32, ESP32-WROOM, ESP32-WROVER, ESP32-PICO). У ESP32-S3, C3 и C6 нет Bluetooth Classic: на них библиотека собирается, а `a2dp_xq_init()` возвращает `ESP_ERR_NOT_SUPPORTED`.
- ESP-IDF 5.3 и новее или ядро Arduino 3.x. Для SBC-XQ нужен ESP-IDF 6.0 и новее.
- Около 100 КБ внутренней RAM под контроллер Bluetooth и Bluedroid, пока библиотека инициализирована. `a2dp_xq_deinit()` возвращает эту память.

## Тесты на компьютере

Части на чистом C собираются GCC, Clang или `zig cc` в режиме C17:

```bash
cmake -S test -B build/test
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

На Windows без компилятора C выполните `python -m pip install ziglang cmake ninja` и добавьте `-G Ninja -DCMAKE_TOOLCHAIN_FILE=<абсолютный путь>/cmake/zig-toolchain.cmake`. В CI тесты идут на Linux с AddressSanitizer и UndefinedBehaviorSanitizer.

## Лицензия

Apache License 2.0, текст в [LICENSE](LICENSE). Библиотека работает поверх стека Bluedroid из ESP-IDF (Apache-2.0) и не содержит стороннего кода.

Код взят из музыкального плеера на ESP32, который я сейчас делаю. Плеер я опубликую после его собственных проверок на железе. Большую часть кода написал Claude (Anthropic) по моим заданиям, я ставил задачи и проверял результат тестами и сборками из раздела «Состояние».
