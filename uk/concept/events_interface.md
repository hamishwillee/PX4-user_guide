# Інтерфейс подій

<Badge type="tip" text="PX4 v1.13" />

Інтерфейс може використовуватися для публікації подій змін стану або будь-якого іншого типу події, включаючи такі речі, як стан готовності, завершення калібрування і досягнення цільової висоти злету.

Інтерфейс може використовуватися для публікації подій змін стану або будь-якого іншого типу події, включаючи такі речі, як стан готовності, завершення калібрування і досягнення цільової висоти злету.

:::note
інтерфейс подій замінить використання викликів `mavlink_log_*` у коді PX4, (а також повідомлень `STATUS_TEXT` у MAVLink) для сповіщення про події в релізах PX4 після v1.13 та новіше. Буде проміжний період коли [обидва підходи будуть підтримуватися](#backward-compatibility).
:::

## Застосування

### Основне

А потім визначте та надішліть подію з бажаного місця коду:

```cpp
#include <px4_platform_common/events.h>
```

А потім визначте та надішліть подію з бажаного місця коду:

```cpp
events::send(events::ID("mymodule_test"), events::Log::Info, "Test Message");
```

#### Зворотна сумісність

Для старих версій GCS без підтримки інтерфейсу подій, PX4 на цей момент надсилає також всі події як `mavlink_log_*` `STATUSTEXT` повідомлення. Крім того, повідомлення має бути промарковано додатковою табуляцією (`\t`), щоб нові GCS змогли проігнорувати це і показати тільки подію.

Отже, коли ви додаєте подію, не забудьте також додати виклик `mavlink_log_`.

```cpp
mavlink_log_info(mavlink_log_pub, "Test Message\t");
events::send(events::ID("mymodule_test"), events::Log::Info, "Test Message");
```

Вище - мінімальний приклад, цей - у більш розширеному вигляді:

### Докладно

Пояснення та вимоги:

```cpp
uint8_t arg1 = 0;
float arg2 = -1.f;
/* EVENT
 * @description
 * Це докладний опис події.
 *
 * - значення arg1: {1}
 * - значення arg2: {2:.1}
 *
 * <profile name="dev">
 * (Цей абзац призначено для показу тільки розробникам).
 * Цю поведінку можна налаштувати за допомогою параметра <param>COM_EXAMPLE</param>.
 * </profile>
 *
 * Посилання на документацію: <a>https://docs.px4.io</a>
 */
events::send<uint8_t, float>(events::ID("event_name"),
    {events::Log::Error, events::LogInternal::Info}, "Event Message", arg1, arg2);
```

Події можуть мати незмінний набір аргументів, які можна вкласти у повідомлення або опис використовуючи шаблонні замінники (наприклад `{2:.1m}`, дивіться наступний розділ).

- `/* EVENT`: Цей тег вказує, що коментар описує метадані для наступної події.
- **event_name**: ім'я події (`events::ID(event_name)`).
  - повинно бути унікальним в межах всього вихідного коду PX4. Як загальне правило, додайте префікс з назвою модуля або вихідного файлу для великих модулів.
  - має бути дійсна назва змінної, тобто не повинна містити пробіли, двокрапки тощо.
  - з цього імені отримується 24-бітний ID події за допомогою геш-функції. Це означає, що до тих пір, поки ім'я події залишається однаковим, ID залишиться тим же.
- **Рівень журналювання**:

  - припустимі рівні журналювання такі ж, як і у перерахуванні MAVLink [MAV_SEVERITY](https://mavlink.io/en/messages/common.html#MAV_SEVERITY). Рівні перелічені за зменшенням важливості:

    ```plain
    Emergency,
    Alert,
    Critical,
    Error,
    Warning,
    Notice,
    Info,
    Debug,
    Disabled,
    ```

  ```
  - Попередньо ми вказали окремий зовнішній і внутрішній рівень журналювання, які є рівнями для користувачів GCS і в файлі журналу, відповідно: `{events::Log::Error, events::LogInternal::Info}`.
    Для більшості випадків ви можете передати один рівень журналювання, і це буде використовуватися як для зовнішніх, так і для внутрішніх випадків.
  Є випадки, коли є сенс мати два різних рівні журналювання.
  Наприклад, дія запобігання RTL: користувач повинен бачити його як Warning/Error, тоді як в журналі це очікувана відповідь системи, тому його можна встановити в `Info`.
  ```

- **Повідомлення про подію**:
  - Коротке повідомлення про подію в один рядок. Може мати шаблонні замінники для аргументів (наприклад `{1}`). Для додаткової інформації дивіться нижче.
- **Опис події**:
  - Докладний, необов'язковий опис події.
  - Може бути кілька рядів/абзаців.
  - Може мати шаблонні замінники для аргументів (тобто `{2}`) та тегів що підтримуються (дивіться нижче).

#### Аргументи та перерахування

Припустимі типи: `uint8_t`, `int8_t`, `uint16_t`, `int16_t`, `uint32_t`, `int32_t`, `uint64_t`, `int64_t` та `float`.

Ви також можете використати перерахування як аргументи:

Формат тексту для опису повідомлення події:

- Користувацькі або характерні для PX4 перерахування для подій повинні бути визначені у [src/lib/events/enums.json](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/events/enums.json), та можуть бути використані як аргументи події у формі `events::send<events::px4::enums::my_enum_t>(...)`.
- "Загальні" події MAVLink визначені у [mavlink/libevents/events/common.json](https://github.com/mavlink/libevents/blob/master/events/common.json) та можуть бути використані як аргументи подій у формі `events::send<events::common::enums::my_enum_t>(...)`.

#### Формат тексту

Події записуються відповідно до рівня внутрішнього журналювання, а [Огляд польоту](../log/flight_review.md) показує події.

- символи можна екранувати за допомогою \\

  Ці символи повинні бути екрановані: '\\\\', '\\<', '\\{'.

- теги що підтримуються:

  - Профілі: `<profile name="[!]NAME">CONTENT</profile>`

    `CONTENT` буде показано, лише якщо назва збігається з налаштованим профілем. Це може бути використано, наприклад, щоб приховати інформацію для розробників від кінцевих користувачів.

  - URL: `<a [href="URL"]>CONTENT</a>`. Якщо `href` не встановлено, використовуйте `CONTENT` як `URL` (наприклад `<a>https://docs.px4.io</a>`  сприймається як `<a href="https://docs.px4.io">https://docs.px4.io</a>`)
  - Параметри: `<param>PARAM_NAME</param>`
  - не дозволено використовувати вкладені теги того ж типу

- аргументи: шаблонні замінники, що відповідають синтаксису python з індексацією що починається з 1 (замість 0)

  - загальна форма: `{ARG_IDX[:.NUM_DECIMAL_DIGITS][UNIT]}`

    UNIT:

    - m: горизонтальна відстань в метрах
    - m_v: вертикальна відстань в метрах
    - m^2: площа в метрах квадратних
    - m/s: швидкість у метрах в секунду
    - C: температура у градусах Цельсія

  - `NUM_DECIMAL_DIGITS` підходить тільки для аргументів у вигляді дійсних чисел.

## Журналювання

Події записуються відповідно до рівня внутрішнього журналювання, а [Огляд польоту](../log/flight_review.md) показує події.

:::note
Огляд польоту завантажує метадані на основі головної гілки PX4, тому, якщо визначення ще немає на головній гілці, огляд зможе показати тільки ID події.
:::

## Реалізація

Метадані для всіх подій вбудовані в окремий JSON файл метаданих (з використанням python скрипту, який сканує весь вихідний код у пошуках викликів подій).

Метадані для всіх подій вбудовані в окремий JSON файл метаданих (з використанням python скрипту, який сканує весь вихідний код у пошуках викликів подій).

### Публікація метаданих події в GCS

Цей процес такий самий як і для [метаданих параметрів](../advanced/parameters_and_configurations.md#publishing-parameter-metadata-to-a-gcs). Для отримання додаткової інформації див. [ Метадані PX4 (трансляція і публікація)](../advanced/px4_metadata.md).

Цей процес такий самий як і для [метаданих параметрів](../advanced/parameters_and_configurations.md#publishing-parameter-metadata-to-a-gcs). Для отримання додаткової інформації див. [ Метадані PX4 (трансляція і публікація)](../advanced/px4_metadata.md).
