# Структура файлів на SD картках для PX4

SD картка у PX4 використовуються для зберігання файлів налаштування, журналів польотів, інформації щодо польотного завдання тощо.

:::tip
SD картку слід відформатувати у FAT32 для використання з PX4 (за замовчуванням для SD карток).
Рекомендуємо відформатувати картки, які використовують іншу файлову систему.
:::

Структуру директорій/розкладку показано нижче.

| Директорія/Файл(и) | Опис                                                                                              |
| ------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `/etc/`                               | Додаткові налаштування. Дивіться [Запуск системи > заміна запуску системи][replace system start]. |
| `/log/`                               | Повні [журнали польоту](../dev_log/logging.md)                                                    |
| `/mission_log/`                       | Скорочені журнали польоту                                                                         |
| `/fw/`                                | Прошивка [DroneCAN](../dronecan/index.md)                                                         |
| `/uavcan.db/`                         | Сервер DB DroneCAN DNA + журнали                                                                  |
| `/params`                             | Параметри (якщо вони не в FRAM/FLASH)                                          |
| `/dataman`                            | Файл сховища польотних завдань                                                                    |
| `/fault_<datetime>.txt`               | Файли журналу апаратних відмов                                                                    |
| `/bootlog.txt`                        | Журнал завантаження                                                                               |

[replace system start]: ../concept/system_startup.md#replacing-the-system-startup
