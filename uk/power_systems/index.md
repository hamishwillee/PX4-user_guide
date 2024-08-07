# Системи живлення

БПЛА потребують регульованого джерела живлення для керуючого контролера, окрім окремого живлення для двигунів, сервоприводів та будь-яких інших периферійних пристроїв.
Зазвичай живлення надходить від акумулятора (або акумуляторів), хоча можуть використовуватися також генератори та інші системи.

Модулі живлення часто використовуються для "розгалуження" регульованого джерела живлення для керуючого контролера та вимірювання напруги батареї та загального струму, спожитого транспортним засобом.
PX4 може використовувати цю інформацію для визначення залишкової ємності акумулятора та надання попереджень про низький заряд батареї та іншу аварійну поведінку.

Дошка розподілу живлення (PDB) може бути використана для спрощення проводки для розгалуження виводу батареї до керуючого контролера, двигунів та інших периферійних пристроїв.
PDB іноді будуть включати модуль живлення, ЕСП для двигунів та ланцюг елімінації батареї (BEC) для живлення сервоприводів.

PX4 також може отримувати більш повну інформацію про батарею/живлення як телеметрію MAVLink замість використання модуля живлення.
Батареї, що можуть надавати інформацію по MAVLink, іноді називаються "розумними батареями" (це визначення відкрите для обговорення).

- [Модулі живлення/PDB](../power_module/README.md)
- [Розумні/MAVLink Батареї](../smart_batteries/README.md)
