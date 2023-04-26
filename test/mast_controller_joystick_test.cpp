/*
Разработано Олегом "Salty NUggeTZ" Величко и Игорем "Terrano"
Разработано с целью управления и мониторинга системы неизвестного происхождения и с неизвестными спецификациями
Гипотетический прототип разработан в Fritzing и собран на макетной плате
Система построена на фреймворке Arduino NANO
*/

//==================ДОБАВЛЕНИЕ НУЖНЫХ БИБЛИОТЕК==================
//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) СЛЕДУЮЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<
#include <Arduino.h> //библиотека Arduino для использования в VSCode PlatformIO (НЕ НУЖНА ЕСЛИ ПИСАТЬ И/ИЛИ ПРОШИВАТЬ ИЗ ARDUINO IDE!!!)
//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) ПРЕДЫДУЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<

//==================ОПРЕДЕЛЕНИЯ ПИНОВ==================

//входы
//датчик давления подключён к модулю INA219 который будет инициализирован ниже
int axis_1 = A0; //потенциометр джойстика оси 1
int axis_2 = A1; //потенциометр джойстика оси 2
int axis_fingers = A2;  //потенциометр джойстика оси "пальцы"


int neutral_switch_1 = 9; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_2 = 8; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_3 = 7; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ


int axis_1_val = 0; //сырое значение состояния оси 1
int axis_2_val = 0; //сырое значение состояния оси 2
int axis_fingers_val = 0; //сырое значение состояния оси "пальцы"


int neutral_switch_1_state = 0; //состояние концевика оси 1
int neutral_switch_2_state = 0; //состояние концевика оси 2
int neutral_switch_3_state = 0; //состояние концевика оси "пальцы"

//==================НАСТРОЙКИ, выполняется разово при включении МК==================
void setup() {
Serial.begin(9600); //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы

//определения режимов работы пинов входов
pinMode(axis_1, INPUT); //вертикальная ось 1, потенциометр, вход
pinMode(axis_2, INPUT); //вертикальная ось 2, потенциометр, вход
pinMode(axis_fingers, INPUT); //ось "пальцы", потенциометр, вход

pinMode(neutral_switch_1, INPUT_PULLUP); //концевик нейтрали вертикальной оси 1, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!
pinMode(neutral_switch_2, INPUT_PULLUP); //концевик нейтрали вертикальной оси 2, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!
pinMode(neutral_switch_3, INPUT_PULLUP); //концевик нейтрали оси "пальцы", !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!


}


//ЗАМЕТКА - центральное положение джойстиков 512, добавить гистерезис +-10, скорректировать после подключения фактических джойстиков
//ЗАМЕТКА - нажатие джойстиков ВВЕРХ активирует реле "ВВЕРХ", а на оси "пальцы" выполняет "ЗАКРЫТИЕ", возможна корректировка логики работы после подключения фактических джойстиков

//Axis 1 center 504 (493) +-, max 737, min 256
//Axis 2 center 510 +-, max 765, min 256
//Axis 3 center 510 +-, max 764, min 256
//switch states:
//Axis 1 center - switch is 1, goes to 0 at 510-512 moving up, 410 moving down
//Axis 2 center - switch is 1, goes to 0 at 546 moving up, 452 moving down
//Axis 3 center - switch is 1, goes to 0 at 542 moving up, 461 moving down

//==================ОСНОВНОЙ ЦИКЛ, выполняется пока работает МК==================
void loop() {

//==================СЧИТЫВАНИЕ ОСЕЙ==================
axis_1_val = analogRead(axis_1);
axis_2_val = analogRead(axis_2);
axis_fingers_val = analogRead(axis_fingers);

neutral_switch_1_state = digitalRead(neutral_switch_1);
neutral_switch_2_state = digitalRead(neutral_switch_2);
neutral_switch_3_state = digitalRead(neutral_switch_3);



//ВЫВОД ДАННЫХ НА ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ ДЛЯ ДЕБАГГИНГА, удалить после завершения написания и проверки и перепроверки работоспособности системы
//данные оси 1
Serial.print("Axis_1_Val:"); Serial.print(axis_1_val);
//данные оси 2
Serial.print(" Axis_2_Val:"); Serial.print(axis_2_val);
//данные оси "пальцы"
Serial.print(" Fingers_Val:"); Serial.print(axis_fingers_val);

Serial.print(" Switch 1 state: "); Serial.print(neutral_switch_1_state);
Serial.print(" Switch 2 state: "); Serial.print(neutral_switch_2_state);
Serial.print(" Switch 3 state: "); Serial.print(neutral_switch_3_state);


Serial.println("  ");

}
