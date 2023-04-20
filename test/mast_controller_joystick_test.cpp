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

int axis_1_val = 0; //сырое значение состояния оси 1

int axis_2_val = 0; //сырое значение состояния оси 2

int axis_fingers_val = 0; //сырое значение состояния оси "пальцы"

//==================НАСТРОЙКИ, выполняется разово при включении МК==================
void setup() {
Serial.begin(9600); //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы

//определения режимов работы пинов входов
pinMode(axis_1, INPUT); //вертикальная ось 1, потенциометр, вход

pinMode(axis_2, INPUT); //вертикальная ось 2, потенциометр, вход

pinMode(axis_fingers, INPUT); //ось "пальцы", потенциометр, вход

}


//ЗАМЕТКА - центральное положение джойстиков 512, добавить гистерезис +-10, скорректировать после подключения фактических джойстиков
//ЗАМЕТКА - нажатие джойстиков ВВЕРХ активирует реле "ВВЕРХ", а на оси "пальцы" выполняет "ЗАКРЫТИЕ", возможна корректировка логики работы после подключения фактических джойстиков

//==================ОСНОВНОЙ ЦИКЛ, выполняется пока работает МК==================
void loop() {

//==================СЧИТЫВАНИЕ ОСЕЙ==================
axis_1_val = analogRead(axis_1);
axis_2_val = analogRead(axis_2);
axis_fingers_val = analogRead(axis_fingers);


//ВЫВОД ДАННЫХ НА ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ ДЛЯ ДЕБАГГИНГА, удалить после завершения написания и проверки и перепроверки работоспособности системы
//данные оси 1
Serial.print("Axis_1_Val:"); Serial.print(axis_1_val);
//данные оси 2
Serial.print(" Axis_2_Val:"); Serial.print(axis_2_val);
//данные оси "пальцы"
Serial.print(" Fingers_Val:"); Serial.print(axis_fingers_val);
Serial.println("  ");

}
