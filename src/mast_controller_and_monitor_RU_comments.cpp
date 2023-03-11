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

#include <Wire.h> //библиотека I2C
#include <INA219_WE.h> //библиотека INA автора Wolfgang Ewald
#include <TM1637.h> //библиотека TM1637 с жёсткогого диска, находится в папке .pio (можно скачать так же с (https://drive.google.com/file/d/1DN5pDko7D1-F4POXICIfd8m_1sAKBZt7/view)

#define I2C_ADDRESS 0x40 //адрес I2C модуля INA219

//==================ОПРЕДЕЛЕНИЯ ПИНОВ==================

//входы
//датчик давления подключён к модулю INA219 который будет инициализирован ниже
int axis_1 = A0; //потенциометр джойстика оси 1
int axis_2 = A1; //потенциометр джойстика оси 2
int axis_fingers = A2;  //потенциометр джойстика оси "пальцы"

int neutral_switch_1 = 9; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_2 = 8; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_3 = 7; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ

//выходы
int up_state = 2; //статус движения "вверх", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВВЕРХ" от одной из осей (в том числе "пальцы" ОТКРЫТИЕ)
int down_state = 4; ///статус движения "вниз", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВНИЗ" от одной из осей (в том числе "пальцы" ЗАКРЫТИЕ)

int out_power_1 = 6; //ШИМ выход оси 1, пропорционален углу отклонения джойстика
int out_power_2 = 5; //ШИМ выход оси 2, пропорционален углу отклонения джойстика
int out_power_fingers = 3; //ШИМ выход оси "пальцы", пропорционален углу отклонения джойстика

//модуль TM1637
int display_clk = 12; //CLK пин модуля TM1637
int display_dio = 13; //DIO пин модуля TM1637

//==================ПЕРЕМЕННЫЕ==================
float current_mA = 0.0; //float для считывания данных с модуля INA219

bool axis_1_neutral = true; //флаг нейтрали оси 1
bool axis_1_up = true; //флаг ВВЕРХ оси 1
bool axis_1_down = true; //флаг ВНИЗ оси 1

bool axis_2_neutral = true; //флаг нейтрали оси 2
bool axis_2_up = true; //флаг ВВЕРХ оси 2
bool axis_2_down = true; //флаг ВНИЗ оси 2

bool axis_fingers_neutral = true; //флаг нейтрали оси "пальцы"
bool axis_fingers_close = true; //флаг ВВЕРХ (ЗАКРЫТЬ) оси "пальцы"
bool axis_fingers_open = true; //флаг ВНИЗ (ОТКРЫТЬ) оси "пальцы"

bool lock_1 = false; //флаг блокировки оси 1
bool lock_2 = false; //флаг блокировки оси 2
bool lock_3 = false; //флаг блокировки оси "пальцы"

int neutral_switch_1_state = 0; //состояние концевика оси 1
int neutral_switch_2_state = 0; //состояние концевика оси 2
int neutral_switch_3_state = 0; //состояние концевика оси "пальцы"

int pressure_val = 0; //переформатированное значение float с модуля INA219 в int для последующей обработки
int pressure_actual = 0; //фактическое давление в bar (атмосфер)

int axis_1_val = 0; //сырое значение состояния оси 1
int axis_1_out = 0; //переформатированное значение оси 1 для вывода ШИМ

int axis_2_val = 0; //сырое значение состояния оси 2
int axis_2_out = 0; //переформатированное значение оси 2 для вывода ШИМ

int axis_fingers_val = 0; //сырое значение состояния оси "пальцы"
int axis_fingers_out = 0; //переформатированное значение оси "пальцы" для вывода ШИМ

//ВНИМАНИЕ!!! ВОЗМОЖНО БУДУТ КОСЯКИ С ДИСПЛЕЕМ ПОСЛЕ 50 ДНЕЙ БЕЗ ПЕРЕЗАГРУЗКИ!!!
unsigned long previousmillis = 0; //когда было последнее обновление дисплея
unsigned long currentmillis = 0; //текущее время в миллисекундах
const long interval = 100; //интервал между обновлениями дисплея в миллисекундах

//==================СОЗДАНИЕ ОБЪЕКТОВ БИБЛИОТЕК==================
INA219_WE ina219 = INA219_WE(I2C_ADDRESS); //создание объекта "ina219" библиотеки INA219_WE, инициализация датчика давления
TM1637 tm1637(display_clk, display_dio); //создание объекта "tm1637" библиотеки TM1637, инициализация дисплея

//==================НАСТРОЙКИ, выполняется разово при включении МК==================
void setup() {
Serial.begin(115200); //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы
Wire.begin(); //инициализуруем интерфейс I2C

//===удалить последующие 2 строки кода +1 строку комментария после завершения написания и проверки и перепроверки работоспособности системы
  if(!ina219.init()){ //проверка подключения модуля ina219 
    Serial.println("INA219 not connected!");} //возвращает ошибку если модуль не подключён
//===удалить предыдущие 2 строки кода +1 строку комментария после завершения написания и проверки и перепроверки работоспособности системы

tm1637.init(); //инициализируем модуль tm1637
tm1637.set(BRIGHT_TYPICAL); //устанавливаем яркость дисплея

//определения режимов работы пинов входов
pinMode(axis_1, INPUT); //вертикальная ось 1, потенциометр, вход
pinMode(neutral_switch_1, INPUT_PULLUP); //концевик нейтрали вертикальной оси 1, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_2, INPUT); //вертикальная ось 2, потенциометр, вход
pinMode(neutral_switch_2, INPUT_PULLUP); //концевик нейтрали вертикальной оси 2, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_fingers, INPUT); //ось "пальцы", потенциометр, вход
pinMode(neutral_switch_3, INPUT_PULLUP); //концевик нейтрали оси "пальцы", !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

//определения режимов работы пинов выходов
pinMode(up_state, OUTPUT); //выход на реле ВВЕРХ
pinMode(down_state, OUTPUT); //выход на реле ВНИЗ
pinMode(display_clk, OUTPUT); //выход на пин CLK модуля tm1637
pinMode(display_dio, OUTPUT); // выход на пин DIO модуля tm1637

//сброс статуса блокировки на ЛОЖЬ для всех трёх осей, скорее всего удалить строки когда будут использоваться концевики
lock_1 = false; //блокировка оси 1
lock_2 = false; //блокировка оси 2
lock_3 = false; //блокировка оси "пальцы"

}


//ЗАМЕТКА - центральное положение джойстиков 512, добавить гистерезис +-10, скорректировать после подключения фактических джойстиков
//ЗАМЕТКА - нажатие джойстиков ВВЕРХ активирует реле "ВВЕРХ", а на оси "пальцы" выполняет "ЗАКРЫТИЕ", возможна корректировка логики работы после подключения фактических джойстиков

//==================ОСНОВНОЙ ЦИКЛ, выполняется пока работает МК==================
void loop() {
//выключить ШИМ выход если ось в нейтрали
if (axis_1_neutral == true){digitalWrite(out_power_1, LOW);}
if (axis_2_neutral == true){digitalWrite(out_power_2, LOW);}
if (axis_fingers_neutral == true){digitalWrite(out_power_fingers, LOW);}

currentmillis = millis(); //записать текущее время с последней перезагрузки

//==================ПОЛУЧЕНИЕ СЫРЫХ ДАННЫХ С ДАТЧИКА ДАВЛЕНИЯ==================
current_mA = ina219.getCurrent_mA(); //получение значения тока на модуде INA219
current_mA = constrain (current_mA, 0.5, 30); //ПРОГРАММНОЕ ограничение значения тока от 0.5мА до 30мА для упрощения расчётов

//==================КОНВЕРТИРОВАНИЕ ДАННЫХ С ФОРМАТА float В ФОРМАТ int==================
pressure_val = current_mA*100;

//==================КОНВЕРТИРОВАНИЕ СЫРЫХ ДАННЫХ В ФАКТИЧЕСКОЕ ДАВЛЕНИЕ ДЛЯ ВЫВОДА НА ДИСПЛЕЙ >>>!!!ОБЯЗАТЕЛЬНО ОТКАЛИБРОВАТЬ!!!<<<==================
pressure_actual = map(pressure_val, 50, 3000, 0, 250);

//==================СЧИТЫВАНИЕ ОСЕЙ==================
axis_1_val = analogRead(axis_1);
axis_2_val = analogRead(axis_2);
axis_fingers_val = analogRead(axis_fingers);

//==================СЧИТЫВАНИЕ КОНЦЕВИКОВ==================
neutral_switch_1_state = digitalRead(neutral_switch_1);
neutral_switch_2_state = digitalRead(neutral_switch_2);
neutral_switch_3_state = digitalRead(neutral_switch_3);

//==================УСТАНОВКА ФЛАЖКОВ БЛОКИРОВКИ==================

if (neutral_switch_1_state == 1){axis_1_neutral = true;}
else {axis_1_neutral = false;}
if (neutral_switch_2_state == 1){axis_2_neutral = true;}
else {axis_2_neutral = false;}
if (neutral_switch_3_state == 1){axis_fingers_neutral = true;}
else {axis_fingers_neutral = false;}

//==================БЛОКИРОВКА ОСЕЙ ПО ФЛАЖКАМ==================
if (axis_2_neutral == false || axis_fingers_neutral == false){lock_1 = true;} //блокировка оси если ЛЮБАЯ из двух остальных осей НЕ в нейтрали
if (axis_2_neutral == true && axis_fingers_neutral == true){lock_1 = false;} //разблокировка оси если ОБЕ остальные оси в нейтрали

if (axis_1_neutral == false || axis_fingers_neutral == false){lock_2 = true;} //блокировка оси если ЛЮБАЯ из двух остальных осей НЕ в нейтрали
if (axis_1_neutral == true && axis_fingers_neutral == true){lock_2 = false;} //разблокировка оси если ОБЕ остальные оси в нейтрали

if (axis_1_neutral == false || axis_2_neutral == false){lock_3 = true;} //блокировка оси если ЛЮБАЯ из двух остальных осей НЕ в нейтрали
if (axis_1_neutral == true && axis_2_neutral == true){lock_3 = false;} //разблокировка оси если ОБЕ остальные оси в нейтрали

if (axis_1_neutral == true){digitalWrite(out_power_1, LOW);} //выключение вывода ШИМ если ось в нейтрали
if (axis_2_neutral == true){digitalWrite(out_power_2, LOW);} //выключение вывода ШИМ если ось в нейтрали
if (axis_fingers_neutral == true){digitalWrite(out_power_fingers, LOW);} //выключение вывода ШИМ если ось в нейтрали


//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ 1==================

    if (axis_1_val >= 522){
        axis_1_out = map(axis_1_val, 522, 1023, 0, 255);
        axis_1_up = true;
        axis_1_down = false;
        }
        
    if (axis_1_val <= 502){
        axis_1_out = map(axis_1_val, 0, 502, 255, 0);
        axis_1_up = false;
        axis_1_down = true;
        }
    if (axis_1_val > 502 && axis_1_val <= 522){axis_1_up = false, axis_1_down = false, axis_1_out = 0;
    }    

//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ 2==================

    if (axis_2_val >= 522){
        axis_2_out = map(axis_2_val, 522, 1023, 0, 255);
        axis_2_up = true;
        axis_2_down = false;
        }
        
    if (axis_2_val <= 502){
        axis_2_out = map(axis_2_val, 0, 502, 255, 0);
        axis_2_up = false;
        axis_2_down = true;
        }
    if (axis_2_val > 502 && axis_2_val <= 522){axis_2_up = false, axis_2_down = false, axis_2_out = 0;
    }

//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ "пальцы"==================

    if (axis_fingers_val >= 522){
        axis_fingers_out = map(axis_fingers_val, 522, 1023, 0, 255);
        axis_fingers_close = true;
        axis_fingers_open = false;
        }
        
    if (axis_fingers_val <= 502){
        axis_fingers_out = map(axis_fingers_val, 0, 502, 255, 0);
        axis_fingers_close = false;
        axis_fingers_open = true;
        }
    if (axis_fingers_val > 502 && axis_fingers_val <= 522){axis_fingers_close = false, axis_fingers_open = false, axis_fingers_out = 0;
    }

//ВКЛЮЧЕНИЕ СООТВЕТСТВУЮЩИХ РЕЛЕ И ШИМ ВЫХОДОВ
if (axis_1_neutral == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_2_neutral == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}
if (axis_fingers_neutral == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, LOW);}

if (axis_1_up == true && lock_1 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_1, axis_1_out);}
if (axis_2_up == true && lock_2 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_2, axis_2_out);}
if (axis_fingers_close == true && lock_3 == false){digitalWrite(up_state, HIGH), digitalWrite(down_state, LOW), analogWrite(out_power_fingers, axis_fingers_out);}

if (axis_1_down == true && lock_1 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_1, axis_1_out);}
if (axis_2_down == true && lock_2 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_2, axis_2_out);}
if (axis_fingers_open == true && lock_3 == false){digitalWrite(up_state, LOW), digitalWrite(down_state, HIGH), analogWrite(out_power_fingers, axis_fingers_out);}

//ВЫВОД ЗНАЧЕНИЯ ДАВЛЕНИЯ НА ДИСПЛЕЙ
    if (currentmillis - previousmillis >= interval) { //если прошло больше времени чем интервал...
        tm1637.clearDisplay(); //...то очистить дисплей...
        tm1637.displayInt(pressure_actual); //...вывести значение на дисплей...
        previousmillis = currentmillis; //...и обновить таймер
    }


//ВЫВОД ДАННЫХ НА ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ ДЛЯ ДЕБАГГИНГА, удалить после завершения написания и проверки и перепроверки работоспособности системы
//данные оси 1
Serial.print("A1V:"); Serial.print(axis_1_val); Serial.print(" A1O:"); Serial.print(axis_1_out);
Serial.print(" A1N:"); Serial.print(axis_1_neutral);
Serial.print(" A1U:"); Serial.print(axis_1_up);
Serial.print(" A1D:"); Serial.print(axis_1_down);
Serial.print(" A1Lock:"); Serial.print(lock_1);
//данные оси 2
Serial.print(" A2V:"); Serial.print(axis_2_val); Serial.print(" A2O:"); Serial.print(axis_2_out);
Serial.print(" A2N:"); Serial.print(axis_2_neutral);
Serial.print(" A2U:"); Serial.print(axis_2_up);
Serial.print(" A2D:"); Serial.print(axis_2_down);
Serial.print(" A2Lock:"); Serial.print(lock_2);
//данные оси "пальцы"
Serial.print(" FV:"); Serial.print(axis_fingers_val); Serial.print(" FO:"); Serial.print(axis_fingers_out);
Serial.print(" AFN: "); Serial.print(axis_fingers_neutral);
Serial.print(" AFU: "); Serial.print(axis_fingers_close);
Serial.print(" AFD: "); Serial.print(axis_fingers_open);
Serial.print(" AFLock: "); Serial.print(lock_3);
//ток на модуле INA219
Serial.print(" Current[mA]:"); Serial.print(current_mA);
//ток на модуле*100
Serial.print(" PressVal:"); Serial.print(pressure_val);
//фактическое давление
Serial.print(" PressAct:"); Serial.print(pressure_actual);
//пустая строчка, перенос каретки
Serial.println("  ");

}
