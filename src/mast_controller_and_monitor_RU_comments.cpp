/*
Разработано Олегом "Salty NUggeTZ" Величко и Игорем "Terrano"
Разработано с целью управления и мониторинга системы неизвестного происхождения и с неизвестными спецификациями
Гипотетический прототип разработан в Fritzing и собран на макетной плате
Система построена на платформе Arduino (NANO)
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
//датчик давления подключён к модулю INA219 который будет инициализирован ниже, пины А4 и А5

int valve_test_pin = A7;

int axis_1 = A0; //потенциометр джойстика оси 1
int axis_2 = A1; //потенциометр джойстика оси 2
int axis_fingers = A2;  //потенциометр джойстика оси "пальцы"

int neutral_switch_1 = 8; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_2 = 9; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_3 = 10; //концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ

//выходы


//определить 8 выходов (7 цифровых выходов (6 управляющих реле + 1 реле для тестов), 1 ШИМ выход для мосфета)

/*----------------------------------------------------------------
out1-D2 (Выход на реле1) реле 1-2 подъемспуск первой секции мачты
out2-D3(выход на реле2)
out3-D4(выход на реле3) реле 3-4 подъемспуск второй секции мачты
out4-D5(выход на реле4)
out5-D6(выход на реле5) реле 5-6 выдвинутьзадвинуть пальцы
out6-D7(выход на реле6)
D11-выход ШИМ на управление мосфетом
*/
int test_relay = 1; //реле замыкается только для проверки сопротивления (напряжения) на клапанах, совместно с реле управления

int mast1_up_relay = 2;
int mast1_down_relay = 3;

int mast2_up_relay = 4;
int mast2_down_relay = 5;

int fingers_open_relay = 6;
int fingers_close_relay = 7;

int output_power = 11;

/*
УДАЛИТЬ ПОСЛЕ ПРОВЕРКИ КОДА
int up_state = 2; //статус движения "вверх", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВВЕРХ" от одной из осей (в том числе "пальцы" ОТКРЫТИЕ)
int down_state = 4; ///статус движения "вниз", цифровой выход (ВКЛ/ВЫКЛ) включается при получении комманды "ВНИЗ" от одной из осей (в том числе "пальцы" ЗАКРЫТИЕ)

int out_power_1 = 6; //ШИМ выход оси 1, пропорционален углу отклонения джойстика
int out_power_2 = 5; //ШИМ выход оси 2, пропорционален углу отклонения джойстика
int out_power_fingers = 3; //ШИМ выход оси "пальцы", пропорционален углу отклонения джойстика
*/

//модуль TM1637
int display_clk = 12; //CLK пин модуля TM1637
int display_dio = 13; //DIO пин модуля TM1637

//==================ПЕРЕМЕННЫЕ==================
float current_mA = 0.0; //float для считывания данных с модуля INA219

float valve_test_value = 0.0; //float для проверки сопротивления (напряжения) на клапанах

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

//Флажки ошибок по току
bool open_circuit = false; //обрыв цепи
bool short_circuit = false; //короткое замыкание

//==================СОЗДАНИЕ ОБЪЕКТОВ БИБЛИОТЕК==================
INA219_WE ina219 = INA219_WE(I2C_ADDRESS); //создание объекта "ina219" библиотеки INA219_WE, инициализация датчика давления
TM1637 tm1637(display_clk, display_dio); //создание объекта "tm1637" библиотеки TM1637, инициализация дисплея

//==================НАСТРОЙКИ, выполняется разово при включении МК==================
void setup() {
//Serial.begin(9600); //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы
Wire.begin(); //инициализуруем интерфейс I2C

//===удалить последующие 2 строки кода +1 строку комментария после завершения написания и проверки и перепроверки работоспособности системы
//  if(!ina219.init()){ //проверка подключения модуля ina219 
//    Serial.println("INA219 not connected!");} //возвращает ошибку если модуль не подключён
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
pinMode(mast1_up_relay, OUTPUT); //мачта элемент 1 выход на реле ВВЕРХ
pinMode(mast1_down_relay, OUTPUT); //мачта элемент 1 выход на реле ВНИЗ
pinMode(mast2_up_relay, OUTPUT); //мачта элемент 2 выход на реле ВВЕРХ
pinMode(mast2_down_relay, OUTPUT); //мачта элемент 2 выход на реле ВНИЗ
pinMode(fingers_open_relay, OUTPUT); //пальцы выход на реле ВЫДВИНУТЬ
pinMode(fingers_close_relay, OUTPUT); //пальцы выход на реле ЗАДВИНУТЬ

pinMode(display_clk, OUTPUT); //выход на пин CLK модуля tm1637
pinMode(display_dio, OUTPUT); // выход на пин DIO модуля tm1637

//сброс статуса блокировки на ЛОЖЬ для всех трёх осей, скорее всего удалить строки когда будут использоваться концевики
lock_1 = false; //блокировка оси 1
lock_2 = false; //блокировка оси 2
lock_3 = false; //блокировка оси "пальцы"
//
//проверить исправвность клапанов путём подачи 1.1В на aref и на входе А7 значение должно быть в диапазоне 0.857 до 0.888, вывести на дисплэй U1on, U2on, U3on - U6on, если ошибка - err
//если напряжение ниже 0.857 - U1SC (короткое замыкание), если выше 0.888 то U1OC (обрыв цепи) 
//для проверки напряжения вкл Д1, Д2, Д3... Д6, подаём на них ареф 1.1В, считываем на А7
//ВСЁ ИДЁТ ЧЕРЕЗ РЕЛЕ ПРОВЕРОК НА Д1!!!
//проверка работоспособности модуля ТМ путём вывода символов "_OC_" (open circuit, обрыв цепи), "_SC_" (short circuit, короткое замыкани) на 0.5 сек каждое, "DONE" и пауза на 2 секунды
tm1637.displayByte(_d, _i, _S, _P);
delay(500);
tm1637.displayByte(_t, _E, _S, _t);
delay(500);

tm1637.displayByte(_8, _8, _8, _8);
delay(500);

tm1637.displayByte(_dash, _O, _C, _dash);
delay(500);

tm1637.displayByte(_dash, _S, _C, _dash);
delay(500);

tm1637.displayByte(_d, _i, _S, _P);
delay(500);
tm1637.displayByte(_t, _E, _S, _t);
delay(500);
tm1637.displayByte(_D, _O, _N, _E);
delay(1000);

//проверка сопротивления на клапанах 1-6
analogReference(INTERNAL); //ставим опорное напряжение на 1.1В для ардуино нано

//клапан 1
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _1, _dash); //отображаем U1 на дисплее, напряжение на клапане 1
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(mast1_up_relay, LOW); //замыкаем реле клапана 1
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _1, _O, _C); //выводим на дисплей ошибку "клапан 1 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _1, _S, _C); //выводим на дисплей ошибку "клапан 1 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _1, _o, _n); //выводим на дисплей "клапан 1 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(mast1_up_relay, HIGH);
//конец проверки клапана 1

//клапан 2
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _2, _dash); //отображаем U2 на дисплее, напряжение на клапане 2
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(mast1_down_relay, LOW); //замыкаем реле клапана 2
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _2, _O, _C); //выводим на дисплей ошибку "клапан 2 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _2, _S, _C); //выводим на дисплей ошибку "клапан 2 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _2, _o, _n); //выводим на дисплей "клапан 2 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(mast1_down_relay, HIGH);
//конец проверки клапана 2

//клапан 3
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _3, _dash); //отображаем U3 на дисплее, напряжение на клапане 3
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(mast2_up_relay, LOW); //замыкаем реле клапана 3
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _3, _O, _C); //выводим на дисплей ошибку "клапан 3 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _3, _S, _C); //выводим на дисплей ошибку "клапан 3 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _3, _o, _n); //выводим на дисплей "клапан 3 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(mast2_up_relay, HIGH);
//конец проверки клапана 3

//клапан 4
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _4, _dash); //отображаем U4 на дисплее, напряжение на клапане 4
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(mast2_down_relay, LOW); //замыкаем реле клапана 4
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _4, _O, _C); //выводим на дисплей ошибку "клапан 4 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _4, _S, _C); //выводим на дисплей ошибку "клапан 4 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _4, _o, _n); //выводим на дисплей "клапан 4 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(mast2_down_relay, HIGH);
//конец проверки клапана 4

//клапан 5
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _5, _dash); //отображаем U5 на дисплее, напряжение на клапане 5
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(fingers_open_relay, LOW); //замыкаем реле клапана 5
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _5, _O, _C); //выводим на дисплей ошибку "клапан 5 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _5, _S, _C); //выводим на дисплей ошибку "клапан 5 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _5, _o, _n); //выводим на дисплей "клапан 5 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(fingers_open_relay, HIGH);
//конец проверки клапана 5

//клапан 6
tm1637.displayByte(_t, _E, _S, _t); //отображаем teSt на дисплее
delay(500); //пауза на 0.5 секунд
tm1637.displayByte(_dash, _U, _6, _dash); //отображаем U6 на дисплее, напряжение на клапане 6
delay(500); //пауза на 0.5 секунд

digitalWrite(test_relay, LOW); //замыкаем тестовое реле
digitalWrite(fingers_close_relay, LOW); //замыкаем реле клапана 6
delay(100); //пауза 0.1 секунд

valve_test_value = analogRead(valve_test_pin); //считываем значение на тестовом пине
    if (valve_test_value < 0.857) //проверяем если значение менне чем 0.857, то...
    {
        tm1637.displayByte(_U, _6, _O, _C); //выводим на дисплей ошибку "клапан 6 обрыв цепи"
        delay(500); //пауза на 0.5 сек
    }
    else if(valve_test_value > 0.888) //проверяем если значение более чем 0.888, то
    {
        tm1637.displayByte(_U, _6, _S, _C); //выводим на дисплей ошибку "клапан 6 короткое замыкание"
        delay(500); //пауза на 0.5 сек
    }
    else //если иначе, то по логике вещей значение между 0.857 и 0.888, и выводим
    {
        tm1637.displayByte(_U, _6, _o, _n); //выводим на дисплей "клапан 6 включен"
        delay(500); //пауза на 0.5 сек
    }
digitalWrite(test_relay, HIGH);
digitalWrite(fingers_close_relay, HIGH);
//конец проверки клапана 6

analogReference(DEFAULT);

tm1637.displayByte(_U, _1, _U, _6);
delay(500);

tm1637.displayByte(_t, _E, _S, _t);
delay(500);

tm1637.displayByte(_D, _O, _N, _E);
delay(1000);

tm1637.displayByte(_r, _E, _d, _Y);
delay(3000);

//конец проверки напряжения на клапанах
//размыкание всех реле
digitalWrite(test_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(mast1_up_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(mast1_down_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(mast2_up_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(mast2_down_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(fingers_open_relay, HIGH); //установить состояние пинов в высокий логический уровень
digitalWrite(fingers_close_relay, HIGH); //установить состояние пинов в высокий логический уровень
}


//ЗАМЕТКА - центральное положение джойстиков 512, добавить гистерезис +-10, скорректировать после подключения фактических джойстиков
//ЗАМЕТКА - нажатие джойстиков ВВЕРХ активирует реле "ВВЕРХ", а на оси "пальцы" выполняет "ЗАКРЫТИЕ", возможна корректировка логики работы после подключения фактических джойстиков
//Axis 1 center 504 (493) +-, max 737, min 256
//Axis 2 center 510 +-, max 765, min 256
//Axis 3 center 510 +-, max 764, min 256
//Axis 1 center - switch is 1, goes to 0 at 510-512 moving up, 410 moving down
//Axis 2 center - switch is 1, goes to 0 at 546 moving up, 452 moving down
//Axis 3 center - switch is 1, goes to 0 at 542 moving up, 461 moving down

//==================ОСНОВНОЙ ЦИКЛ, выполняется пока работает МК==================
void loop() {

tm1637.clearDisplay(); //очистить дисплей
//выключить ШИМ выход и разомкнуть все реле если ось в нейтрали
if (axis_1_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast1_up_relay, HIGH);digitalWrite(mast1_down_relay, HIGH);}
if (axis_2_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast2_up_relay, HIGH);digitalWrite(mast2_down_relay, HIGH);}
if (axis_fingers_neutral == true){digitalWrite(output_power, HIGH);;digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, HIGH);}


currentmillis = millis(); //записать текущее время с последней перезагрузки

//==================ПОЛУЧЕНИЕ СЫРЫХ ДАННЫХ С ДАТЧИКА ДАВЛЕНИЯ==================
current_mA = ina219.getCurrent_mA(); //получение значения тока на модуде INA219
current_mA = constrain (current_mA, 0.5, 30); //ПРОГРАММНОЕ ограничение значения тока от 0.5мА до 30мА для упрощения расчётов
if (current_mA < 0.5){open_circuit = true;} //при значении ниже заданного, ставим флажок "ИСТИНА" на "обрыв цепи"
if (current_mA > 20){short_circuit = true;} //при значении выше заданного, ставим флажок "ИСТИНА" на "короткое замыкание"

//==================КОНВЕРТИРОВАНИЕ ДАННЫХ С ФОРМАТА float В ФОРМАТ int==================
pressure_val = current_mA*100;

//==================КОНВЕРТИРОВАНИЕ СЫРЫХ ДАННЫХ В ФАКТИЧЕСКОЕ ДАВЛЕНИЕ ДЛЯ ВЫВОДА НА ДИСПЛЕЙ >>>!!!ОБЯЗАТЕЛЬНО ОТКАЛИБРОВАТЬ!!!<<<==================
pressure_actual = map(pressure_val, 400, 2000, 0, 250);

//==================СЧИТЫВАНИЕ ОСЕЙ==================





//======================корректировать движение с использованием реле 6шт и ШИМ 1 шт ==========================





//проверяем состояние концевиков и если они не замкнуты, то считываем значения джойстиков
if (axis_1_neutral == false){axis_1_val = analogRead(axis_1);}
    else {axis_1_val = 500;} //примерно нейтральное положение оси, значение потенциометра если джойстик не отклонён от центра
if (axis_2_neutral == false){axis_2_val = analogRead(axis_2);}
    else {axis_2_val = 500;} //примерно нейтральное положение оси, значение потенциометра если джойстик не отклонён от центра
if (axis_fingers_neutral == false){axis_fingers_val = analogRead(axis_fingers);}
    else {axis_fingers_val = 500;} //примерно нейтральное положение оси, значение потенциометра если джойстик не отклонён от центра
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

if (axis_1_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast1_up_relay, HIGH);digitalWrite(mast1_down_relay, HIGH);} //выключение вывода ШИМ и реле если ось в нейтрали
if (axis_2_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast2_up_relay, HIGH);digitalWrite(mast2_down_relay, HIGH);} //выключение вывода ШИМ и реле если ось в нейтрали
if (axis_fingers_neutral == true){digitalWrite(output_power, HIGH);;digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, HIGH);} //выключение вывода ШИМ и реле если ось в нейтрали




//Axis 1 center - switch is 1, goes to 0 at 510-512 moving up, 410 moving down
//Axis 2 center - switch is 1, goes to 0 at 546 moving up, 452 moving down
//Axis 3 center - switch is 1, goes to 0 at 542 moving up, 461 moving down

//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ 1==================

    if (axis_1_val >= 517){
        axis_1_out = map(axis_1_val, 517, 737, 0, 255);
        axis_1_up = true;
        axis_1_down = false;
        }
        
    if (axis_1_val <= 476){
        axis_1_out = map(axis_1_val, 256, 476, 255, 0);
        axis_1_up = false;
        axis_1_down = true;
        }
    if (axis_1_val > 476 && axis_1_val <= 517){axis_1_up = false, axis_1_down = false, axis_1_out = 0;
    }    

//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ 2==================

    if (axis_2_val >= 520){
        axis_2_out = map(axis_2_val, 520, 765, 0, 255);
        axis_2_up = true;
        axis_2_down = false;
        }
        
    if (axis_2_val <= 480){
        axis_2_out = map(axis_2_val, 256, 480, 255, 0);
        axis_2_up = false;
        axis_2_down = true;
        }
    if (axis_2_val > 480 && axis_2_val <= 520){axis_2_up = false, axis_2_down = false, axis_2_out = 0;
    }

//==================ОБРАБОТКА ЗНАЧЕНИЙ ОСИ "пальцы"==================

    if (axis_fingers_val >= 520){
        axis_fingers_out = map(axis_fingers_val, 520, 765, 0, 255);
        axis_fingers_close = true;
        axis_fingers_open = false;
        }
        
    if (axis_fingers_val <= 480){
        axis_fingers_out = map(axis_fingers_val, 256, 480, 255, 0);
        axis_fingers_close = false;
        axis_fingers_open = true;
        }
    if (axis_fingers_val > 480 && axis_fingers_val <= 520){axis_fingers_close = false, axis_fingers_open = false, axis_fingers_out = 0;
    }

//ВКЛЮЧЕНИЕ СООТВЕТСТВУЮЩИХ РЕЛЕ И ШИМ ВЫХОДОВ
if (axis_1_neutral == true && lock_1 == false){digitalWrite(output_power, HIGH);digitalWrite(mast1_up_relay, HIGH);digitalWrite(mast1_down_relay, HIGH);}
if (axis_2_neutral == true && lock_2 == false){digitalWrite(output_power, HIGH);digitalWrite(mast2_up_relay, HIGH);digitalWrite(mast2_down_relay, HIGH);}
if (axis_fingers_neutral == true && lock_3 == false){digitalWrite(output_power, HIGH);digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, HIGH);}

if (axis_1_up == true && lock_1 == false){analogWrite(output_power, axis_1_out);digitalWrite(mast1_up_relay, LOW);digitalWrite(mast1_down_relay, HIGH);}
if (axis_2_up == true && lock_2 == false){analogWrite(output_power, axis_2_out);digitalWrite(mast2_up_relay, LOW);digitalWrite(mast2_down_relay, HIGH);}
if (axis_fingers_open == true && lock_3 == false){digitalWrite(output_power, axis_fingers_out);digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, LOW);}

if (axis_1_down == true && lock_1 == false){analogWrite(output_power, axis_1_out);digitalWrite(mast1_up_relay, HIGH);digitalWrite(mast1_down_relay, LOW);}
if (axis_2_down == true && lock_2 == false){analogWrite(output_power, axis_2_out);digitalWrite(mast2_up_relay, HIGH);digitalWrite(mast2_down_relay, LOW);}
if (axis_fingers_close == true && lock_3 == false){digitalWrite(output_power, axis_fingers_out);digitalWrite(fingers_open_relay, LOW);digitalWrite(fingers_close_relay, HIGH);}

//ВЫВОД ЗНАЧЕНИЯ ДАВЛЕНИЯ НА ДИСПЛЕЙ
    if (currentmillis - previousmillis >= interval) { //если прошло больше времени чем интервал...
        if (open_circuit == true || short_circuit == true) //если "обрыв цепи" ИЛИ "короткое замыкание"...
        {
            if (open_circuit == true) //если "обрыв цепи" ИСТИНА
            {
            tm1637.clearDisplay(); //...то очистить дисплей...
            tm1637.displayByte(_dash, _O, _C, _dash); //...вывести "_OC_" на дисплей...
            previousmillis = currentmillis; //...и обновить таймер
            }
            else if (short_circuit == true) //иначе если "короткое замыкание" ИСТИНА
            {
            tm1637.clearDisplay(); //...то очистить дисплей...
            tm1637.displayByte(_dash, _S, _C, _dash); //...вывести "_SC_" на дисплей...
            previousmillis = currentmillis; //...и обновить таймер    
            }    
        }        
        
        else
        {
        tm1637.clearDisplay(); //...то очистить дисплей...
        tm1637.displayInt(pressure_actual); //...вывести значение на дисплей...
        previousmillis = currentmillis; //...и обновить таймер
        }
    }


/*
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
*/
}
