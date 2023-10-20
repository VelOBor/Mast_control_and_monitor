/*
Разработано Олегом "Salty NUggeTZ" Величко и Игорем "Terrano"
Разработано с целью управления и мониторинга системы неизвестного происхождения и с неизвестными спецификациями
Гипотетический прототип разработан в Fritzing и собран на макетной плате
Система построена на платформе Arduino (NANO)
*/

//====================================БИБЛИОТЕКИ====================================

//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) СЛЕДУЮЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<
//#include <Arduino.h> //библиотека Arduino для использования в VSCode PlatformIO (НЕ НУЖНА ЕСЛИ ПИСАТЬ И/ИЛИ ПРОШИВАТЬ ИЗ ARDUINO IDE!!!)
//>>>УДАЛИТЬ (ИЛИ ЗАКОММЕНТИРОВАТЬ //) ПРЕДЫДУЩУЮ 1 СТРОКУ ПРИ ОБРАБОТКЕ ВНЕ СРЕДЫ VSCode Platformio<<<

#include <Wire.h> //библиотека I2C
#include <INA219_WE.h> //библиотека INA автора Wolfgang Ewald
#include <TM1637.h> //библиотека TM1637 с жёсткогого диска, находится в папке .pio (можно скачать так же с (https://drive.google.com/file/d/1DN5pDko7D1-F4POXICIfd8m_1sAKBZt7/view)

#define I2C_ADDRESS 0x40 //адрес I2C модуля INA219
//====================================КОНЕЦ БИБЛИОТЕК====================================



//====================================ОПРЕДЕЛЕНИЯ ПИНОВ====================================
//----------входы----------

//датчик давления подключён к модулю INA219 который будет инициализирован ниже, пины А4 и А5
int valve_test_pin = A7;    //ВХОД тестовой цепи реле

int axis_1 = A0;            //ВХОД потенциометр джойстика оси 1
int axis_2 = A1;            //ВХОД потенциометр джойстика оси 2
int axis_fingers = A2;      //ВХОД потенциометр джойстика оси "пальцы"

int neutral_switch_1 = 8;   //ВХОД концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_2 = 9;   //ВХОД концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ
int neutral_switch_3 = 10;  //ВХОД концевик нейтрального положения джойстика, в нейтральном положении концевик РАЗОМКНУТ

//----------выходы----------
int test_relay = A3;            //ВЫХОД, реле замыкается (низким уровнем сигнала) только для проверки сопротивления (напряжения) на клапанах, совместно с реле управления

int mast1_up_relay = 2;         //ВЫХОД реле подъёма первой секции мачты
int mast1_down_relay = 3;       //ВЫХОД реле спуска первой секции мачты

int mast2_up_relay = 4;         //ВЫХОД реле подъёма второй секции мачты
int mast2_down_relay = 5;       //ВЫХОД реле спуска второй секции мачты

int fingers_close_relay = 6;    //ВЫХОД реле закрытия пальцев
int fingers_open_relay = 7;     //ВЫХОД реле открытия пальцев

int output_power = 11;          //ВЫХОД ШИМ скорости двигателя

//----------модуль TM1637-----------
int display_clk = 12; //CLK пин модуля TM1637
int display_dio = 13; //DIO пин модуля TM1637

//----------модуль INA219----------
//A4 = SDA I2C INA219
//A5 = SCL I2C INA219
//====================================КОНЕЦ ОПРЕДЕЛЕНИЯ ПИНОВ====================================



//====================================ПЕРЕМЕННЫЕ====================================

//----------состояние осей----------
int neutral_switch_1_state = 0;     //состояние концевика оси 1
int neutral_switch_2_state = 0;     //состояние концевика оси 2
int neutral_switch_3_state = 0;     //состояние концевика оси "пальцы"

int axis_1_val = 0;         //сырое значение состояния оси 1
int axis_1_out = 0;         //переформатированное значение оси 1 для вывода ШИМ

int axis_2_val = 0;         //сырое значение состояния оси 2
int axis_2_out = 0;         //переформатированное значение оси 2 для вывода ШИМ

int axis_fingers_val = 0;   //сырое значение состояния оси "пальцы"
int axis_fingers_out = 0;   //переформатированное значение оси "пальцы" для вывода ШИМ

int pressure_val = 0;       //переформатированное значение float с модуля INA219 в int для последующей обработки
int pressure_actual = 0;    //фактическое давление в bar (атмосфер)

//----------значение тока на INA219 (давление)----------
float current_mA = 0.0;         //float для считывания данных с модуля INA219


  
//==========переменные для тестирования клапанов==========
//----------значение напряжения на А7 (самотестирование клапанов)----------
float valve_test_value = 0.0;   //float для проверки сопротивления (напряжения) на клапанах на время тестирования через тестовое реле и вход А7, valve_test_pin

//----------пределы значений напряжения на клапанах----------
//если напряжение ниже 0.857 - UxSC (короткое замыкание), если выше 0.888 то UxOC (обрыв цепи)
float valve_sc_limit = 0.860;   //нижний предел напряжения на тестируемом клапане
float valve_oc_limit = 0.880;   //верхний предел напряжения на тестируемом клапане
//==========конец переменных для тестирования клапанов==========

//----------паузы и тайминг----------
int delay_test_disp = 2000;     //пауза на время вывода tESt на дисплей
int delay_u_disp = 2000;        //пауза на время вывода номера клапана на дисплей
int delay_err_disp = 2000;      //пауза на время вывода результата проверки на дисплей
int delay_done_disp = 2000;     //пауза на время вывода dOnE на дисплей
int delay_redy_disp = 3000;     //пауза на время вывода rEdY на дисплей
int delay_boot = 1000;          //пауза на время загрузки (стабилизация напряжения не AREF)
int delay_short = 500;          //пауза на время считывания и т.д.

//ВНИМАНИЕ!!! ВОЗМОЖНО БУДУТ КОСЯКИ С ДИСПЛЕЕМ ПОСЛЕ 50 ДНЕЙ БЕЗ ПЕРЕЗАГРУЗКИ!!!
unsigned long previousmillis = 0;   //когда было последнее обновление дисплея
unsigned long currentmillis = 0;    //текущее время в миллисекундах
const long interval = 100;          //интервал между обновлениями дисплея в миллисекундах
//====================================КОНЕЦ ПЕРЕМЕННЫХ====================================


//====================================ФЛАЖКИ====================================

//----------Флажки ошибок----------
bool open_circuit = false; //обрыв цепи
bool short_circuit = false; //короткое замыкание

//----------флажки состояния осей----------
bool axis_1_neutral = true;     //флаг нейтрали оси 1
bool axis_1_up = true;          //флаг ВВЕРХ оси 1
bool axis_1_down = true;        //флаг ВНИЗ оси 1

bool axis_2_neutral = true;     //флаг нейтрали оси 2
bool axis_2_up = true;          //флаг ВВЕРХ оси 2
bool axis_2_down = true;        //флаг ВНИЗ оси 2

bool axis_fingers_neutral = true;   //флаг нейтрали оси "пальцы"
bool axis_fingers_close = true;     //флаг ВВЕРХ (ЗАКРЫТЬ) оси "пальцы"
bool axis_fingers_open = true;      //флаг ВНИЗ (ОТКРЫТЬ) оси "пальцы"

//----------флажки блокировки осей----------
bool lock_1 = false;                //флаг блокировки оси 1
bool lock_2 = false;                //флаг блокировки оси 2
bool lock_3 = false;                //флаг блокировки оси "пальцы"
//====================================КОНЕЦ ФЛАЖКОВ====================================



//==================СОЗДАНИЕ ОБЪЕКТОВ БИБЛИОТЕК==================
INA219_WE ina219 = INA219_WE(I2C_ADDRESS); //создание объекта "ina219" библиотеки INA219_WE, инициализация датчика давления
TM1637 tm1637(display_clk, display_dio); //создание объекта "tm1637" библиотеки TM1637, инициализация дисплея
//==================КОНЕЦ СОЗДАНИЯ ОБЪЕКТОВ БИБЛИОТЕК==================



//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//================НАСТРОЙКИ, выполняется разово при включении МК================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//

void setup() 
{
Serial.begin(9600);         //инициализируем последовательный протокол, удалить строку после завершения написания и проверки и перепроверки работоспособности системы
Wire.begin();               //инициализуруем интерфейс I2C

tm1637.init();              //инициализируем модуль tm1637
tm1637.set(BRIGHT_TYPICAL); //устанавливаем яркость дисплея



//----------определения режимов работы пинов входов----------
pinMode(axis_1, INPUT);                     //вертикальная ось 1, потенциометр, вход
pinMode(neutral_switch_1, INPUT_PULLUP);    //концевик нейтрали вертикальной оси 1, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_2, INPUT);                     //вертикальная ось 2, потенциометр, вход
pinMode(neutral_switch_2, INPUT_PULLUP);    //концевик нейтрали вертикальной оси 2, !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!

pinMode(axis_fingers, INPUT);               //ось "пальцы", потенциометр, вход
pinMode(neutral_switch_3, INPUT_PULLUP);    //концевик нейтрали оси "пальцы", !!!РАЗОМКНУТ КОНЦЕВИК == НЕЙТРАЛЬНОЕ ПОЛОЖЕНИЕ!!!



//----------определения режимов работы пинов выходов----------
pinMode(test_relay, OUTPUT);            //реле тестирования клапанов

pinMode(mast1_up_relay, OUTPUT);        //мачта элемент 1 выход на реле ВВЕРХ
pinMode(mast1_down_relay, OUTPUT);      //мачта элемент 1 выход на реле ВНИЗ

pinMode(mast2_up_relay, OUTPUT);        //мачта элемент 2 выход на реле ВВЕРХ
pinMode(mast2_down_relay, OUTPUT);      //мачта элемент 2 выход на реле ВНИЗ

pinMode(fingers_open_relay, OUTPUT);    //пальцы выход на реле ВЫДВИНУТЬ
pinMode(fingers_close_relay, OUTPUT);   //пальцы выход на реле ЗАДВИНУТЬ

pinMode(display_clk, OUTPUT); //выход на пин CLK модуля tm1637
pinMode(display_dio, OUTPUT); // выход на пин DIO модуля tm1637



//----------размыкание всех реле----------
//реле управления клапанами РАЗМЫКАЮТСЯ на ВЫСОКОМ уровне И замыкаются на низком, ТЕСТОВОЕ РЕЛЕ НАОБОРОТ!!!
digitalWrite(test_relay, LOW);              //установить состояние пинов в низкий логический уровень

digitalWrite(mast1_up_relay, HIGH);         //установить состояние пинов в высокий логический уровень
digitalWrite(mast1_down_relay, HIGH);       //установить состояние пинов в высокий логический уровень

digitalWrite(mast2_up_relay, HIGH);         //установить состояние пинов в высокий логический уровень
digitalWrite(mast2_down_relay, HIGH);       //установить состояние пинов в высокий логический уровень

digitalWrite(fingers_open_relay, HIGH);     //установить состояние пинов в высокий логический уровень
digitalWrite(fingers_close_relay, HIGH);    //установить состояние пинов в высокий логический уровень

//----------сброс статуса блокировки на ЛОЖЬ для всех трёх осей---------- ПРОВЕРИТЬ НЕОБХОДИМОСТЬ! ВОЗМОЖНО УДАЛИТЬ ЕСЛИ РАБОТАЮТ КОНЦЕВИКИ!!!
lock_1 = false; //блокировка оси 1
lock_2 = false; //блокировка оси 2
lock_3 = false; //блокировка оси "пальцы"



//проверить исправвность клапанов путём подачи 1.1В на aref и на входе А7 значение должно быть в диапазоне 0.857 до 0.888, вывести на дисплэй U1on, U2on, U3on - U6on, если ошибка - err
//если напряжение ниже 0.857 - U1SC (короткое замыкание), если выше 0.888 то U1OC (обрыв цепи) 
//для проверки напряжения вкл Д1, Д2, Д3... Д6, подаём на них ареф 1.1В, считываем на А7

//----------проверка работоспособности модуля ТМ----------
tm1637.displayByte(_d, _i, _S, _P);
delay(delay_test_disp);
tm1637.displayByte(_t, _E, _S, _t);
delay(delay_test_disp);

tm1637.displayByte(_dash, _dash, _dash, _8);
delay(delay_short);
tm1637.displayByte(_dash, _dash, _8, _dash);
delay(delay_short);
tm1637.displayByte(_dash, _8, _dash, _dash);
delay(delay_short);
tm1637.displayByte(_8, _dash, _dash, _dash);
delay(delay_short);

tm1637.displayByte(_dash, _O, _C, _dash);
delay(delay_err_disp);
tm1637.displayByte(_dash, _S, _C, _dash);
delay(delay_err_disp);

tm1637.displayByte(_d, _i, _S, _P);
delay(delay_test_disp);
tm1637.displayByte(_t, _E, _S, _t);
delay(delay_test_disp);
tm1637.displayByte(_D, _O, _N, _E);
delay(delay_done_disp);
//----------конец проверки работоспособности модуля ТМ----------


  
//==================ПРОВЕРКА НАПРЯЖЕНИЯ НА КЛАПАНАХ 1-6==================
analogReference(INTERNAL);  //ставим опорное напряжение на 1.1В для ардуино нано
delay(delay_boot);          //пауза для стабилизации напряжения на AREF



//==========проверка клапана 1==========
tm1637.displayByte(_t, _E, _S, _t);         //отображаем teSt на дисплее
delay(delay_test_disp);                     //пауза
tm1637.displayByte(_dash, _U, _1, _dash);   //отображаем U1 на дисплее, напряжение на клапане 1
delay(delay_u_disp);                        //пауза

//----------замыкание реле----------
digitalWrite(test_relay, HIGH);             //замыкаем тестовое реле
digitalWrite(mast1_up_relay, LOW);          //замыкаем реле клапана 1
delay(delay_u_disp);                        //пауза

//----------считывание и обработка значения----------
valve_test_value = map(analogRead(valve_test_pin), 0, 1023, 0, 1.100);  //считываем значение АЦП на тестовом пине, и переводим его в фактическое напряжение
Serial.print(" U1 = "); Serial.println(valve_test_value);               //выводим значение напряжения на серийный порт
delay(delay_err_disp);                                                  //пауза

//----------логика----------
    if (valve_test_value < valve_oc_limit)      //проверяем если значение менне чем нижний предел, то...
    {
        tm1637.displayByte(_U, _1, _O, _C);     //выводим на дисплей ошибку "клапан 1 обрыв цепи"
        delay(delay_err_disp);                  //пауза
    }
    
    else if(valve_test_value > valve_sc_limit)  //проверяем если значение более чем верхний предел, то
    {
        tm1637.displayByte(_U, _1, _S, _C);     //выводим на дисплей ошибку "клапан 1 короткое замыкание"
        delay(delay_err_disp);                  //пауза
    }
    
    else                                        //если иначе, то по логике вещей значение между нижним и верхним пределом, и выводим
    {
        tm1637.displayByte(_U, _1, _o, _n);     //выводим на дисплей "клапан 1 включен"
        delay(delay_err_disp);                  //пауза
    }

//----------размыкание реле----------
//digitalWrite(test_relay, LOW);        //размыкаем тестовое реле
//digitalWrite(mast1_up_relay, HIGH);   //размыкаем реле клапана 1
//==========конец проверки клапана 1==========
  
//----------подготовка к старту МК----------
analogReference(DEFAULT);   //установка опорного напряжения на значение по умолчанию, 5В
delay(delay_boot);          //пауза для стабилизации напряжения на AREF



tm1637.displayByte(_t, _E, _S, _t);
delay(delay_test_disp);

tm1637.displayByte(_U, _1, _U, _6);
delay(delay_err_disp);

tm1637.displayByte(_D, _O, _N, _E);
delay(delay_test_disp);

tm1637.displayByte(_r, _E, _d, _Y);
delay(delay_redy_disp);
//==================КОНЕЦ ПРОВЕРКИ НАПРЯЖЕНИЯ НА КЛАПАНАХ 1-6==================

}
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||============КОНЕЦ НАСТРОЕК============||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//



//ЗАМЕТКА - центральное положение джойстиков 512, добавить гистерезис +-10, скорректировать после подключения фактических джойстиков
//ЗАМЕТКА - нажатие джойстиков ВВЕРХ активирует реле "ВВЕРХ", а на оси "пальцы" выполняет "ЗАКРЫТИЕ", возможна корректировка логики работы после подключения фактических джойстиков
//Axis 1 center 504 (493) +-, max 737, min 256
//Axis 2 center 510 +-, max 765, min 256
//Axis 3 center 510 +-, max 764, min 256
//Axis 1 center - switch is 1, goes to 0 at 510-512 moving up, 410 moving down
//Axis 2 center - switch is 1, goes to 0 at 546 moving up, 452 moving down
//Axis 3 center - switch is 1, goes to 0 at 542 moving up, 461 moving down



//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||============ОСНОВНОЙ ЦИКЛ=============||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//
//==================||==================||==================||==================//

void loop() 

//==================||ВРЕМЕННО ОТКЛЮЧЁН ОСНОВНОЙ ЦИКЛ||==================//
/*  <<<УДАЛИТЬ ВСЮ ЭТУ СТРОЧКУ ЧТОБЫ ВКЛЮЧИТЬ ОСНОВНОЙ ЦИКЛ (СМОТРЕТЬ В КОНЦЕ ЦИКЛА ТОЖЕ!!!)

{
tm1637.clearDisplay(); //очистить дисплей

//выключить ШИМ выход и разомкнуть все реле если ось в нейтрали
//turned off for testing
//if (axis_1_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast1_up_relay, HIGH);digitalWrite(mast1_down_relay, HIGH);}
//if (axis_2_neutral == true){digitalWrite(output_power, HIGH);digitalWrite(mast2_up_relay, HIGH);digitalWrite(mast2_down_relay, HIGH);}
//if (axis_fingers_neutral == true){digitalWrite(output_power, HIGH);;digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, HIGH);}

//testing valve 1 voltage
digitalWrite(test_relay, HIGH); //замыкаем тестовое реле
digitalWrite(mast1_up_relay, LOW); //замыкаем реле клапана 1
valve_test_value = map(analogRead(valve_test_pin), 0, 1023, 0, 1.100); //считываем значение на тестовом пине
Serial.println(valve_test_value);

currentmillis = millis(); //записать текущее время с последней перезагрузки

//==================ПОЛУЧЕНИЕ СЫРЫХ ДАННЫХ С ДАТЧИКА ДАВЛЕНИЯ==================
current_mA = ina219.getCurrent_mA(); //получение значения тока на модуде INA219
current_mA = constrain (current_mA, 0.4, 30); //ПРОГРАММНОЕ ограничение значения тока от 0.5мА до 30мА для упрощения расчётов
if (current_mA < 0.4){open_circuit = true;} //при значении ниже заданного, ставим флажок "ИСТИНА" на "обрыв цепи"
if (current_mA > 20){short_circuit = true;} //при значении выше заданного, ставим флажок "ИСТИНА" на "короткое замыкание"

//==================КОНВЕРТИРОВАНИЕ ДАННЫХ С ФОРМАТА float В ФОРМАТ int==================
pressure_val = current_mA*100;

//==================КОНВЕРТИРОВАНИЕ СЫРЫХ ДАННЫХ В ФАКТИЧЕСКОЕ ДАВЛЕНИЕ ДЛЯ ВЫВОДА НА ДИСПЛЕЙ >>>!!!ОБЯЗАТЕЛЬНО ОТКАЛИБРОВАТЬ!!!<<<==================
pressure_actual = map(pressure_val, 400, 2000, 0, 250);

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

//==================СЧИТЫВАНИЕ ОСЕЙ==================

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
if (axis_fingers_close == true && lock_3 == false){digitalWrite(output_power, axis_fingers_out);digitalWrite(fingers_open_relay, HIGH);digitalWrite(fingers_close_relay, LOW);}


===============================================================================================




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


//==================||ВРЕМЕННО ОТКЛЮЧЁН ОСНОВНОЙ ЦИКЛ||==================//
*/  //<<<УДАЛИТЬ ВСЮ ЭТУ СТРОЧКУ ЧТОБЫ ВКЛЮЧИТЬ ОСНОВНОЙ ЦИКЛ (СМОТРЕТЬ В НАЧАЛЕ ЦИКЛА ТОЖЕ!!!)