#pragma once

// Класс управления двигателями (наследует GyverPID)
class DriverControllerMaxon : public GyverPID{
    public:
        DriverControllerMaxon(int pin_enb, int pin_dir, int pin_pwm, float kp, float ki, float kd, int dt_ms, DEMS22A &encoderObj) :
        GyverPID(kp, ki, kd, dt_ms), _encoderObj(encoderObj){
            // Локализируем переменные с номерами пинов
            _pin_enb = pin_enb;
            _pin_dir = pin_dir;
            _pin_pwm = pin_pwm;

            // Настраиваем режим работы пинов
            pinMode(_pin_enb, OUTPUT);
            pinMode(_pin_dir, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            // Задаем граничные значения регулятора
            setLimits(_OUTPUT_LIMIT_MIN, _OUTPUT_LIMIT_MAX);
        };

        void manualControl(float _voltage){
            int _pwm = constrain(abs(_voltage), 0, 24) * (_OUTPUT_LIMIT_MAX / 24);
            if(_voltage > 0){
                digitalWrite(_pin_dir, LOW);
                _pwm = map(_pwm, 0, 255, 24, 229);
            } else if(_voltage < 0) {
               digitalWrite(_pin_dir, HIGH);
                _pwm = map(_pwm, -1, -255, 24, 229);
            } else{
                _pwm = 24;
            }
            digitalWrite(_pin_enb, true);
            analogWrite(_pin_pwm, _pwm);
        }

        // Функция управления приводом
        void control(float set_point){
            // Передаем регулятору текущее и заданные положения
            setpoint = set_point;
            input = _encoderObj.getPositionDeg();
            // Выходное значение регулятора
            int _output;

            // Считаем ошибку
            float _error = setpoint - input;
            
            // Если мы вне допуска, в зависимости от знака ошибки
            if(_error > _ALLOW_ERROR){
                // Меняем флаг направления вращения
                digitalWrite(_pin_dir, LOW);
                // Получаем от регулятора и масштабируем ШИМ (убираем по 10% от краев, ограничение драйвера)
                // 20 - подобрано во время демонстрации. 10% - это 26, но с таким ШИМом драйвер продолжает медленно перемещаться
                // Нужно экспериментально уменьшить диапазон до меньшего значения
                _output = map(getResultTimer(), 0, 255, 24, 229);
                // Опускаем флаг успешного позиционирования
                _is_positioned = false;
            }else if(_error < -1 * _ALLOW_ERROR){
                digitalWrite(_pin_dir, HIGH);
                _output = map(getResultTimer(), -1, -255, 24, 229);
                _is_positioned = false;
            }
            // Если мы находимся в допуске
            else{
                // Выдаем на драйвер минимальный ШИМ (от меньшего может упасть в ошибку и отключиться)
                _output = 24;
                // Поднимаем флаг успешного позиционирования
                _is_positioned = true; 
            }
            // Генерируем ШИМ
            analogWrite(_pin_pwm, _output);  
        }

        // Включает/выключает питание привода
        void setEnable(bool enable){
            digitalWrite(_pin_enb, enable);  
        }

        // Возвращает единицу, если привод находится в допуске
        bool isPositioned(){
            return _is_positioned;
        }

    private:
        // Граничные значения регулятора
        const int _OUTPUT_LIMIT_MIN = -255;
        const int _OUTPUT_LIMIT_MAX = 255;
        // Максимальная допустимая ошибка (~3 тика в градусах)
        const int _ALLOW_ERROR = 1.06;
        // Флаг, поднимаемый при попадании привода в допуск
        bool _is_positioned;
        // Пины управления драйвером двигателя
        int _pin_enb;
        int _pin_dir;
        int _pin_pwm;
        // Ссылка на объект энкодера
        DEMS22A &_encoderObj;
};

// Период обновления регулятора
const int PID_DRIVER_DT = 10;

// Калибровка по отдельности каждого звена, не актуальна после снижения напряжения питания до 24В
// при включеном на драйверах моментной контуре

// // Коэффициенты регулятора приводов бедра
// const float HIP_DRIVER_KP = 50.0;
// const float HIP_DRIVER_KI = 2.0;
// const float HIP_DRIVER_KD = 1.4;
// // Коэффициенты регулятора приводов колена
// const float KNEE_DRIVER_KP = 60.0;
// const float KNEE_DRIVER_KI = 2.0;
// const float KNEE_DRIVER_KD = 1.4;
// // Коэффициенты регулятора приводов стопы
// const float FOOT_DRIVER_KP = 40.0;
// const float FOOT_DRIVER_KI = 2.0;
// const float FOOT_DRIVER_KD = 1.4;

// ТУДУ ПОДОБРАНЫ НА ГЛАЗ ПРИ 24В ПИТАНИЯ
// Коэффициенты регулятора приводов бедра
const float HIP_DRIVER_KP = 20.0;
const float HIP_DRIVER_KI = 0.0;
const float HIP_DRIVER_KD = 5.0;
// Коэффициенты регулятора приводов колена
const float KNEE_DRIVER_KP = 15.0;
const float KNEE_DRIVER_KI = 0.0;
const float KNEE_DRIVER_KD = 5.0;
// Коэффициенты регулятора приводов стопы
const float FOOT_DRIVER_KP = 13.0;
const float FOOT_DRIVER_KI = 0.0;
const float FOOT_DRIVER_KD = 0.0;

// Создаем объекты регуляторов
DriverControllerMaxon driverHipLeftLink(PIN_ENB_HIP_LEFT, PIN_DIR_HIP_LEFT, PIN_PWM_HIP_LEFT,
HIP_DRIVER_KP, HIP_DRIVER_KI, HIP_DRIVER_KD, PID_DRIVER_DT, encodHipLeftLink);
DriverControllerMaxon driverKneeLeftLink(PIN_ENB_KNEE_LEFT, PIN_DIR_KNEE_LEFT, PIN_PWM_KNEE_LEFT,
KNEE_DRIVER_KP, KNEE_DRIVER_KI, KNEE_DRIVER_KD, PID_DRIVER_DT, encodKneeLeftLink);
DriverControllerMaxon driverFootLeftLink(PIN_ENB_FOOT_LEFT, PIN_DIR_FOOT_LEFT, PIN_PWM_FOOT_LEFT,
FOOT_DRIVER_KP, FOOT_DRIVER_KI, FOOT_DRIVER_KD, PID_DRIVER_DT, encodFootLeftLink);
DriverControllerMaxon driverHipRightLink(PIN_ENB_HIP_RIGHT, PIN_DIR_HIP_RIGHT, PIN_PWM_HIP_RIGHT,
HIP_DRIVER_KP, HIP_DRIVER_KI, HIP_DRIVER_KD, PID_DRIVER_DT, encodHipRightLink);
DriverControllerMaxon driverKneeRightLink(PIN_ENB_KNEE_RIGHT, PIN_DIR_KNEE_RIGHT, PIN_PWM_KNEE_RIGHT,
KNEE_DRIVER_KP, KNEE_DRIVER_KI, KNEE_DRIVER_KD, PID_DRIVER_DT, encodKneeRightLink);
DriverControllerMaxon driverFootRightLink(PIN_ENB_FOOT_RIGHT, PIN_DIR_FOOT_RIGHT, PIN_PWM_FOOT_RIGHT,
FOOT_DRIVER_KP, FOOT_DRIVER_KI, FOOT_DRIVER_KD, PID_DRIVER_DT, encodFootRightLink);

//.................................................................................................................................
//регулировки

// Класс для управления приводами регулировок с обратной связью по энкодеру
class DriverControllerEncod : public GyverPID{
    public:
        DriverControllerEncod(int pin_in_a, int pin_in_b, int pin_pwm, float kp, float ki, float kd, int dt,
        DOptoEncoder &encoderObj) : GyverPID(kp, ki, kd, dt), _encoderObj(encoderObj){
            // Локализируем переменные с номерами пинов
            _pin_in_a = pin_in_a;
            _pin_in_b = pin_in_b;
            _pin_pwm = pin_pwm;

            // Настраиваем пины
            pinMode(_pin_in_a, OUTPUT);
            pinMode(_pin_in_b, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            // Задаем ограничения регулятору
            setLimits(_OUTPUT_LIMIT_MIN, _OUTPUT_LIMIT_MAX);
        };

        // Ручное управление приводами, принимает значение напряжения с учетом знака 
        void manualControl(float _voltage){
            int _pwm = constrain(abs(_voltage), 0, MOTOR_VOLTAGE_LIMIT) * (_OUTPUT_LIMIT_MAX / MOTOR_VOLTAGE_LIMIT);

            // Управляем направлением вращения двигателя
            if(_voltage > 0){
                digitalWrite(_pin_in_a, LOW);
                digitalWrite(_pin_in_b, HIGH);
                _encoderObj.setVector(0);
            } else if(_voltage < 0) {
                digitalWrite(_pin_in_a, HIGH);
                digitalWrite(_pin_in_b, LOW);
                _encoderObj.setVector(1);
            } else{
                digitalWrite(_pin_in_a, LOW);
                digitalWrite(_pin_in_b, LOW);
            }

            // Подаем ШИМ на драйвер
            analogWrite(_pin_pwm, _pwm);
        }

        // Функция регулятора, перемещает привод в переданное положение
        void control(float set_point){
            // Передаем значения переменных регулятору
            setpoint = set_point;
            input = _encoderObj.getPosition();
            // Считаем ошибку
            float _error = (setpoint - input) * 10;

            // Если ошибка вне допуска, устраняем ее
            if(_error > _ALLOW_ERROR){
                digitalWrite(_pin_in_a, LOW);
                digitalWrite(_pin_in_b, HIGH);
                analogWrite(_pin_pwm, abs(getResultTimer()));
                // Переключаем вектор для расчета положения
                _encoderObj.setVector(1);
                // Опускаем флаг попадания в допуск
                _is_positioned = false; 
            } else if(_error < -1 * _ALLOW_ERROR){
                digitalWrite(_pin_in_a, HIGH);
                digitalWrite(_pin_in_b, LOW);
                analogWrite(_pin_pwm, abs(getResultTimer()));  
                _encoderObj.setVector(0);
                _is_positioned = false;  
            } else{
                // Если ошибка в допуске, отключаем приводы, поднимаем флаг
                digitalWrite(_pin_in_a, LOW);
                digitalWrite(_pin_in_b, LOW);
                _is_positioned = true; 
            }
        }

        // Возвращает единицу, если привод находится в допуске
        bool isPositioned(){
            return _is_positioned;
        }

    private:
        // Максимальное подаваемое напряжение на двигатели
        const float MOTOR_VOLTAGE_LIMIT = 12.0;
        // Граничные значения регулятора (255/2, т.к. силовое питание = 24В, а напряжение двигателя - 12В)
        const int _OUTPUT_LIMIT_MAX = 127;
        const int _OUTPUT_LIMIT_MIN = -1 * _OUTPUT_LIMIT_MAX;
        // Максимальная допустимая ошибка (16 тика - 3 мм => 1мм ~ 6 тиков)
        const int _ALLOW_ERROR = 6;
        // Флаг, поднимаемый при попадании привода в допуск
        bool _is_positioned;
        // Пины управления драйвером двигателя
        int _pin_in_a;
        int _pin_in_b;
        int _pin_pwm;
        // Ссылка на объект энкодера
        DOptoEncoder &_encoderObj;
};

// Подкласс для управления приводами регулировок с обратной связью по потенциометру
class DriverControllerPoten : public GyverPID{
    public:
        // Используем базовый конструктор класса + передаем указатель на функцию опроса потенциометра
        DriverControllerPoten(int pin_dir, int pin_pwm, float kp, float ki, float kd, int dt, int (*potenFunk)()) :
        GyverPID(kp, ki, kd, dt), _potenFunk(potenFunk){
            // Локализируем переменные с номерами пинов
            _pin_dir = pin_dir;
            _pin_pwm = pin_pwm;

            // Настраиваем пины
            pinMode(_pin_dir, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            // Задаем ограничения регулятору
            setLimits(_OUTPUT_LIMIT_MIN, _OUTPUT_LIMIT_MAX);
        };

        // Ручное управление приводами, принимает значение напряжения с учетом знака 
        void manualControl(float _voltage){
            int _pwm = constrain(abs(_voltage), 0, MOTOR_VOLTAGE_LIMIT) * (_OUTPUT_LIMIT_MAX / MOTOR_VOLTAGE_LIMIT);

            // Управляем направлением вращения двигателя
            if(_voltage > 0)        digitalWrite(_pin_dir, LOW);
            else if(_voltage < 0)   digitalWrite(_pin_dir, HIGH);
            
            // Подаем ШИМ на драйвер
            analogWrite(_pin_pwm, _pwm);
        }
        
        // Функция регулятора, перемещает привод в переданное положение
        void control(float set_point){
            // Передаем значения переменных регулятору
            setpoint = set_point;
            input = _potenFunk();
            // Считаем ошибку
            float _error = setpoint - input;

            // Если ошибка вне допуска, устраняем ее
            if(_error > _ALLOW_ERROR){
                digitalWrite(_pin_dir, LOW);
                analogWrite(_pin_pwm, abs(getResultTimer()));
                // Опускаем флаг попадания в допуск
                _is_positioned = false; 
            } else if(_error < -1 * _ALLOW_ERROR){
                digitalWrite(_pin_dir, HIGH);
                analogWrite(_pin_pwm, abs(getResultTimer()));  
                _is_positioned = false;  
            } else{
                // Если ошибка в допуске, поднимаем флаг
                _is_positioned = true; 
            }
        }

        // Возвращает единицу, если привод находится в допуске
        bool isPositioned(){
            return _is_positioned;
        }

    private:
        // Максимальное подаваемое напряжение на двигатели
        const float MOTOR_VOLTAGE_LIMIT = 12.0;
        // Граничные значения регулятора (255/2, т.к. силовое питание = 24В, а напряжение двигателя - 12В)
        const int _OUTPUT_LIMIT_MAX = 127;
        const int _OUTPUT_LIMIT_MIN = -1 * _OUTPUT_LIMIT_MAX;
        // Максимальная допустимая ошибка (1024 АЦП - 10 об. => 102 - 3мм => 1мм ~ 30 АЦП)
        const int _ALLOW_ERROR = 30;
        // Флаг, поднимаемый при попадании привода в допуск
        bool _is_positioned;
        // Пины управления драйвером двигателя
        int _pin_dir;
        int _pin_pwm;
        // Указатель на функцию опроса потенциометра
        int (*_potenFunk)();
};

// Настройки регуляторов двигателей
const float DRIVER_ENCOD_KP = 5.0;
const float DRIVER_ENCOD_KI = 24.3; 
const float DRIVER_ENCOD_KD = 90.0;

const float DRIVER_POTEN_KP = 0.4;
const float DRIVER_POTEN_KI = 0.1;
const float DRIVER_POTEN_KD = 0;

const int DRIVER_DT = 10;

// Создаем объекты двигателей регулировок
DriverControllerEncod driverHipLeftAdj(PIN_DRIVER_HIP_LEFT_INA, PIN_DRIVER_HIP_LEFT_INB, PIN_DRIVER_HIP_LEFT_PWM, DRIVER_ENCOD_KP, DRIVER_ENCOD_KI, DRIVER_ENCOD_KD, DRIVER_DT, encodHipLeftAdj);

DriverControllerEncod driverKneeLeftAdj(PIN_DRIVER_KNEE_LEFT_INA, PIN_DRIVER_KNEE_LEFT_INB, PIN_DRIVER_KNEE_LEFT_PWM,
DRIVER_ENCOD_KP, DRIVER_ENCOD_KI, DRIVER_ENCOD_KD, DRIVER_DT, encodKneeLeftAdj);

DriverControllerPoten driverFootLeftAdj(PIN_DRIVER_FOOT_LEFT_DIR, PIN_DRIVER_FOOT_LEFT_PWM,
DRIVER_POTEN_KP, DRIVER_POTEN_KI, DRIVER_POTEN_KD, DRIVER_DT, &getAngleFootLeftADC);

DriverControllerEncod driverHipRightAdj(PIN_DRIVER_HIP_RIGHT_INA, PIN_DRIVER_HIP_RIGHT_INB, PIN_DRIVER_HIP_RIGHT_PWM,
DRIVER_ENCOD_KP, DRIVER_ENCOD_KI, DRIVER_ENCOD_KD, DRIVER_DT, encodHipRightAdj);

DriverControllerEncod driverKneeRightAdj(PIN_DRIVER_KNEE_RIGHT_INA, PIN_DRIVER_KNEE_RIGHT_INB, PIN_DRIVER_KNEE_RIGHT_PWM,
DRIVER_ENCOD_KP, DRIVER_ENCOD_KI, DRIVER_ENCOD_KD, DRIVER_DT, encodKneeRightAdj);

DriverControllerPoten driverFootRightAdj(PIN_DRIVER_FOOT_RIGHT_DIR, PIN_DRIVER_FOOT_RIGHT_PWM,
DRIVER_POTEN_KP, DRIVER_POTEN_KI, DRIVER_POTEN_KD, DRIVER_DT, &getAngleFootRightADC);