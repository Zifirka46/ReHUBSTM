#pragma once

#include <DEMS22A.h>
#include "DHX711.h"

//создание объектов энекодеров звеньев
DEMS22A encodHipLeftLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_HIP_LEFT);
DEMS22A encodKneeLeftLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_KNEE_LEFT);
DEMS22A encodFootLeftLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_FOOT_LEFT);
DEMS22A encodHipRightLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_HIP_RIGHT);
DEMS22A encodKneeRightLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_KNEE_RIGHT);
DEMS22A encodFootRightLink(PIN_BURNS_CLOCK, PIN_BURNS_DATA, PIN_CS_FOOT_RIGHT);

// Для правых энкодров умножаем на -1
// Задаем нули энкодеров
void setupEncod(){
    encodHipLeftLink.setOffset(-102);
    encodKneeLeftLink.setOffset(-492);
    encodFootLeftLink.setOffset(-445);
    encodHipRightLink.setOffset(-539);
    encodKneeRightLink.setOffset(-336);
    encodFootRightLink.setOffset(-506);
}

// Обновляем показания энкодеров
void updateEncod(){
    encodHipLeftLink.update();
    encodKneeLeftLink.update();
    encodFootLeftLink.update();
    encodHipRightLink.update();
    encodKneeRightLink.update();
    encodFootRightLink.update();
}

// Коэффициенты АЦП силомоментных датчиков
const float SCALE_HIP_LEFT_COEF = 207.0;
const float SCALE_KNEE_LEFT_COEF = 361.0;
const float SCALE_FOOT1_LEFT_COEF = 45.0;
const float SCALE_FOOT2_LEFT_COEF = 49.0;
const float SCALE_FOOT3_LEFT_COEF = 45.0;
const float SCALE_HIP_RIGHT_COEF = 315.0;
const float SCALE_KNEE_RIGHT_COEF = 197.0;
const float SCALE_FOOT1_RIGHT_COEF = 46.0;
const float SCALE_FOOT2_RIGHT_COEF = 38.0;
const float SCALE_FOOT3_RIGHT_COEF = 45.0;

// Отключены, так как без подключенных датчиков вешают МК при запуске

// Создаем объекты для работы с АЦП СМИ
// DHX711 scaleHipLeft(PIN_TENZO_HIP_LEFT_DT, PIN_TENZO_HIP_LEFT_SCK, SCALE_HIP_LEFT_sCOEF);
// DHX711 scaleKneeLeft(PIN_TENZO_KNEE_LEFT_DT, PIN_TENZO_KNEE_LEFT_SCK, SCALE_KNEE_LEFT_COEF);
// DHX711 scaleFootLeft1(PIN_TENZO_FOOT_LEFT_DT1, PIN_TENZO_FOOT_LEFT_SCK1, SCALE_FOOT1_LEFT_COEF);
// DHX711 scaleFootLeft2(PIN_TENZO_FOOT_LEFT_DT2, PIN_TENZO_FOOT_LEFT_SCK2, SCALE_FOOT2_LEFT_COEF);
// DHX711 scaleFootLeft3(PIN_TENZO_FOOT_LEFT_DT3, PIN_TENZO_FOOT_LEFT_SCK3, SCALE_FOOT3_LEFT_COEF);
// DHX711 scaleHipRight(PIN_TENZO_HIP_RIGHT_DT, PIN_TENZO_HIP_RIGHT_SCK, SCALE_HIP_RIGHT_COEF);
// DHX711 scaleKneeRight(PIN_TENZO_KNEE_RIGHT_DT, PIN_TENZO_KNEE_RIGHT_SCK, SCALE_KNEE_RIGHT_COEF);
// DHX711 scaleFootRight1(PIN_TENZO_FOOT_RIGHT_DT1, PIN_TENZO_FOOT_RIGHT_SCK1, SCALE_FOOT1_RIGHT_COEF);
// DHX711 scaleFootRight2(PIN_TENZO_FOOT_RIGHT_DT3, PIN_TENZO_FOOT_RIGHT_SCK2, SCALE_FOOT2_RIGHT_COEF);
// DHX711 scaleFootRight3(PIN_TENZO_FOOT_RIGHT_DT3, PIN_TENZO_FOOT_RIGHT_SCK3, SCALE_FOOT3_RIGHT_COEF);

//..........................................................................................................
//регулировки

// Вектор сигнала соответствует вектору перемещения, сигналы НЕ инвертированы относительно друг друга
const int ANGLE_FOOT_LEFT_MAX = 670;  // Минимальное значение ДУП стопы
const int ANGLE_FOOT_LEFT_MIN = 335;  // Максимальное значение ДУП стопы

// Положение ДУП левой стопы [АЦП]
int getAngleFootLeftADC(){
  static Median_filter<int, 3> _med;
  static RA_filter<int, 8> _ra;
  return _ra.filtered(_med.filtered(analogRead(PIN_ANGLE_FOOT_LEFT)));
}

const int ANGLE_FOOT_RIGHT_MAX = 740;
const int ANGLE_FOOT_RIGHT_MIN = 400;

// Положение ДУП правой стопы [АЦП]
int getAngleFootRightADC(){
  static Median_filter<int, 3> _med;
  static RA_filter<int, 8> _ra;
  return _ra.filtered(_med.filtered(analogRead(PIN_ANGLE_FOOT_RIGHT)));
}

// Создаем объекты энкодеров, создаем функции для вызова обработчика прерывания.
// Внутри класса функцию attachInterrupt() вызывать нельзя, тк она использует статичные ссылки

// Объект энкодера левого бедра
DOptoEncoder encodHipLeftAdj(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT));

// Функция обработчика прерывания левого бедра
void encodHipLeftHandler(){ encodHipLeftAdj.update(); }

// Объект энкодера левой голени
DOptoEncoder encodKneeLeftAdj(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT));

// Функция обработчика прерывания левой голени
void encodKneeLeftHandler(){ encodKneeLeftAdj.update(); }

// Объект энкодера правого бедра
DOptoEncoder encodHipRightAdj(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT));

// Функция обработчика прерывания правого бедра
void encodHipRightHandler(){ encodHipRightAdj.update(); }

// Объект энкодера правой голени
DOptoEncoder encodKneeRightAdj(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT));

// Функция обработчика прерывания правой голени
void encodKneeRightHandler(){ encodKneeRightAdj.update(); }

// Задаем коэффициент перевода единиц измерения энкодеров
void setEncodScales(){
  // Коэффициент перевода тиков в мм линейного измерения
  // 16 - число срабатываний на оборот, 1.1 - передаточное ДУП, 3 - шаг ШВП
  const float _ENCOD_COEF = (1.0 / 16.0) * (3.0 / 1.1);

  // Задаем сдвиг каждому объекту энкодера
  encodHipLeftAdj.setScaleCoef(_ENCOD_COEF);
  encodKneeLeftAdj.setScaleCoef(_ENCOD_COEF);
  encodHipRightAdj.setScaleCoef(_ENCOD_COEF);
  encodKneeRightAdj.setScaleCoef(_ENCOD_COEF);
};
