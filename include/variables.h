#pragma once

#include <math.h>

// Enum состояний системы (верхний уровень КА)
enum SYSTEM_STATES_LINK
{
    SS_MAIN_MENU_LINK,     // Ожидание команды запуска от ППО
    SS_ANGLE_DESIRED_LINK, // Полуавтомат. желаемые углы
    SS_START_POSING_LINK,  // Перевод звеньев в положение по умолчанию
    SS_EXE_DEMO_LINK,      // Упражнение, перемещение левого бедра
    SS_SEDENTARY_LINK,     // Сидячее положение
    SS_WALK_1_LINK,        // Походка
    SS_WALK_2_LINK,
    SS_HMN_GAIT_LINK
};

SYSTEM_STATES_LINK system_state = SS_MAIN_MENU_LINK;
int system_state_last = -1;

// Если сменилось состояние системы, вернет единицу на 1 цикл
bool isSSChangeLink()
{
    if (system_state != system_state_last)
    {
        system_state_last = system_state;
        return true;
    }
    else
        return false;
}

float deg2rad(float angle)
{
    float new_angle = angle * (PI / 180);
    return new_angle;
}

float rad2deg(float angle)
{
    float new_angle = angle * (180 / PI);
    return new_angle;
}

float lyambda = 0.99;          // Коэффициент показывающий насколько нога будет прямая
float angle_des = deg2rad(15); // Желаемый угол
float lengthHip = 0.4;         // Длина бедра
float lengthKnee = 0.4;        // Длина голени
float ls = 0.2;
float hs = 0.1;
const int hmn_gt_time = 5; // Пар-тр для 3 походки(время шага)
float hmn_gt_scaling = 0.9;
float hmn_gt_z_offset = -0.79;

float x_ankle_start_des; // Желаемая точка по иксу
float y_ankle_start_des; // Желаемая точка по игрику
float x_ankle_start;
float y_ankle_start;
float step_x;
float step_y;
float _x_targ;
float _y_targ;
float q1_des;
float q21_des;
float x_offset;
float y_offset;
float dt = 0.0159;
float t;
int k = 0;
const int hmn_gt_size = hmn_gt_time * 83;
unsigned long last_time_hmn_gait;

void getApproxHmnGt(float &x_aprox, float &y_aprox, float x)
{

    static float w, a0, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6, a7, b7, a8, b8;

    a0 = 0.07011;
    a1 = -0.0411;
    b1 = 0.05987;
    a2 = -0.03011;
    b2 = -0.02541;
    a3 = 0.01418;
    b3 = 0.01037;
    a4 = -0.001549;
    b4 = -0.000431;
    a5 = -0.001278;
    b5 = 0.001983;
    a6 = 0.001459;
    b6 = -0.0003046;
    a7 = 0.0004606;
    b7 = -0.0002769;
    a8 = -0.0004648;
    b8 = 0.0003664;
    w = 4.772;

    y_aprox = a0 + a1 * cos(x * w) + b1 * sin(x * w) + a2 * cos(2 * x * w) + b2 * sin(2 * x * w) + a3 * cos(3 * x * w) + b3 * sin(3 * x * w) + a4 * cos(4 * x * w) + b4 * sin(4 * x * w) + a5 * cos(5 * x * w) + b5 * sin(5 * x * w) + a6 * cos(6 * x * w) + b6 * sin(6 * x * w) + a7 * cos(7 * x * w) + b7 * sin(7 * x * w) + a8 * cos(8 * x * w) + b8 * sin(8 * x * w);

    a0 = -0.08484;
    a1 = -0.09385;
    b1 = -0.3012;
    a2 = -0.01739;
    b2 = 0.02488;
    a3 = 0.005448;
    b3 = 0.009218;
    a4 = -0.008137;
    b4 = 0.006326;
    a5 = 0.002729;
    b5 = 0.004278;
    a6 = 0.001207;
    b6 = -0.0002039;
    a7 = 0.0005637;
    b7 = -0.001207;
    a8 = -0.0004928;
    b8 = 0.0004946;
    w = 4.772;

    x_aprox = a0 + a1 * cos(x * w) + b1 * sin(x * w) + a2 * cos(2 * x * w) + b2 * sin(2 * x * w) + a3 * cos(3 * x * w) + b3 * sin(3 * x * w) + a4 * cos(4 * x * w) + b4 * sin(4 * x * w) + a5 * cos(5 * x * w) + b5 * sin(5 * x * w) + a6 * cos(6 * x * w) + b6 * sin(6 * x * w) + a7 * cos(7 * x * w) + b7 * sin(7 * x * w) + a8 * cos(8 * x * w) + b8 * sin(8 * x * w);
}

//.......................................................................................................................
// регулировки

// Enum состояний системы (верхний уровень КА)
enum SYSTEM_STATES_ADJ{
    SS_MAIN_MENU_ADJ,   // Главное меню
    SS_POSITION_DESIRED_ADJ,
    SS_ZEROING_ADJ,     // Обнуление энкодеров
    SS_DEMO_ADJ
  };
  
  // Текущее состояние системы
  SYSTEM_STATES_ADJ system_state_adj = SS_MAIN_MENU_ADJ;
  int system_state_last_adj = -1;
  
  // Если сменилось состояние системы, вернет единицу на 1 цикл
  bool isSSChangeAdj(){
    if(system_state_adj != system_state_last_adj){
        system_state_last_adj = system_state_adj;
        return true;
    }
    else return false;  
  }
  
  // Состояния обнуления регулировок с ДУП - энкодером (второй уровень КА)
  enum ZEROING_STATES{
    ZS_HIP_LEFT,   // Поиск нуля ДУП левого бедра
    ZS_KNEE_LEFT,  // Поиск нуля ДУП левого колена
    ZS_HIP_RIGHT,  // Поиск нуля ДУП правого бедра
    ZS_KNEE_RIGHT, // Поиск нуля ДУП правого колена
  };
  
  // Текущее состояние обнуления
  ZEROING_STATES zeroing_state = ZS_HIP_LEFT;
  int zeroing_state_last = -1;
  
  // Если сменилось состояние обнуление, вернет единицу на 1 цикл
  bool isZSChange(){
    if(zeroing_state != zeroing_state_last){
      zeroing_state_last = zeroing_state;
      return true;
    }
    else return false;  
  }
  
  // Флаг, поднимаемый, если прошла процедура инициализации
  bool is_zeroed_flag = false; 
  
  enum DEMO_STATES{
    DS_HIP_LEFT,
    DS_KNEE_LEFT,
    DS_FOOT_LEFT,
    DS_HIP_RIGHT,
    DS_KNEE_RIGHT,
    DS_FOOT_RIGHT,
    DS_HIP_LEFT_BACK,
    DS_KNEE_LEFT_BACK,
    DS_HIP_RIGHT_BACK,
    DS_KNEE_RIGHT_BACK
  };
  
  DEMO_STATES demo_state = DS_HIP_RIGHT;
  int demo_state_last = -1;
  
  bool isDSChange(){
    if(demo_state != demo_state_last){
      demo_state_last = demo_state;
      return true;
    }
    else return false;  
  }