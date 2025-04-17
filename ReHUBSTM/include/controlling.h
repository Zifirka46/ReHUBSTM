#pragma once

#include <math.h>

// Парсим данные откуда-нибудь
void splitString(String data, char separator, String *dest, int maxDestCount, int &destCount)
{
    destCount = 0;
    int startIndex = 0;
    int index = 0;
    while ((index = data.indexOf(separator, startIndex)) != -1 && destCount < maxDestCount)
    {
        dest[destCount++] = data.substring(startIndex, index);
        startIndex = index + 1;
    }
    if (startIndex < data.length() && destCount < maxDestCount)
    {
        dest[destCount++] = data.substring(startIndex);
    }
}

struct Kinematics
{
    float A1x;
    float A1y;
    float A2x;
    float A2y;
    float A3x;
    float A3y;
};

Kinematics directKinematics(float lengthHip, float lengthKnee)
{
    Kinematics result;
    result.A1x = 0;
    result.A1y = 0;

    float q1 = -1 * deg2rad(encodHipRightLink.getPositionDeg());
    float q2 = -1 * deg2rad(encodKneeRightLink.getPositionDeg());

    result.A2x = result.A1x + lengthHip * sin(q1);
    result.A2y = result.A1y - lengthHip * cos(q1);

    result.A3x = result.A2x + lengthKnee * sin(q1 + q2);
    result.A3y = result.A2y - lengthKnee * cos(q1 + q2);

    return result;
}

struct AxisForWalk1
{
    float x_set = 0;
    float y_set = 0;
};

AxisForWalk1 trajectoryPlannerForWalk1(float stepLength, float stepHeight)
{
    AxisForWalk1 axis;

    static Polynomial_3 __poly_3_X[4];
    static Polynomial_3 __poly_3_Y[2];

    static int mode = 0;
    static float Vsx = 1.5 * (2 * stepLength) / 3000;

    switch (mode)
    {
    case 0:
    {
        __poly_3_X[0].calculate(0, -stepLength / 2.0, -Vsx, 0, 0, 750);
        __poly_3_X[1].calculate(-stepLength / 2.0, 0, 0, Vsx, 750, 1500);
        __poly_3_X[2].calculate(0, stepLength / 2.0, Vsx, 0, 1500, 2250);
        __poly_3_X[3].calculate(stepLength / 2, 0, 0, -Vsx, 2250, 3000);

        __poly_3_Y[0].calculate(0, stepHeight, 0, 0, 750, 1500);
        __poly_3_Y[1].calculate(stepHeight, 0, 0, 0, 1500, 2250);

        mode = 1;
    }
    break;

    case 1:
    {
        axis.x_set = __poly_3_X[0].getPosition();
        if (__poly_3_X[0].isFinished())
        {
            axis.x_set = __poly_3_X[1].getPosition();
            axis.y_set = __poly_3_Y[0].getPosition();
        }
        if (__poly_3_X[1].isFinished() && __poly_3_Y[0].isFinished())
        {
            axis.x_set = __poly_3_X[2].getPosition();
            axis.y_set = __poly_3_Y[1].getPosition();
        }
        if (__poly_3_X[2].isFinished())
        {
            axis.x_set = __poly_3_X[3].getPosition();
        }
        if (__poly_3_X[3].isFinished())
        {
            mode = 0;
        }
    }
    break;
    }
    return axis;
}

struct AxisForWalk2
{
    float x_set = 0;
    float y_set = 0;
};

AxisForWalk2 trajectoryPlannerForWalk2(float stepHeight)
{
    AxisForWalk2 axis;

    static Polynomial_3 __poly_3_X[2];
    static Polynomial_3 __poly_3_Y[2];
    static Polynomial_3 __poly_3_q;

    static float _x_ankle_start = lyambda * (lengthHip + lengthKnee) * sin(angle_des);
    static float _y_ankle_start = -lyambda * (lengthHip + lengthKnee) * cos(angle_des);

    static float Vsx = 1.5 * (2 * _x_ankle_start) / 2000;

    static int mode = 0;

    axis.x_set = _x_ankle_start;
    axis.y_set = _y_ankle_start;

    switch (mode)
    {
    case 0:
    {
        __poly_3_q.calculate(angle_des, -angle_des, 0, 0, 0, 1500);

        __poly_3_X[0].calculate(-_x_ankle_start, 0, 0, Vsx, 1500, 2500);
        __poly_3_X[1].calculate(0, _x_ankle_start, Vsx, 0, 2500, 3500);

        __poly_3_Y[0].calculate(_y_ankle_start, _y_ankle_start + stepHeight, 0, 0, 1500, 2500);
        __poly_3_Y[1].calculate(_y_ankle_start + stepHeight, _y_ankle_start, 0, 0, 2500, 3500);

        mode = 1;
    }
    break;

    case 1:
    {
        // t > 0 && t < 1
        axis.x_set = lyambda * (lengthHip + lengthKnee) * sin(__poly_3_q.getPosition());
        axis.y_set = -lyambda * (lengthHip + lengthKnee) * cos(__poly_3_q.getPosition());

        if (__poly_3_q.isFinished())
        {
            axis.x_set = __poly_3_X[0].getPosition();
            axis.y_set = __poly_3_Y[0].getPosition();
        }
        if (__poly_3_X[0].isFinished() && __poly_3_Y[0].isFinished())
        {
            axis.x_set = __poly_3_X[1].getPosition();
            axis.y_set = __poly_3_Y[1].getPosition();
        }

        if (__poly_3_X[1].isFinished() && __poly_3_Y[1].isFinished())
        {
            mode = 0;
        }
    }
    break;
    }
    return axis;
}

struct Angle
{
    float q1 = 0;
    float q21 = 0;
};

Angle inverseKinematics(float x_des, float z_des, float x_start, float z_start, float l1, float l2, float q1, float q21)
{
    Angle angle;
    angle.q1 = -(l2 + z_des * cos(q1 + q21) - z_start * cos(q1 + q21) - x_des * sin(q1 + q21) + x_start * sin(q1 + q21) + l1 * cos(q21) - l1 * q1 * sin(q21)) / (l1 * sin(q21));
    angle.q21 = q21 + ((l2 * cos(q1 + q21) + l1 * cos(q1)) * (z_des - z_start + l2 * cos(q1 + q21) + l1 * cos(q1))) / (l1 * l2 * sin(q21)) + ((l2 * sin(q1 + q21) + l1 * sin(q1)) * (x_start - x_des + l2 * sin(q1 + q21) + l1 * sin(q1))) / (l1 * l2 * sin(q21));

    if (angle.q1 > deg2rad(85))
    {
        angle.q1 = deg2rad(85);
    }
    if (angle.q1 < deg2rad(-15))
    {
        angle.q1 = deg2rad(-15);
    }
    if (angle.q21 < deg2rad(-90))
    {
        angle.q21 = deg2rad(-90);
    }
    if (angle.q21 > deg2rad(-3))
    {
        angle.q21 = deg2rad(-3);
    }
    return angle;
}

// Функция для выбора привода и скорости
void setMotorSpeedLink(int motorNumber, int speed)
{
    switch (motorNumber)
    {
    case 1:
        driverHipLeftLink.manualControlLink(speed);
        break;
    case 2:
        driverKneeLeftLink.manualControlLink(speed);
        break;
    case 3:
        driverFootLeftLink.manualControlLink(speed);
        break;
    case 4:
        driverHipRightLink.manualControlLink(-speed);
        break;
    case 5:
        driverKneeRightLink.manualControlLink(-speed);
        break;
    case 6:
        driverFootRightLink.manualControlLink(-speed);
        break;
    }
}

// Функция для остановки всех приводов
void stopAllMotorsLink()
{
    driverHipLeftLink.manualControlLink(0);
    driverKneeLeftLink.manualControlLink(0);
    driverFootLeftLink.manualControlLink(0);
    driverHipRightLink.manualControlLink(0);
    driverKneeRightLink.manualControlLink(0);
    driverFootRightLink.manualControlLink(0);
}

// Функция для режима полуавтоматического управления углами
void setAngleDesired(int motorNumber, float angle, float time)
{
    static Polynomial_3 __poly_hip_left;
    static Polynomial_3 __poly_knee_left;
    static Polynomial_3 __poly_foot_left;
    static Polynomial_3 __poly_hip_right;
    static Polynomial_3 __poly_knee_right;
    static Polynomial_3 __poly_foot_right;

    static float _hip_left_set_pos_deg;
    static float _knee_left_set_pos_deg;
    static float _foot_left_set_pos_deg;
    static float _hip_right_set_pos_deg;
    static float _knee_right_set_pos_deg;
    static float _foot_right_set_pos_deg;

    static int mode = 0;
    switch (motorNumber)
    {
    case 1:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_hip_left.calculate(encodHipLeftLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _hip_left_set_pos_deg = __poly_hip_left.getPosition();
            driverHipLeftLink.control(_hip_left_set_pos_deg);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_left.isFinished())
            {
                driverHipLeftLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case 2:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _knee_left_set_pos_deg = __poly_knee_left.getPosition();
            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.control(_knee_left_set_pos_deg);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_left.isFinished())
            {
                driverKneeLeftLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case 3:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_foot_left.calculate(encodFootLeftLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _foot_left_set_pos_deg = __poly_foot_left.getPosition();
            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.control(_foot_left_set_pos_deg);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_foot_left.isFinished())
            {
                driverFootLeftLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case 4:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();
            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                driverHipRightLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case 5:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();
            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_right.isFinished())
            {
                driverKneeRightLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case 6:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_foot_right.calculate(encodFootRightLink.getPositionDeg(), angle, time);
            mode = 1;
        }
        break;

        case 1:
        {
            _foot_right_set_pos_deg = __poly_foot_right.getPosition();
            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.control(_foot_right_set_pos_deg);

            if (__poly_foot_right.isFinished())
            {
                driverFootRightLink.manualControlLink(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;
    }
    // Включаем питание приводов
    driverHipLeftLink.setEnable(true);
    driverKneeLeftLink.setEnable(true);
    driverFootLeftLink.setEnable(true);
    driverHipRightLink.setEnable(true);
    driverKneeRightLink.setEnable(true);
    driverFootRightLink.setEnable(true);
}

// Функция защиты датчиков ступней от выхода за диапазон
void angleFootProtection()
{
    // Допустимый выход за диапазон
    const int _ALLOW_ERROR = 25;
    // Обновляем значения углов
    int _left_angle = getAngleFootLeftADC();
    int _right_angle = getAngleFootRightADC();

    // Если вышли за допустимый диапазон
    if (_left_angle <= ANGLE_FOOT_LEFT_MIN - _ALLOW_ERROR || _left_angle >= ANGLE_FOOT_LEFT_MAX + _ALLOW_ERROR ||
        _right_angle <= ANGLE_FOOT_RIGHT_MIN - _ALLOW_ERROR || _right_angle >= ANGLE_FOOT_RIGHT_MAX + _ALLOW_ERROR)
    {
        // Размыкаем реле силового питания
        digitalWrite(PIN_RELAY_ADJ, LOW);
    }
}

void setMotorSpeedAdj(int motorNumber, int speed)
{
    switch (motorNumber)
    {
    case 1:
        attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT), encodHipLeftHandler, CHANGE);
        driverHipLeftAdj.manualControlAdj(speed);
        break;
    case 2:
        attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT), encodKneeLeftHandler, CHANGE);
        driverKneeLeftAdj.manualControlAdj(speed);
        break;
    case 3:
        driverFootLeftAdj.manualControl(speed);
        break;
    case 4:
        attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT), encodHipRightHandler, CHANGE);
        driverHipRightAdj.manualControlAdj(speed);
        break;
    case 5:
        attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT), encodKneeRightHandler, CHANGE);
        driverKneeRightAdj.manualControlAdj(speed);
        break;
    case 6:
        driverFootRightAdj.manualControl(speed);
        break;
    }
}

void stopAllMotors()
{
    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT));
    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT));
    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT));
    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT));

    driverHipLeftAdj.manualControlAdj(0);
    driverKneeLeftAdj.manualControlAdj(0);
    driverFootLeftAdj.manualControl(0);
    driverHipRightAdj.manualControlAdj(0);
    driverKneeRightAdj.manualControlAdj(0);
    driverFootRightAdj.manualControl(0);
}

void SetPositionDesired(int linkNumber, float length, float time)
{
    static Polynomial_3 __poly_hip_left_adj;
    static Polynomial_3 __poly_knee_left_adj;
    static Polynomial_3 __poly_foot_left_adj;
    static Polynomial_3 __poly_hip_right_adj;
    static Polynomial_3 __poly_knee_right_adj;
    static Polynomial_3 __poly_foot_right_adj;

    static float _hip_left_set_pos_len;
    static float _knee_left_set_pos_len;
    static float _foot_left_set_pos_len;
    static float _hip_right_set_pos_len;
    static float _knee_right_set_pos_len;
    static float _foot_right_set_pos_len;

    static int mode = 0;

    switch (linkNumber)
    {
    case 1:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_hip_left_adj.calculate(encodHipLeftAdj.getPosition(), -1 * (length - 360), time * 1000);
            mode = 1;
        }
        break;

        case 1:
        {
            attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT), encodHipLeftHandler, CHANGE);
            _hip_left_set_pos_len = __poly_hip_left_adj.getPosition();
            driverHipLeftAdj.control(_hip_left_set_pos_len);
            driverKneeLeftAdj.manualControlAdj(0);
            driverFootLeftAdj.manualControl(0);
            driverHipRightAdj.manualControlAdj(0);
            driverKneeRightAdj.manualControlAdj(0);
            driverFootRightAdj.manualControl(0);

            if (__poly_hip_left_adj.isFinished())
            {
                driverHipLeftAdj.manualControlAdj(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
            Serial.print(encodHipLeftAdj.getPosition());
            Serial.print(" , ");
            Serial.println(_hip_left_set_pos_len);
        }
        break;
        }
    }
    break;

    case 2:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_knee_left_adj.calculate(encodKneeLeftAdj.getPosition(), -1 * (length - 360), time * 1000);
            mode = 1;
        }
        break;

        case 1:
        {
            attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT), encodKneeLeftHandler, CHANGE);
            _knee_left_set_pos_len = __poly_knee_left_adj.getPosition();
            driverHipLeftAdj.manualControlAdj(0);
            driverKneeLeftAdj.control(_knee_left_set_pos_len);
            driverFootLeftAdj.manualControl(0);
            driverHipRightAdj.manualControlAdj(0);
            driverKneeRightAdj.manualControlAdj(0);
            driverFootRightAdj.manualControl(0);

            if (__poly_knee_left_adj.isFinished())
            {
                driverKneeLeftAdj.manualControlAdj(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
            Serial.print(encodKneeLeftAdj.getPosition());
            Serial.print(" , ");
            Serial.println(_knee_left_set_pos_len);
        }
        break;
        }
    }
    break;

        // case 3:{
        //     switch(mode){
        //         case 0:{
        //             __poly_foot_left.calculate(encodFootLeft.getPosition(), Length, time);
        //             mode = 1;
        //         } break;

        //         case 1:{
        //             _foot_left_set_pos_deg = __poly_foot_left.getPosition();
        //             driverHipLeft.manualControl(0);
        //             driverKneeLeft.manualControl(0);
        //             driverFootLeft.control(_foot_left_set_pos_deg);
        //             driverHipRight.manualControl(0);
        //             driverKneeRight.manualControl(0);
        //             driverFootRight.manualControl(0);

        //             if(__poly_foot_left.isFinished()){
        //                 driverFootLeft.manualControl(0);
        //                 mode = 0;
        //                 system_state = SS_MAIN_MENU;
        //             }
        //         } break;
        //     }
        // } break;

    case 4:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_hip_right_adj.calculate(encodHipRightAdj.getPosition(), -1 * (length - 360), time * 1000);
            mode = 1;
        }
        break;

        case 1:
        {
            attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT), encodHipRightHandler, CHANGE);
            _hip_right_set_pos_len = __poly_hip_right_adj.getPosition();
            driverHipLeftAdj.manualControlAdj(0);
            driverKneeLeftAdj.manualControlAdj(0);
            driverFootLeftAdj.manualControl(0);
            driverHipRightAdj.control(_hip_right_set_pos_len);
            driverKneeRightAdj.manualControlAdj(0);
            driverFootRightAdj.manualControl(0);

            if (__poly_hip_right_adj.isFinished())
            {
                driverHipRightAdj.manualControlAdj(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
            Serial.print(encodHipRightAdj.getPosition());
            Serial.print(" , ");
            Serial.println(_hip_right_set_pos_len);
        }
        break;
        }
    }
    break;

    case 5:
    {
        switch (mode)
        {
        case 0:
        {
            __poly_knee_right_adj.calculate(encodKneeRightAdj.getPosition(), -1 * (length - 360), time * 1000);
            mode = 1;
        }
        break;

        case 1:
        {
            attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT), encodKneeRightHandler, CHANGE);
            _knee_right_set_pos_len = __poly_knee_right_adj.getPosition();
            driverHipLeftAdj.manualControlAdj(0);
            driverKneeLeftAdj.manualControlAdj(0);
            driverFootLeftAdj.manualControl(0);
            driverHipRightAdj.manualControlAdj(0);
            driverKneeRightAdj.control(_knee_right_set_pos_len);
            driverFootRightAdj.manualControl(0);

            if (__poly_knee_right_adj.isFinished())
            {
                driverKneeRightAdj.manualControlAdj(0);
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
            Serial.print(encodKneeRightAdj.getPosition());
            Serial.print(" , ");
            Serial.println(_knee_right_set_pos_len);
        }
        break;
        }
    }
    break;

        // case 6:{
        //     switch(mode){
        //         case 0:{
        //             __poly_foot_right.calculate(encodFootRight.getPositionDeg(), angle, time);
        //             mode = 1;
        //         } break;

        //         case 1:{
        //             _foot_right_set_pos_deg = __poly_foot_right.getPosition();
        //             driverHipLeft.manualControl(0);
        //             driverKneeLeft.manualControl(0);
        //             driverFootLeft.manualControl(0);
        //             driverHipRight.manualControl(0);
        //             driverKneeRight.manualControl(0);
        //             driverFootRight.control(_foot_right_set_pos_deg);

        //             if(__poly_foot_right.isFinished()){
        //                 driverFootRight.manualControl(0);
        //                 mode = 0;
        //                 system_state = SS_MAIN_MENU;
        //             }
        //         } break;
        //     }
        // } break;
    }
}

void mainControl()
{
    // Заданные положения звеньев
    static float _hip_left_set_pos_deg;
    static float _knee_left_set_pos_deg;
    static float _foot_left_set_pos_deg;
    static float _hip_right_set_pos_deg;
    static float _knee_right_set_pos_deg;
    static float _foot_right_set_pos_deg;

    static float _hip_left_set_pos_deg_adj;
    static float _knee_left_set_pos_deg_adj;
    static float _foot_left_set_pos_deg_adj;
    static float _hip_right_set_pos_deg_adj;
    static float _knee_right_set_pos_deg_adj;
    static float _foot_right_set_pos_deg_adj;

    static int motorNumber;
    static int linkNumber;
    static int angle;
    static int length;
    static int time;

    switch (system_state)
    {
    case SS_MAIN_MENU:
    {
        if (isSSChange());
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.startsWith("ManLin"))
            {
                String params[7];
                int paramsCount = 0;
                int speed;
                int motorNumber;
                splitString(input, ';', params, 7, paramsCount);
                if (paramsCount > 1)
                {
                    for (int i = 1; i < paramsCount; i++)
                    {
                        String param = params[i];
                        if (param.length() > 0)
                        {
                            motorNumber = param.toInt();
                            speed = param.substring(param.indexOf(':') + 1).toInt();
                            if (input.startsWith("ManLinUp"))
                            {
                                setMotorSpeedLink(motorNumber, speed);
                            }
                            else if (input.startsWith("ManLinDown"))
                            {
                                setMotorSpeedLink(motorNumber, -speed);
                            }
                        }
                    }
                }
            }
            else if (input.startsWith("ManAdj"))
            {
                String params[7];
                int paramsCount = 0;
                int speed;
                int motorNumber;
                splitString(input, ';', params, 7, paramsCount);
                if (paramsCount > 1)
                {
                    for (int i = 1; i < paramsCount; i++)
                    {
                        String param = params[i];
                        if (param.length() > 0)
                        {
                            motorNumber = param.toInt();
                            speed = param.substring(param.indexOf(':') + 1).toInt();
                            if (input.startsWith("ManAdjUp"))
                                setMotorSpeedAdj(motorNumber, speed);
                            else if (input.startsWith("ManAdjDown"))
                                setMotorSpeedAdj(motorNumber, -speed);
                        }
                    }
                }
            }
            else if (input.startsWith("Angle"))
            {
                String params[7];
                int paramsCount = 0;
                splitString(input, ';', params, 7, paramsCount);
                if (paramsCount > 1)
                {
                    for (int i = 1; i < paramsCount; i++)
                    {
                        String param = params[i];
                        if (param.length() > 0)
                        {
                            motorNumber = param.substring(0, param.indexOf(':') + 1).toInt();
                            angle = param.substring(param.indexOf(':') + 1, param.lastIndexOf(':')).toInt();
                            time = param.substring(param.lastIndexOf(':') + 1).toInt();
                            system_state = SS_ANGLE_DESIRED_LINK;
                        }
                    }
                }
            }
            else if (input.equals("Stop"))
            {
                stopAllMotorsLink();
            }
            else if (input.equals("Init"))
            {
                system_state = SS_START_POSING_LINK;
            }
            else if (input.equals("DemoStart"))
            {
                system_state = SS_EXE_DEMO_LINK;
            }
            else if (input.equals("Sedentary"))
            {
                system_state = SS_SEDENTARY_LINK;
            }
            else if (input.equals("Walk1"))
            {
                system_state = SS_WALK_1_LINK;
            }
            else if (input.equals("Walk2"))
            {
                system_state = SS_WALK_2_LINK;
            }
            else if (input.equals("Walk3"))
            {
                system_state = SS_HMN_GAIT_LINK;
            }
        }
    }
    break;

    // Кейс для полуавтоматического управления звеньями
    case SS_ANGLE_DESIRED_LINK:
    {
        setAngleDesired(motorNumber, angle, time * 1000);
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
            }
        }
    }
    break;

    // Кейс для возвращения звеньев в стартовую позицию
    case SS_START_POSING_LINK:
    {
        // Скорость перемещения приводов к начальные положению [град./сек]
        const float __VELOCITY = 10.0;
        // Заданные начальные положения звеньев
        const float __HIP_START_POS_DEG = 0;
        const float __KNEE_START_POS_DEG = 0;
        const float __FOOT_START_POS_DEG = 0;

        // Инициализируем полиному для каждого звена
        static Polynomial_3 __poly_hip_left;
        static Polynomial_3 __poly_knee_left;
        static Polynomial_3 __poly_foot_left;
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        static Polynomial_3 __poly_foot_right;

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
            }
        }

        if (isSSChange())
        {
            // Рассчитываем продолжительность перемещения звеньев
            float __hip_left_posing_time = (abs(encodHipLeftLink.getPositionDeg() - __HIP_START_POS_DEG) / __VELOCITY) * 1000.0;
            float __knee_left_posing_time = (abs(encodKneeLeftLink.getPositionDeg() - __KNEE_START_POS_DEG) / __VELOCITY) * 1000.0;
            float __foot_left_posing_time = (abs(encodFootLeftLink.getPositionDeg() - __FOOT_START_POS_DEG) / __VELOCITY) * 1000.0;
            float __hip_right_posing_time = (abs(encodHipRightLink.getPositionDeg() - __HIP_START_POS_DEG) / __VELOCITY) * 1000.0;
            float __knee_right_posing_time = (abs(encodKneeRightLink.getPositionDeg() - __KNEE_START_POS_DEG) / __VELOCITY) * 1000.0;
            float __foot_right_posing_time = (abs(encodFootRightLink.getPositionDeg() - __FOOT_START_POS_DEG) / __VELOCITY) * 1000.0;

            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);

            // Рассчитываем коэффициенты полинома
            __poly_hip_left.calculate(encodHipLeftLink.getPositionDeg(), __HIP_START_POS_DEG, __hip_left_posing_time);
            __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), __KNEE_START_POS_DEG, __knee_left_posing_time);
            __poly_foot_left.calculate(encodFootLeftLink.getPositionDeg(), __FOOT_START_POS_DEG, __foot_left_posing_time);
            __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), -1 * __HIP_START_POS_DEG, __hip_right_posing_time);
            __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), -1 * __KNEE_START_POS_DEG, __knee_right_posing_time);
            __poly_foot_right.calculate(encodFootRightLink.getPositionDeg(), -1 * __FOOT_START_POS_DEG, __foot_right_posing_time);
        };

        // Обновляем заданное положение приводов
        _hip_left_set_pos_deg = __poly_hip_left.getPosition();
        _knee_left_set_pos_deg = __poly_knee_left.getPosition();
        _foot_left_set_pos_deg = __poly_foot_left.getPosition();
        _hip_right_set_pos_deg = __poly_hip_right.getPosition();
        _knee_right_set_pos_deg = __poly_knee_right.getPosition();
        _foot_right_set_pos_deg = __poly_foot_right.getPosition();

        driverHipLeftLink.control(_hip_left_set_pos_deg);
        driverKneeLeftLink.control(_knee_left_set_pos_deg);
        driverFootLeftLink.control(_foot_left_set_pos_deg);
        driverHipRightLink.control(_hip_right_set_pos_deg);
        driverKneeRightLink.control(_knee_right_set_pos_deg);
        driverFootRightLink.control(_foot_right_set_pos_deg);

        // Проверяем, заняли ли приводы начальное положение
        if (__poly_hip_left.isFinished() && __poly_knee_left.isFinished() && __poly_foot_left.isFinished() &&
            __poly_hip_right.isFinished() && __poly_knee_right.isFinished() && __poly_foot_right.isFinished())
        {
            system_state = SS_MAIN_MENU;
            stopAllMotorsLink();
        }

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
            }
        }
    }
    break;

    // Кейс для демо режима
    case SS_EXE_DEMO_LINK:
    {

        static int mode = 0;

        static Polynomial_3 __poly_hip_left;
        static Polynomial_3 __poly_knee_left;
        static Polynomial_3 __poly_foot_left;
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        static Polynomial_3 __poly_foot_right;

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                system_state = SS_MAIN_MENU;
                stopAllMotorsLink();
            }
        }

        if (isSSChange())
        {
            mode = 0;
            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);
        };

        switch (mode)
        {
        case 0:
        {
            __poly_hip_left.calculate(encodHipLeftLink.getPositionDeg(), 30, 3000);
            mode = 1;
        }
        break;

        case 1:
        {
            _hip_left_set_pos_deg = __poly_hip_left.getPosition();

            driverHipLeftLink.control(_hip_left_set_pos_deg);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), -30, 3000);
                mode = 2;
            }
        }
        break;

        case 2:
        {
            _knee_left_set_pos_deg = __poly_knee_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.control(_knee_left_set_pos_deg);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_foot_left.calculate(encodFootLeftLink.getPositionDeg(), -10, 2000);
                mode = 3;
            }
        }
        break;

        case 3:
        {
            _foot_left_set_pos_deg = __poly_foot_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.control(_foot_left_set_pos_deg);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_foot_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_foot_left.calculate(encodFootLeftLink.getPositionDeg(), 0, 2000);
                mode = 4;
            }
        }
        break;

        case 4:
        {
            _foot_left_set_pos_deg = __poly_foot_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.control(_foot_left_set_pos_deg);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_foot_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), 0, 3000);
                mode = 5;
            }
        }
        break;

        case 5:
        {
            _knee_left_set_pos_deg = __poly_knee_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.control(_knee_left_set_pos_deg);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_hip_left.calculate(encodHipLeftLink.getPositionDeg(), 0, 3000);
                mode = 6;
            }
        }
        break;

        case 6:
        {
            _hip_left_set_pos_deg = __poly_hip_left.getPosition();

            driverHipLeftLink.control(_hip_left_set_pos_deg);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_left.isFinished())
            {
                stopAllMotorsLink();
                __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), -30, 3000);
                mode = 7;
            }
        }
        break;

        case 7:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), 30, 3000);
                mode = 8;
            }
        }
        break;

        case 8:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_foot_right.calculate(encodFootRightLink.getPositionDeg(), 10, 2000);
                mode = 9;
            }
        }
        break;

        case 9:
        {
            _foot_right_set_pos_deg = __poly_foot_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.control(_foot_right_set_pos_deg);

            if (__poly_foot_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_foot_right.calculate(encodFootRightLink.getPositionDeg(), 0, 2000);
                mode = 10;
            }
        }
        break;

        case 10:
        {
            _foot_right_set_pos_deg = __poly_foot_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.control(_foot_right_set_pos_deg);

            if (__poly_foot_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), 0, 3000);
                mode = 11;
            }
        }
        break;

        case 11:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), 0, 3000);
                mode = 12;
            }
        }
        break;

        case 12:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                mode = 0;
            }
        }
        break;
        }
    }
    break;

    // Перевод экзоскелета в сидячее положение
    case SS_SEDENTARY_LINK:
    {
        static int mode = 0;
        const float __VELOCITY = 12.0;

        static Polynomial_3 __poly_hip_left;
        static Polynomial_3 __poly_knee_left;
        static Polynomial_3 __poly_foot_left;
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        static Polynomial_3 __poly_foot_right;

        static float __hip_left_posing_time;
        static float __knee_left_posing_time;
        static float __foot_left_posing_time;
        static float __hip_right_posing_time;
        static float __knee_right_posing_time;
        static float __foot_right_posing_time;

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
            }
        }

        if (isSSChange())
        {
            mode = 0;
            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);
        };

        switch (mode)
        {
        case 0:
        {
            __knee_left_posing_time = (abs(encodKneeLeftLink.getPositionDeg() - 45.0) / __VELOCITY) * 1000.0;
            __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), -45, __knee_left_posing_time);
            mode = 1;
        }
        break;

        case 1:
        {
            _knee_left_set_pos_deg = __poly_knee_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.control(_knee_left_set_pos_deg);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_left.isFinished())
            {
                stopAllMotorsLink();
                __hip_left_posing_time = (abs(encodHipLeftLink.getPositionDeg() - 85.0) / __VELOCITY) * 1000.0;
                __poly_hip_left.calculate(encodHipLeftLink.getPositionDeg(), 85, __hip_left_posing_time);
                mode = 2;
            }
        }
        break;

        case 2:
        {
            _hip_left_set_pos_deg = __poly_hip_left.getPosition();

            driverHipLeftLink.control(_hip_left_set_pos_deg);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_left.isFinished())
            {
                stopAllMotorsLink();
                __knee_left_posing_time = (abs(encodKneeLeftLink.getPositionDeg() - 90.0) / __VELOCITY) * 1000.0;
                __poly_knee_left.calculate(encodKneeLeftLink.getPositionDeg(), -90, __knee_left_posing_time);
                mode = 3;
            }
        }
        break;

        case 3:
        {
            _knee_left_set_pos_deg = __poly_knee_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.control(_knee_left_set_pos_deg);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_left.isFinished())
            {
                stopAllMotorsLink();
                __foot_left_posing_time = (abs(encodFootLeftLink.getPositionDeg() - 0.0) / __VELOCITY) * 1000.0;
                __poly_foot_left.calculate(encodFootLeftLink.getPositionDeg(), 0, __foot_left_posing_time);
                mode = 4;
            }
        }
        break;

        case 4:
        {
            _foot_left_set_pos_deg = __poly_foot_left.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.control(_foot_left_set_pos_deg);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_foot_left.isFinished())
            {
                stopAllMotorsLink();
                __knee_right_posing_time = (abs(encodKneeRightLink.getPositionDeg() - 45.0) / __VELOCITY) * 1000.0;
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), 45, __knee_right_posing_time);
                mode = 5;
            }
        }
        break;

        case 5:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                __hip_right_posing_time = (abs(encodHipRightLink.getPositionDeg() - 85.0) / __VELOCITY) * 1000.0;
                __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), -85, __hip_right_posing_time);
                mode = 6;
            }
        }
        break;

        case 6:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                __knee_right_posing_time = (abs(encodKneeRightLink.getPositionDeg() - 90.0) / __VELOCITY) * 1000.0;
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), 90, __knee_right_posing_time);
                mode = 7;
            }
        }
        break;

        case 7:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);

            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                __foot_right_posing_time = (abs(encodFootRightLink.getPositionDeg() - 0.0) / __VELOCITY) * 1000.0;
                __poly_foot_right.calculate(encodFootRightLink.getPositionDeg(), 0, __foot_right_posing_time);
                mode = 8;
            }
        }
        break;

        case 8:
        {
            _foot_right_set_pos_deg = __poly_foot_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.control(_foot_right_set_pos_deg);

            if (__poly_foot_left.isFinished())
            {
                stopAllMotorsLink();
                mode = 0;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case SS_WALK_1_LINK:
    {
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        Kinematics kinematics;
        Angle angle;
        AxisForWalk1 ax;

        static int mode = 0;
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
                mode = 0;
            }
        }

        if (isSSChange())
        {
            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);

            x_ankle_start_des = lyambda * (lengthHip + lengthKnee) * sin(angle_des);
            y_ankle_start_des = -lyambda * (lengthHip + lengthKnee) * cos(angle_des);

            mode = 0;
        }

        switch (mode)
        {
        case 0:
        {
            kinematics = directKinematics(lengthHip, lengthKnee);
            x_ankle_start = kinematics.A3x;
            y_ankle_start = kinematics.A3y;
            step_x = (x_ankle_start_des - x_ankle_start) / (100 - 1);
            step_y = (y_ankle_start_des - y_ankle_start) / (100 - 1);

            angle.q1 = deg2rad(-1 * encodHipRightLink.getPositionDeg());
            angle.q21 = deg2rad(-1 * encodKneeRightLink.getPositionDeg());

            for (int i = 0; i < 100; i++)
            {
                _x_targ = x_ankle_start + (i * step_x);
                _y_targ = y_ankle_start + (i * step_y);

                angle = inverseKinematics(_x_targ, _y_targ, 0, 0, lengthHip, lengthKnee, angle.q1, angle.q21);
            }
            q1_des = rad2deg(-1 * angle.q1);
            q21_des = rad2deg(-1 * angle.q21);

            mode = 1;
        }
        break;

        case 1:
        {
            __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), q1_des, 0, 0, 0, 3000);
            mode = 2;
        }
        break;

        case 2:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), q21_des, 0, 0, 0, 3000);
                mode = 3;
            }
        }
        break;

        case 3:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);
            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                mode = 4;
            }
        }
        break;

        case 4:
        {
            kinematics = directKinematics(lengthHip, lengthKnee);
            x_offset = kinematics.A3x;
            y_offset = kinematics.A3y;
            mode = 5;
        }
        break;

        case 5:
        {
            ax = trajectoryPlannerForWalk1(ls, hs);
            angle = inverseKinematics(ax.x_set + x_offset, ax.y_set + y_offset, 0, 0, lengthHip, lengthKnee, deg2rad(-1 * encodHipRightLink.getPositionDeg()), deg2rad(-1 * encodKneeRightLink.getPositionDeg()));

            kinematics = directKinematics(lengthHip, lengthKnee);

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(rad2deg(-1 * angle.q1));
            driverKneeRightLink.control(rad2deg(-1 * angle.q21));
            driverFootRightLink.manualControlLink(0);

            Serial.print(kinematics.A3x);
            Serial.print(" , ");
            Serial.print(ax.x_set);
            Serial.print(" , ");
            Serial.print(kinematics.A3y);
            Serial.print(" , ");
            Serial.println(ax.y_set);
        }
        break;
        }
    }
    break;

    case SS_WALK_2_LINK:
    {
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        Kinematics kinematics;
        Angle angle;
        AxisForWalk2 ax;
        static int mode = 0;

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
                mode = 0;
            }
        }

        if (isSSChange())
        {
            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);

            x_ankle_start_des = lyambda * (lengthHip + lengthKnee) * sin(angle_des);
            y_ankle_start_des = -lyambda * (lengthHip + lengthKnee) * cos(angle_des);
            mode = 0;
        }

        switch (mode)
        {
        case 0:
        {
            kinematics = directKinematics(lengthHip, lengthKnee);
            x_ankle_start = kinematics.A3x;
            y_ankle_start = kinematics.A3y;

            step_x = (x_ankle_start_des - x_ankle_start) / (100 - 1);
            step_y = (y_ankle_start_des - y_ankle_start) / (100 - 1);

            angle.q1 = deg2rad(-1 * encodHipRightLink.getPositionDeg());
            angle.q21 = deg2rad(-1 * encodKneeRightLink.getPositionDeg());

            for (int i = 0; i < 100; i++)
            {
                _x_targ = x_ankle_start + (i * step_x);
                _y_targ = y_ankle_start + (i * step_y);

                angle = inverseKinematics(_x_targ, _y_targ, 0, 0, lengthHip, lengthKnee, angle.q1, angle.q21);
            }
            q1_des = rad2deg(-1 * angle.q1);
            q21_des = rad2deg(-1 * angle.q21);

            mode = 1;
        }
        break;

        case 1:
        {
            __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), q1_des, 0, 0, 0, 3000);
            mode = 2;
        }
        break;

        case 2:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), q21_des, 0, 0, 0, 3000);
                mode = 3;
            }
        }
        break;

        case 3:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);
            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                mode = 4;
            }
        }
        break;

        case 4:
        {
            ax = trajectoryPlannerForWalk2(hs);

            angle = inverseKinematics(ax.x_set, ax.y_set, 0, 0, lengthHip, lengthKnee, deg2rad(-1 * encodHipRightLink.getPositionDeg()), deg2rad(-1 * encodKneeRightLink.getPositionDeg()));

            kinematics = directKinematics(lengthHip, lengthKnee);

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(rad2deg(-1 * angle.q1));
            driverKneeRightLink.control(rad2deg(-1 * angle.q21));
            driverFootRightLink.manualControlLink(0);

            Serial.print(kinematics.A3x);
            Serial.print(" , ");
            Serial.print(ax.x_set);
            Serial.print(" , ");
            Serial.print(kinematics.A3y);
            Serial.print(" , ");
            Serial.println(ax.y_set);
        }
        break;
        }
    }
    break;

    case SS_HMN_GAIT_LINK:
    {
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        Kinematics kinematics;
        Angle angle;

        static int mode = 0;

        if (isSSChange())
        {
            // Включаем питание приводов
            driverHipLeftLink.setEnable(true);
            driverKneeLeftLink.setEnable(true);
            driverFootLeftLink.setEnable(true);
            driverHipRightLink.setEnable(true);
            driverKneeRightLink.setEnable(true);
            driverFootRightLink.setEnable(true);
        }

        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotorsLink();
                system_state = SS_MAIN_MENU;
                mode = 0;
            }
        }

        switch (mode)
        {
        case 0:
        {
            kinematics = directKinematics(lengthHip, lengthKnee);
            x_ankle_start = kinematics.A3x;
            y_ankle_start = kinematics.A3y;
            getApproxHmnGt(_x_targ, _y_targ, 0);

            _x_targ = hmn_gt_scaling * _x_targ;
            _y_targ = (hmn_gt_scaling * _y_targ) + hmn_gt_z_offset;

            step_x = (_x_targ - x_ankle_start) / (100 - 1);
            step_y = (_y_targ - y_ankle_start) / (100 - 1);

            angle.q1 = deg2rad(-1 * encodHipRightLink.getPositionDeg());
            angle.q21 = deg2rad(-1 * encodKneeRightLink.getPositionDeg());

            for (int i = 0; i < 100; i++)
            {
                _x_targ = x_ankle_start + (i * step_x);
                _y_targ = y_ankle_start + (i * step_y);

                angle = inverseKinematics(_x_targ, _y_targ, 0, 0, lengthHip, lengthKnee, angle.q1, angle.q21);
            }
            q1_des = rad2deg(-1 * angle.q1);
            q21_des = rad2deg(-1 * angle.q21);
            Serial.print(q1_des);
            Serial.print(" , ");
            Serial.println(q21_des);
            mode = 1;
        }
        break;

        case 1:
        {
            __poly_hip_right.calculate(encodHipRightLink.getPositionDeg(), q1_des, 0, 0, 0, 3000);
            mode = 2;
        }
        break;

        case 2:
        {
            _hip_right_set_pos_deg = __poly_hip_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(_hip_right_set_pos_deg);
            driverKneeRightLink.manualControlLink(0);
            driverFootRightLink.manualControlLink(0);

            if (__poly_hip_right.isFinished())
            {
                stopAllMotorsLink();
                __poly_knee_right.calculate(encodKneeRightLink.getPositionDeg(), q21_des, 0, 0, 0, 3000);
                mode = 3;
            }
        }
        break;

        case 3:
        {
            _knee_right_set_pos_deg = __poly_knee_right.getPosition();

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.manualControlLink(0);
            driverKneeRightLink.control(_knee_right_set_pos_deg);
            driverFootRightLink.manualControlLink(0);
            if (__poly_knee_right.isFinished())
            {
                stopAllMotorsLink();
                mode = 4;
            }
        }
        break;

        case 4:
        {
            if (millis() - last_time_hmn_gait > 16)
            {
                last_time_hmn_gait = millis();

                t = k * dt;
                getApproxHmnGt(_x_targ, _y_targ, t / hmn_gt_time);
                k = k + 1;

                if (k >= hmn_gt_size)
                {
                    k = hmn_gt_size;
                }

                _x_targ = hmn_gt_scaling * _x_targ;
                _y_targ = (hmn_gt_scaling * _y_targ) + hmn_gt_z_offset;
            }

            if (k == hmn_gt_size)
                k = 0;

            angle = inverseKinematics(_x_targ, _y_targ, 0, 0, lengthHip, lengthKnee, deg2rad(-1 * encodHipRightLink.getPositionDeg()), deg2rad(-1 * encodKneeRightLink.getPositionDeg()));

            kinematics = directKinematics(lengthHip, lengthKnee);

            driverHipLeftLink.manualControlLink(0);
            driverKneeLeftLink.manualControlLink(0);
            driverFootLeftLink.manualControlLink(0);
            driverHipRightLink.control(rad2deg(-1 * angle.q1));
            driverKneeRightLink.control(rad2deg(-1 * angle.q21));
            driverFootRightLink.manualControlLink(0);

            Serial.print(kinematics.A3x);
            Serial.print(" , ");
            Serial.print(_x_targ, 6);
            Serial.print(" , ");
            Serial.print(kinematics.A3y);
            Serial.print(" , ");
            Serial.println(_y_targ, 6);
        }
        break;
        }
    }
    break;
    // case SS_MAIN_MENU:
    // {
    //     if (isSSChange())
    //         ;
    //     // Ждем ручной запуск системы
    //     if (Serial.available())
    //     {
    //         String input = Serial.readStringUntil('\n');
    //         if (input.equals("Init"))
    //         {
    //             system_state = SS_ZEROING_ADJ;
    //             is_zeroed_flag = false;
    //         }
    //         else if (input.startsWith("Demo"))
    //         {
    //             system_state = SS_DEMO_ADJ;
    //         }
    //         else if (input.startsWith("ManAdj"))
    //         {
    //             String params[7];
    //             int paramsCount = 0;
    //             int speed;
    //             int motorNumber;
    //             splitString(input, ';', params, 7, paramsCount);
    //             if (paramsCount > 1)
    //             {
    //                 for (int i = 1; i < paramsCount; i++)
    //                 {
    //                     String param = params[i];
    //                     if (param.length() > 0)
    //                     {
    //                         motorNumber = param.toInt();
    //                         speed = param.substring(param.indexOf(':') + 1).toInt();
    //                         if (input.startsWith("ManAdjUp"))
    //                             setMotorSpeedAdj(motorNumber, speed);
    //                         else if (input.startsWith("ManAdjDown"))
    //                             setMotorSpeedAdj(motorNumber, -speed);
    //                     }
    //                 }
    //             }
    //         }
    //         else if (input.startsWith("Length"))
    //         {
    //             String params[7];
    //             int paramsCount = 0;
    //             splitString(input, ';', params, 7, paramsCount);
    //             if (paramsCount > 1)
    //             {
    //                 for (int i = 1; i < paramsCount; i++)
    //                 {
    //                     String param = params[i];
    //                     if (param.length() > 0)
    //                     {
    //                         linkNumber = param.substring(0, param.indexOf(':') + 1).toInt();
    //                         length = param.substring(param.indexOf(':') + 1, param.lastIndexOf(':')).toInt();
    //                         time = param.substring(param.lastIndexOf(':') + 1).toInt();
    //                         system_state = SS_POSITION_DESIRED_ADJ;
    //                     }
    //                 }
    //             }
    //         }
    //         else if (input.equals("Stop"))
    //         {
    //             stopAllMotors();
    //         }
    //     }
    // }
    // break;

    case SS_POSITION_DESIRED_ADJ:
    {
        SetPositionDesired(linkNumber, length, time);
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            if (input.equals("Stop"))
            {
                stopAllMotors();
                system_state = SS_MAIN_MENU;
            }
        }
    }
    break;

    case SS_ZEROING_ADJ:
    {
        // Напряжение на приводах при их инициализации
        const int _INIT_VOLTAGE = 3;
        // Скорость увеличения напряжения при инициализации, [Вольт/сек]
        const float _CALIBRATION_VELOCITY_ENCOD_ZERO = 1.5;

        if (isSSChange())
            ;

        // Плавно увеличиваем напряжение на двигателе
        static float _voltage = 0;                           // Напряжение, подаваемое на драйвер
        static unsigned long _loop_timer = 0;                // Время в микросекундах
        unsigned long _delta_timer = millis() - _loop_timer; // Период цикла в микросекундах
        _loop_timer = millis();
        // Если напряжение меньше заданного, увеличиваем его по линейному закону
        if (_voltage < _INIT_VOLTAGE)
            _voltage += _CALIBRATION_VELOCITY_ENCOD_ZERO * ((float)_delta_timer / 1000.0);

        switch (zeroing_state)
        {
        case ZS_HIP_LEFT:
        {
            // Обнуляем переменную напряжение драйвера
            if (isZSChange())
                _voltage = 0;
            // Если концевик не нажат, перемещаем привод в сторону концевика
            if (digitalRead(PIN_SWITCH_HIP_LEFT))
                driverHipLeftAdj.manualControlAdj(_voltage);
            // Если концевик нажат
            else
            {
                // Останавливаем привод (иначе он продолжит двигаться, т.к. analogWrite()
                // "Запоминает" ШИМ, переданный на пин, даже если функция больше не вызывается
                driverHipLeftAdj.manualControlAdj(0);
                // Обнуляем положение энкодера
                encodHipLeftAdj.setZero();
                // Приключаем режим
                zeroing_state = ZS_KNEE_LEFT;
            }
        }
        break;

        case ZS_KNEE_LEFT:
        {
            if (isZSChange())
                _voltage = 0;

            if (digitalRead(PIN_SWITCH_KNEE_LEFT))
                driverKneeLeftAdj.manualControlAdj(_voltage);
            else
            {
                driverKneeLeftAdj.manualControlAdj(0);
                encodKneeLeftAdj.setZero();
                zeroing_state = ZS_HIP_RIGHT;
            }
        }
        break;

        case ZS_HIP_RIGHT:
        {
            if (isZSChange())
                _voltage = 0;

            if (digitalRead(PIN_SWITCH_HIP_RIGHT))
                driverHipRightAdj.manualControlAdj(_voltage);
            else
            {
                driverHipRightAdj.manualControlAdj(0);
                encodHipRightAdj.setZero();
                zeroing_state = ZS_KNEE_RIGHT;
            }
        }
        break;

        case ZS_KNEE_RIGHT:
        {
            if (isZSChange())
                _voltage = 0;

            if (digitalRead(PIN_SWITCH_KNEE_RIGHT))
                driverKneeRightAdj.manualControlAdj(_voltage);
            else
            {
                driverKneeRightAdj.manualControlAdj(0);
                encodKneeRightAdj.setZero();

                is_zeroed_flag = true;
                zeroing_state = ZS_HIP_LEFT;
                system_state = SS_MAIN_MENU;
            }
        }
        break;
        }
    }
    break;

    case SS_DEMO_ADJ:
    {
        static Polynomial_3 __poly_hip_left;
        static Polynomial_3 __poly_knee_left;
        static Polynomial_3 __poly_foot_left;
        static Polynomial_3 __poly_hip_right;
        static Polynomial_3 __poly_knee_right;
        static Polynomial_3 __poly_foot_right;

        static int mode = 0;

        // if(!is_zeroed_flag){
        //     system_state = SS_ZEROING;
        // }
        // else{
        switch (demo_state)
        {
        case DS_HIP_LEFT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_hip_left.calculate(encodHipLeftAdj.getPosition(), -10, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT), encodHipLeftHandler, CHANGE);
                _hip_left_set_pos_deg_adj = __poly_hip_left.getPosition();
                driverHipLeftAdj.control(_hip_left_set_pos_deg_adj);

                if (__poly_hip_left.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT));
                    driverHipLeftAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_KNEE_LEFT;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodHipLeftAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_hip_left_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_KNEE_LEFT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_knee_left.calculate(encodKneeLeftAdj.getPosition(), -10, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT), encodKneeLeftHandler, CHANGE);
                _knee_left_set_pos_deg_adj = __poly_knee_left.getPosition();
                driverKneeLeftAdj.control(_knee_left_set_pos_deg_adj);

                if (__poly_knee_left.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT));
                    driverKneeLeftAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_HIP_RIGHT;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodKneeLeftAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_knee_left_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_FOOT_LEFT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_foot_left.calculate(getAngleFootLeftADC(), -10, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                _foot_left_set_pos_deg_adj = __poly_foot_left.getPosition();
                driverFootLeftAdj.control(_foot_left_set_pos_deg_adj);

                if (__poly_foot_left.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_ANGLE_FOOT_LEFT));
                    driverFootLeftAdj.manualControl(0);
                    mode = 0;
                    demo_state = DS_FOOT_LEFT;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.println(_foot_left_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_HIP_RIGHT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_hip_right.calculate(encodHipRightAdj.getPosition(), -10, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT), encodHipRightHandler, CHANGE);
                _hip_right_set_pos_deg_adj = __poly_hip_right.getPosition();
                driverHipRightAdj.control(_hip_right_set_pos_deg_adj);

                if (__poly_hip_right.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT));
                    driverHipRightAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_KNEE_RIGHT;
                    system_state = SS_MAIN_MENU;
                }
                Serial.print(encodHipRightAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_hip_right_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_KNEE_RIGHT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_knee_right.calculate(encodKneeRightAdj.getPosition(), -10, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT), encodKneeRightHandler, CHANGE);
                _knee_right_set_pos_deg_adj = __poly_knee_right.getPosition();
                driverKneeRightAdj.control(_knee_right_set_pos_deg_adj);

                if (__poly_knee_right.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT));
                    driverKneeRightAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_KNEE_RIGHT_BACK;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodKneeRightAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_knee_right_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_FOOT_RIGHT:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_foot_right.calculate(getAngleFootRightADC(), -10, 1000);
                mode = 1;
            }
            break;

            case 1:
            {
                _foot_right_set_pos_deg_adj = __poly_foot_right.getPosition();
                driverFootRightAdj.control(_foot_right_set_pos_deg_adj);

                if (__poly_foot_right.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_ANGLE_FOOT_RIGHT));
                    driverFootRightAdj.manualControl(0);
                    mode = 0;
                    demo_state = DS_FOOT_RIGHT;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.println(_foot_right_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_KNEE_RIGHT_BACK:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_knee_right.calculate(encodKneeRightAdj.getPosition(), 0, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT), encodKneeRightHandler, CHANGE);
                _knee_right_set_pos_deg_adj = __poly_knee_right.getPosition();
                driverKneeRightAdj.control(_knee_right_set_pos_deg_adj);

                if (__poly_knee_right.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_RIGHT));
                    driverKneeRightAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_HIP_RIGHT_BACK;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodKneeRightAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_knee_right_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_HIP_RIGHT_BACK:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_hip_right.calculate(encodHipRightAdj.getPosition(), 0, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT), encodHipRightHandler, CHANGE);
                _hip_right_set_pos_deg_adj = __poly_hip_right.getPosition();
                driverHipRightAdj.control(_hip_right_set_pos_deg_adj);

                if (__poly_hip_right.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_RIGHT));
                    driverHipRightAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_KNEE_LEFT_BACK;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodHipRightAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_hip_right_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_KNEE_LEFT_BACK:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_knee_left.calculate(encodKneeLeftAdj.getPosition(), 0, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT), encodKneeLeftHandler, CHANGE);
                _knee_left_set_pos_deg_adj = __poly_knee_left.getPosition();
                driverKneeLeftAdj.control(_knee_left_set_pos_deg_adj);

                if (__poly_knee_left.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_KNEE_LEFT));
                    driverKneeLeftAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_HIP_LEFT_BACK;
                    // system_state = SS_MAIN_MENU;
                }
                Serial.print(encodKneeLeftAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_knee_left_set_pos_deg_adj);
            }
            break;
            }
        }
        break;

        case DS_HIP_LEFT_BACK:
        {
            switch (mode)
            {
            case 0:
            {
                __poly_hip_left.calculate(encodHipLeftAdj.getPosition(), 0, 1500);
                mode = 1;
            }
            break;

            case 1:
            {
                attachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT), encodHipLeftHandler, CHANGE);
                _hip_left_set_pos_deg_adj = __poly_hip_left.getPosition();
                driverHipLeftAdj.control(_hip_left_set_pos_deg_adj);

                if (__poly_hip_left.isFinished())
                {
                    detachInterrupt(digitalPinToInterrupt(PIN_OPTO_ENCODER_HIP_LEFT));
                    driverHipLeftAdj.manualControlAdj(0);
                    mode = 0;
                    demo_state = DS_HIP_LEFT_BACK;
                    system_state = SS_MAIN_MENU;
                }
                Serial.print(encodHipLeftAdj.getPosition());
                Serial.print(" , ");
                Serial.println(_hip_left_set_pos_deg_adj);
            }
            break;
            }
        }
        break;
        }
        // }
    }
    break;
    }
}

//....................................................................................................................................................................
// регулировки

// void mainControlAdj()
// {
//     static float _hip_left_set_pos_deg_adj;
//     static float _knee_left_set_pos_deg_adj;
//     static float _foot_left_set_pos_deg_adj;
//     static float _hip_right_set_pos_deg_adj;
//     static float _knee_right_set_pos_deg_adj;
//     static float _foot_right_set_pos_deg_adj;

//     static int linkNumber;
//     static int length;
//     static int time;

//     switch (system_state)
//     {
//     }
// }