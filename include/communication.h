#pragma once

int angle_hip_left, angle_knee_left, angle_foot_left, angle_hip_right, angle_knee_right, angle_foot_right;
int force_hip_left, force_knee_left, force_foot_left1, force_foot_left2, force_foot_left3, force_hip_right, force_knee_right, force_foot_right1, force_foot_right2, force_foot_right3;
int value_hip_left, value_knee_left, value_foot_left, value_hip_right, value_knee_right, value_foot_right; //переменные для регулировок

String start_marker = "A0";
String end_marker = "C0";
String separator = ",";

void updateSensors(){
    angle_hip_left = encodHipLeftLink.getPositionDeg();
    angle_knee_left = encodKneeLeftLink.getPositionDeg();
    angle_foot_left = encodFootLeftLink.getPositionDeg();

    angle_hip_right = encodHipRightLink.getPositionDeg();
    angle_knee_right = encodKneeRightLink.getPositionDeg();
    angle_foot_right = encodFootRightLink.getPositionDeg();

    force_hip_left = scaleHipLeft.getForce();
    force_knee_left = scaleKneeLeft.getForce();
    force_foot_left1 = scaleFootLeft1.getForce();
    force_foot_left2 = scaleFootLeft2.getForce();
    force_foot_left3 = scaleFootLeft3.getForce();

    force_hip_right = scaleHipRight.getForce();
    force_knee_right = scaleKneeRight.getForce();
    force_foot_right1 = scaleFootRight1.getForce();
    force_foot_right2 = scaleFootRight2.getForce();
    force_foot_right3 = scaleFootRight3.getForce();

    //регулировки
    value_hip_left = encodHipLeftAdj.getPosition();
    value_knee_left = encodKneeLeftAdj.getPosition();
    value_foot_left = getAngleFootLeftADC();

    value_hip_right = encodHipRightAdj.getPosition();
    value_knee_right = encodKneeRightAdj.getPosition();
    value_foot_right = getAngleFootRightADC();
}

void sendingData(){
    static unsigned long _loop_timer = 0; 
    unsigned long _delta_timer = millis() - _loop_timer;    // Период цикла в милисекундах
    
    if(_delta_timer >= 200){
        updateSensors();
        String _output = "";
        
        _output += start_marker;
        _output += angle_hip_left;
        _output += separator;
        _output += angle_knee_left;
        _output += separator;
        _output += angle_foot_left;
        _output += separator;

        _output += -1 * angle_hip_right;
        _output += separator;
        _output += -1 * angle_knee_right;
        _output += separator;
        _output += -1 * angle_foot_right;
        _output += separator;

        _output += force_hip_left;
        _output += separator;
        _output += force_knee_left;
        _output += separator;

        _output += force_hip_right;
        _output += separator;
        _output += force_knee_right;
        _output += separator;

        _output += force_foot_left1;
        _output += separator;
        _output += force_foot_left2;
        _output += separator;
        _output += force_foot_left3;
        _output += separator;

        _output += force_foot_right1;
        _output += separator;
        _output += force_foot_right2;
        _output += separator;
        _output += force_foot_right3;
        _output += separator;

        _output += value_hip_left;
        _output += separator;
        _output += value_knee_left;
        _output += separator;
        _output += value_foot_left;
        _output += separator;
        _output += value_hip_right;
        _output += separator;
        _output += value_knee_right;
        _output += separator;
        _output += value_foot_right;
        _output += end_marker;

        Serial.println(_output);
        _loop_timer = millis();
    }
}