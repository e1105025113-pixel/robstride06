#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <string.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

const uint8_t MOTOR_ID = 0x7F;     // 127
const uint16_t HOST_ID = 0xFD;

// 0 = MIT
// 1 = Position
// 2 = Velocity
// 3 = Current
// 5 = CSP

uint8_t currentMode = 0;//モード指定

// -------- Position PP --------
float TARGET_POSITION = 0.0f;     

// -------- Velocity --------
float TARGET_SPEED = 30.0f;        

// -------- Current --------
float TARGET_CURRENT = 2.0f; 

// -------- MIT / Operation --------
float MIT_POSITION = 0.0f;//目標位置
float MIT_VELOCITY = 5.0f;//目標速度
float MIT_KP = 0.0f;//比例ゲイン
float MIT_KD = 6.0f;//微分ゲイン
float MIT_TORQUE_FF = 1.0f;//トルクフィードフォワード

//フィードバック
float actualspeed=0.0f;
float actualtorque=0.0f;
//====速度PI制御用パラメータ====
float speed_kp=0.5f;//速度比例ゲイン
float speed_ki=0.3f;//速度積分ゲイン

float speed_integral=0.0f;//速度積分値
float torque_command=0.0f;

const float torque_limit=5.0f;//トルク制限
const float control_dt=0.02f;//周期制限
//=================固定設定===================
const float CURRENT_LIMIT = 10.0f;
const float ACCELERATION = 5.0f;
bool stopped = false;

// CAN ID
uint32_t makeCANID(uint8_t type){
    return ((uint32_t)type << 24)| ((uint32_t)HOST_ID << 8)| MOTOR_ID;
}

// Floatパラメータ書き込み
void writeFloat(uint16_t index, float value){
    CAN_message_t msg;

    msg.id = makeCANID(0x12);
    msg.flags.extended = 1;
    msg.len = 8;

    memset(msg.buf, 0, 8);

    memcpy(&msg.buf[0], &index, 2);
    memcpy(&msg.buf[4], &value, 4);

    Can1.write(msg);
}


// モード設定
void setMode(uint8_t mode){
    CAN_message_t msg;

    msg.id = makeCANID(0x12);
    msg.flags.extended = 1;
    msg.len = 8;

    memset(msg.buf, 0, 8);

    uint16_t index = 0x7005;

    memcpy(&msg.buf[0], &index, 2);
    memcpy(&msg.buf[4], &mode, 1);

    Can1.write(msg);

    delay(50);

    Serial.print("Run Mode = ");

    switch (mode){
        case 0:
            Serial.println("0 : Operation / MIT");
            break;

        case 1:
            Serial.println("1 : Position PP");
            break;

        case 2:
            Serial.println("2 : Velocity");
            break;

        case 3:
            Serial.println("3 : Current");
            break;

        case 5:
            Serial.println("5 : CSP");
            break;

        default:
            Serial.println("Unknown");
            break;
    }
}

//モーター有効化
void motorEnable(){
    CAN_message_t msg;

    msg.id = makeCANID(0x03);
    msg.flags.extended = 1;
    msg.len = 8;

    memset(msg.buf, 0, 8);

    Can1.write(msg);

    Serial.println("Motor ENABLE");
}

//モーター無効化
void motorStop(){
    CAN_message_t msg;

    msg.id = makeCANID(0x04);
    msg.flags.extended = 1;
    msg.len = 8;

    memset(msg.buf, 0, 8);

    Can1.write(msg);

    Serial.println("Motor STOP");
}

//位置制御
void positionControl(){
    writeFloat(0x7016,TARGET_POSITION);
}

//速度制御
void velocityControl(){
    writeFloat(0x700A,TARGET_SPEED);
}

//電流制御
void currentControl(){
    writeFloat(0x7006,TARGET_CURRENT);
}

//MIT制御用のfloat→uint16_t変換
uint16_t floatToUint(float x,float x_min,float x_max)
{
    if (x < x_min)x = x_min;

    if (x > x_max)x = x_max;

    return (uint16_t)((x - x_min)* 65535.0f/ (x_max - x_min));
}

float speedPIControl(){// 速度PI制御
    float error = MIT_VELOCITY - actualspeed;

    // 積分
    speed_integral += error * control_dt;

    // 積分ワインドアップ防止
    const float INTEGRAL_LIMIT = 5.0f;

    if (speed_integral > INTEGRAL_LIMIT)
        speed_integral = INTEGRAL_LIMIT;

    if (speed_integral < -INTEGRAL_LIMIT)
        speed_integral = -INTEGRAL_LIMIT;

    // PI計算
    float correction =speed_kp * error +speed_ki * speed_integral;

    return correction;
}
//MIT制御
void MITControl(){
    const float P_MIN = -4.0f * PI;
    const float P_MAX =  4.0f * PI;

    const float V_MIN = -50.0f;
    const float V_MAX =  50.0f;

    const float T_MIN = -36.0f;
    const float T_MAX =  36.0f;

    uint16_t p_int =floatToUint(MIT_POSITION,P_MIN,P_MAX);//目標位置

    uint16_t v_int =floatToUint(MIT_VELOCITY,V_MIN,V_MAX);//目標速度

    uint16_t kp_int =floatToUint(MIT_KP,0.0f,5000.0f);//比例ゲイン

    uint16_t kd_int =floatToUint(MIT_KD,0.0f,100.0f);//微分ゲイン

// 速度PIによるトルク補正
float pi_correction = speedPIControl();

// 重力補償トルク + PI補正
torque_command =MIT_TORQUE_FF + pi_correction;

// トルク制限
if (torque_command > torque_limit)torque_command = torque_limit;

if (torque_command < -torque_limit)torque_command = -torque_limit;

uint16_t t_int =floatToUint(torque_command,T_MIN,T_MAX);

    CAN_message_t msg;

    msg.id =((uint32_t)0x01 << 24)|((uint32_t)t_int << 8)|MOTOR_ID;

    msg.flags.extended = 1;
    msg.len = 8;

    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;

    msg.buf[2] = v_int >> 8;
    msg.buf[3] = v_int & 0xFF;

    msg.buf[4] = kp_int >> 8;
    msg.buf[5] = kp_int & 0xFF;

    msg.buf[6] = kd_int >> 8;
    msg.buf[7] = kd_int & 0xFF;

    Can1.write(msg);
    Serial.print("Target V = ");
    Serial.print(MIT_VELOCITY, 3);

    Serial.print(" | Actual V = ");
    Serial.print(actualspeed, 3);

    Serial.print(" | Torque Cmd = ");
    Serial.println(torque_command, 3);
}

//CSP制御
void CSPControl(){
    writeFloat(0x7016,TARGET_POSITION);
}

//上の制御関数を元に送信
void sendControlCommand(){
    switch (currentMode){
        case 0://MIT制御
            MITControl();
            break;
        case 1://位置制御
            positionControl();
            break;
        case 2://速度制御
            velocityControl();
            break;
        case 3://電流制御
            currentControl();
            break;
        case 5://CSP制御
            CSPControl();
            break;
        default:
            break;
    }
}

//feedback受信
void receiveCAN(){
    CAN_message_t msg;

    while (Can1.read(msg)){
        uint8_t type =(msg.id >> 24) & 0x1F;

        if (type == 2 && msg.len >= 8){
            uint16_t rawPosition =((uint16_t)msg.buf[0] << 8)| msg.buf[1];//位置データ

            uint16_t rawSpeed =((uint16_t)msg.buf[2] << 8)| msg.buf[3];//速度データ

            uint16_t rawTorque =((uint16_t)msg.buf[4] << 8)| msg.buf[5];//トルクデータ

            uint16_t rawTemp =((uint16_t)msg.buf[6] << 8)| msg.buf[7];//温度データ

            //位置
            float position =((float)rawPosition / 65535.0f)* (8.0f * PI)- (4.0f * PI);
            
            //速度制御
            float speed =((float)rawSpeed / 65535.0f)* 100.0f- 50.0f;
            actualspeed=speed;
            //rad/sをrpmに変換
            float rpm =speed * 60.0f / (2.0f * PI);

            //トルク
            float torque =((float)rawTorque / 65535.0f)* 72.0f- 36.0f;
            actualtorque=torque;
            //温度
            float temperature =(float)rawTemp / 10.0f;
            
            //シリアルモニターに計算したパラメータを表示
            Serial.print("POS = ");
            Serial.print(position, 3);
            Serial.print(" rad   ");//位置表示

            Serial.print("VEL = ");
            Serial.print(speed, 3);
            Serial.print(" rad/s   ");//速度表示

            Serial.print("RPM = ");
            Serial.print(rpm, 2);
            Serial.print(" rpm   ");//rpm表示

            Serial.print("TORQUE = ");
            Serial.print(torque, 3);
            Serial.print(" Nm   ");//トルク表示

            Serial.print("TEMP = ");
            Serial.print(temperature, 1);
            Serial.println(" C");//温度表示
        }
    }
}

//停止
void emergencyStop(){
    Serial.println();
    Serial.println("==============================");
    Serial.println("          STOP");
    Serial.println("==============================");

    motorStop();

    stopped = true;

    Serial.println("Motor stopped");
    Serial.println("R = Restart");
}

//シリアルモニターに現在のモードを表示
void printMode(){
    Serial.println();
    Serial.println("==============================");
    Serial.println("Current Mode");

    switch (currentMode){
        case 0:
            Serial.println("0 : Operation / MIT");
            break;

        case 1:
            Serial.println("1 : Position PP");
            break;

        case 2:
            Serial.println("2 : Velocity");
            break;

        case 3:
            Serial.println("3 : Current");
            break;

        case 5:
            Serial.println("5 : CSP");
            break;
    }

    Serial.println("==============================");
}


// ======================================================
// モード変更
// ======================================================

void changeMode(uint8_t mode)
{
    // まず停止
    motorStop();

    delay(100);

    currentMode = mode;

    setMode(currentMode);

    delay(100);

    motorEnable();

    delay(100);

    stopped = false;

    printMode();
}

void setup(){
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" RS06 MULTI MODE CONTROLLER");
    Serial.println(" Teensy 4.0 / CAN1 / 1 Mbps");
    Serial.println(" Motor ID = 0x7F");
    Serial.println("================================");

    Can1.begin();
    Can1.setBaudRate(1000000);//通信速度を1Mbpsに設定

    delay(500);

    Serial.println("CAN1 = 1 Mbps");

    writeFloat(0x7018,CURRENT_LIMIT);//電流制限値設定
    delay(50);

    writeFloat(0x7022,ACCELERATION);//加速度設定
    delay(50);

    setMode(currentMode);//初期モード設定
    delay(100);

    motorEnable();//モーター有効化
    delay(100);

    stopped = false;

    Serial.println();
    Serial.println("================================");
    Serial.println(" CONTROL READY");
    Serial.println("================================");

    Serial.println("MODE SELECT:");
    Serial.println("0 = Operation / MIT");
    Serial.println("1 = Position PP");
    Serial.println("2 = Velocity");
    Serial.println("3 = Current");
    Serial.println("5 = CSP");

    Serial.println();
    Serial.println("S = STOP");
    Serial.println("R = RESTART");

    Serial.println("================================");
}

void loop(){

    static unsigned long lastCommand = 0;

    receiveCAN();//CAN受信
    if (!stopped){
        if (millis() - lastCommand >= 20){
            lastCommand = millis();

            sendControlCommand();
        }
    }

   

    //シリアルモニターからの入力を処理
    while (Serial.available() > 0){
        char c = Serial.read();

        if (c == '0'){//モード0(MIT)に変更
            Serial.println("Changing to Operation / MIT");

            changeMode(0);
        }
        else if (c == '1'){//モード1(Position PP)に変更
            Serial.println("Changing to Position PP");

            changeMode(1);
        }
        else if (c == '2'){//モード2(Velocity)に変更
            Serial.println("Changing to Velocity");

            changeMode(2);
        }
        else if (c == '3'){//モード3(Current)に変更
            Serial.println("Changing to Current");

            changeMode(3);
        }
        else if (c == '5'){//モード5(CSP)に変更
            Serial.println("Changing to CSP");

            changeMode(5);
        }
        else if (c == 's' || c == 'S'){//停止
            emergencyStop();
        }
        else if (c == 'r' || c == 'R'){//再開
            Serial.println("Restarting motor...");

            motorEnable();

            delay(100);

            stopped = false;

            Serial.println("Motor restarted");
        }
        else if (c == 'm' || c == 'M'){//現在のモードを表示
            printMode();
        }
    }
}