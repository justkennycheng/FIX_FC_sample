#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_bt.h>


//按照MPU6050上印刷的坐标轴, X向前, Y向左,Z向上, 根据右手方向: 右滚为正, 低头为正,左转为正.
// ---------------------------
// 遥控信号接收部分定义
//定义第二串口引脚
#define PIN_RX 20//18
#define PIN_TX 21//19

//ADC
#define ADC_PIN 0  //用于电池电压

//定义LEDC通道用于硬件PWM输出
#define LEDC_Throttle_PIN 5
#define LEDC_Roll1_PIN 1
#define LEDC_Roll2_PIN 2
#define LEDC_Pitch_PIN 3
#define LEDC_Yaw_PIN 4

//LEDC配置
#define LEDC_TIMER_BIT 12   //ESP32-C3只支持最大14bit. 	12bit是0 ~ 4095
//LEDC frequency
#define LEDC_FREQ_400 50    //400Hz 用于电调和数字舵机
#define LEDC_FREQ_50 50     //50Hz 用于传统舵机,高频率不支持

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
//存储获取并转化后的遥控器指令
struct RcData {
    float roll_CMD;
    float pitch_CMD;
    float thr_CMD;
    float yaw_CMD;
    int SA_CMD;
    int SB_CMD;
    int SC_CMD;
    int SD_CMD;
    int SE_CMD;
    float S1_CMD;
};
RcData rc;   // 全局变量

// ---------------------------
// 硬件 & DMP 相关
MPU6050 mpu;

bool dmpReady = false;      // DMP 是否初始化成功
uint8_t mpuIntStatus;       // 中断状态
uint8_t devStatus;          // DMP 初始化状态
uint16_t packetSize;        // 每个 DMP 包的字节数
uint16_t fifoCount;         // FIFO 当前字节数
uint8_t fifoBuffer[64];     // FIFO 缓冲区

int flymode = 2;   //默认easy模式

// 姿态相关（四元数 + YPR）
Quaternion q_current;               // 当前姿态四元数
Quaternion q_target;               //  目标姿态四元数
VectorFloat gravity;        // 重力向量
float ypr[3];               // yaw, pitch, roll（单位：弧度）

float pitch = 0.0f;  //显示用
float roll = 0.0f;   //显示用
float yaw = 0.0f;    //显示用

//陀螺仪数据
int16_t gx_raw, gy_raw, gz_raw;

// ----------- PID 状态存储变量 -----------
float roll_i = 0.0f; float roll_last = 0.0f;
float pitch_i = 0.0f; float pitch_last = 0.0f;
float yaw_i = 0.0f; float yaw_last = 0.0f;

//拨轮转化后的倍数Kp_gain
float Kp_gain = 1.0f;

//最大姿态角（可调）45度
float max_roll  = 45.0f * 0.0174533f;   //转换为 rad
float max_pitch = 45.0f * 0.0174533f;   //转换为 rad

float roll_target, pitch_target;
float yaw_target;

float current_yaw; Quaternion current_q_tilt; 
Quaternion target_q_tile, target_q_yaw;

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

void setup() {
    
    Serial.begin(115200);
    Serial.println("start...");
    delay(500);

      // --- 关闭 WiFi ---
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    // --- 关闭蓝牙 ---
    #if CONFIG_IDF_TARGET_ESP32
    btStop();  // ESP32 专用
    Serial.println("Bluetooth (Classic+BLE) OFF");
    #elif CONFIG_IDF_TARGET_ESP32C3
    esp_bt_controller_disable();  // ESP32-C3 专用
    Serial.println("BLE OFF");
    #endif
    Serial.println("WiFi OFF");

    //-----------------------------------------
    //遥控信号部分
    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
    if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
    crsf.begin(crsfSerial);

    //ADC配置
    pinMode(ADC_PIN, INPUT);
    analogReadResolution(12);   //默认值为12位（范围从 0 到 4095）
    analogSetAttenuation(ADC_11db);  //ADC_11db是新版本的写法,代替旧版本的ADC_ATTEN_DB_110 mV ~ 3100 mV

    //LEDC绑定
    ledcAttach(LEDC_Throttle_PIN, LEDC_FREQ_400, LEDC_TIMER_BIT);
    ledcAttach(LEDC_Roll1_PIN, LEDC_FREQ_50, LEDC_TIMER_BIT);
    ledcAttach(LEDC_Roll2_PIN, LEDC_FREQ_50, LEDC_TIMER_BIT);
    ledcAttach(LEDC_Pitch_PIN, LEDC_FREQ_50, LEDC_TIMER_BIT);
    ledcAttach(LEDC_Yaw_PIN, LEDC_FREQ_50, LEDC_TIMER_BIT);

    //-----------------------------------------
    //MPU6050初始化部分
    // ESP32-C3 Mini I2C 引脚：SDA=8, SCL=9
    Wire.begin(8, 9);  //(8, 9);
    Wire.setClock(400000);

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    //设置时钟源（飞控必做）,切换到 陀螺仪 PLL 时钟(若不设置, MPU6050 默认使用内部 8MHz RC 振荡器)
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    //设置陀螺仪量程,飞控一般用 ±1000°/s 或 ±500°/s
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    //设置加速度计量程,默认 ±2g，太小，飞控一般用 ±4g 或 ±8g
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    //设置硬件低通滤波（DLPF）
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);   // 或 42Hz
    //设置采样率（可选但推荐）
    mpu.setRate(4);   // 200Hz

    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    //mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    //mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);

    if (devStatus == 0) {

        Serial.println("Self-calibrating...");
        //mpu.CalibrateAccel(6); // 自动迭代 6 次校准加速度计
        //mpu.CalibrateGyro(6);  // 自动迭代 6 次校准陀螺仪
        calibrateGyro();
       
        // 开启 DMP
        mpu.setDMPEnabled(true);
        // 获取中断状态
        mpuIntStatus = mpu.getIntStatus();
        // 获取 DMP 包大小
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("DMP ready! Waiting for data...");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }

    delay(1000);

    pitch = 0.0f; roll = 0.0f; yaw = 0.0f; //显示用

}

void loop() {
    //计算dt
    static uint32_t last_t = micros();      //微秒
    uint32_t now_t = micros();
    float dt = (now_t - last_t) * 1e-6f;    // 转换成秒
    last_t = now_t;

    // Must call crsf.update() in loop() to process data
    crsf.update();

    // get Voltage of bettery
    float RxBt = analogReadMilliVolts(ADC_PIN) / 1000.0f;   //analogReadMilliVolts根据analogReadResolution和analogSetAttenuation配置, 直接读电压; 且该函数会自动读取芯片内部 eFuse 里的校准数据进行补偿。
    RxBt = RxBt / 0.3125;   //0.3125是我的电阻分压设置,100k/(100K+220K)=0.3125

    // 先获取所有通道值
    rc.roll_CMD = mapValue(crsf.getChannel(1), 1000, 2000, -1, 1);   //摇杆
    rc.pitch_CMD = mapValue(crsf.getChannel(2), 1000, 2000, -1, 1);  //摇杆
    rc.thr_CMD = crsf.getChannel(3);    //摇杆,保持1000-2000
    rc.yaw_CMD = mapValue(crsf.getChannel(4), 1000, 2000, -1, 1);    //摇杆
    rc.SA_CMD = mapSwitch(crsf.getChannel(5), 2);  //2档
    rc.SB_CMD = mapSwitch(crsf.getChannel(6), 3);  //3档
    rc.SC_CMD = mapSwitch(crsf.getChannel(7), 3);  //3档
    rc.SD_CMD = mapSwitch(crsf.getChannel(8), 2);  //2档
    rc.SE_CMD = mapSwitch(crsf.getChannel(9), 2);  //2档
    rc.S1_CMD = mapValue(crsf.getChannel(10), 1000, 2000, -1, 1);  //拨轮

    flymode = rc.SB_CMD;   //飞行模式由SB档位控制, 0,1,2分别对应不同的飞行模式, 0代表stable模式/只做角速度pid稳定, 1代表angle模式/做pitch和roll的姿态角度串级pid控制, 2代表easy模式/实现pitch,roll,yaw的协同转弯控制, 但目前还没有实现.
    Kp_gain = mapKnobToKp_linear(rc.S1_CMD);   //将拨轮S1的输入(-1,0,1)映射到(0.1,1.0,10)

    // 当无法获得无线电信号时，将所有指令置为0
    if (!crsf.isLinkUp()) {
        rc.roll_CMD = 0.0f;
        rc.pitch_CMD = 0.0f;
        rc.thr_CMD = 1000.0f;
        rc.yaw_CMD = 0.0f;
        rc.SA_CMD = 0;
        rc.SB_CMD = 0;
        rc.SC_CMD = 0;
        rc.SD_CMD = 0;
        rc.SE_CMD = 0;
        rc.S1_CMD = 0.0f;
        flymode = 1;   //失去信号时切换到安全模式, 也可以选择其他模式, 例如直接断电或保持最后的指令不变等.
        Kp_gain = 1.0f;
    }
    // 当有无线电信号时，如果rc.SA_CMD为0，则摇杆指令置为0
    if (crsf.isLinkUp() && rc.SA_CMD == 0) {
        rc.roll_CMD = 0.0f;
        rc.pitch_CMD = 0.0f;
        rc.thr_CMD = 1000.0f;
        rc.yaw_CMD = 0.0f;
    }
    
    /////////////获得四元数和陀螺仪角速度/////////////////
    if (!dmpReady) return;
    // 检查 FIFO 中是否有完整包
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) {
        // 数据还不够一包，稍等
        return;
    }
    //对齐校验.如果缓冲区内的数据不是完整包的整数倍，说明数据流已经错位
    if (fifoCount % packetSize != 0) {
        mpu.resetFIFO(); // 立即清空缓冲区，强制 DMP 重新开始写入对齐的包
        Serial.println("FIFO alignment recovered."); // 调试用
        return; 
    }
    if (fifoCount >= 1024) {
        // FIFO 溢出，清空
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }
    if (fifoCount > 0) {
        // 读取一个完整的 DMP 包
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // 从 DMP 包中解析四元数和重力向量
        mpu.dmpGetQuaternion(&q_current, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q_current);
        mpu.dmpGetYawPitchRoll(ypr, &q_current, &gravity);  // ypr[0] = yaw, ypr[1] = pitch, ypr[2] = roll（单位：弧度）
        yaw   = rad2deg(ypr[0]);
        pitch = rad2deg(ypr[1]);
        roll  = rad2deg(ypr[2]);
    }

    //获取陀螺仪数据
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    // 从LSB 转换为 deg/s
    float gx = gx_raw / 16.4f;
    float gy = gy_raw / 16.4f;
    float gz = gz_raw / 16.4f;
    // 转换为 rad/s（推荐）
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    //gx += 0.02f;       /////////校准后还是总偏-0.02

    //静态变量记录上一次的状态
    static int last_flymode = flymode;
    //边缘检测：如果当前状态与上一次不同，说明开关刚刚被拨动
    if (flymode != last_flymode) {
        // 模式切换瞬间，清空所有积分项和上次误差值，防止 PID “惊跳”
        roll_i = 0.0f;   roll_last = 0.0f;
        pitch_i = 0.0f;  pitch_last = 0.0f;
        yaw_i = 0.0f;    yaw_last = 0.0f;
        // 调试打印，确认切换成功
        Serial.println("Mode Switched: Integral Reset.");        
        // 更新旧状态记录
        last_flymode = flymode;
        
    }

    //从q_current获得当前姿态的参数
    current_yaw = atan2f(
        2.0f * (q_current.w * q_current.z + q_current.x * q_current.y),
        1.0f - 2.0f * (q_current.y * q_current.y + q_current.z * q_current.z)
    );  //当前航向角（单位：弧度）
    current_q_tilt = removeYaw(q_current);

    ///////////根据摇杆输入构造q_target///////////////////////////

    roll_target  = rc.roll_CMD  * max_roll;   //摇杆输入（-1.0 ~ +1.0）转为rad角度.
    pitch_target = rc.pitch_CMD * max_pitch;  //摇杆输入（-1.0 ~ +1.0）转为rad角度
    
    if(flymode == 2){
        yaw_target = yaw_target - rc.yaw_CMD * 0.01f;   // 每次循环累积一点;//动态更新航向锁定点：航向随转向摇杆漂移
    }
    if(flymode == 1){
        yaw_target = current_yaw; //让目标yaw始终跟随当前yaw. q_err是构造外环误差的, 而遥控器输入在内环.
    }
    if(flymode == 0){
        //yaw_target = current_yaw; //这句其实没用,后面q_target = q_current;
    } 
    
    //不再需要
    //if (yaw_target > M_PI)  yaw_target -= 2.0f * M_PI;
    //if (yaw_target < -M_PI) yaw_target += 2.0f * M_PI;

    //输入转目标四元数(欧拉角 → 四元数)
    if(flymode == 2 || flymode == 1){
        //q_target = eulerToQuaternion(roll_target, pitch_target, yaw_target);
        target_q_tile = tiltFromStick(roll_target, pitch_target);
        target_q_yaw = Quaternion(cosf(yaw_target/2), 0, 0, sinf(yaw_target/2));
        q_target = target_q_yaw.getProduct(target_q_tile);


    }else{
        q_target = q_current;
    }

    ///////////进行PID计算得到姿态控制输出////////////////////////
    float err_pitch_rate, err_roll_rate, err_yaw_rate, u_roll, u_pitch, u_yaw;
    //得到姿态控制输出u_roll, u_pitch, u_yaw, 这三个值似乎是机体坐标系
    attitudeControlStep(q_target, q_current, gx, gy, gz, dt, err_pitch_rate, err_roll_rate, err_yaw_rate, u_roll, u_pitch, u_yaw);  // 传入目标四元数、当前四元数、角速度、时间间隔、误差、控制输出

    float final_u_roll  = u_roll;
    float final_u_pitch = u_pitch;
    float final_u_yaw   = u_yaw;

    if (flymode == 2) {
        // --- 协同逻辑 A: 偏航带横滚. 当你打 Yaw 摇杆时，直接给 Roll 输出加一个补偿量
        //final_u_roll = final_u_roll + rc.yaw_CMD * 50.0f;
        final_u_roll = 0.5f * u_roll - 0.5f * u_yaw;    // 假设 rc.yaw_CMD 左打为正，左滚为负，则用减号

        // --- 协同逻辑 B: 横滚带俯仰 (自动抬头) : 只要有横滚角（无论左倾右倾），就给俯仰输出加一个“抬头”量
        float pitch_comp = (1.0f - cosf(yaw_target)) * 80.0f;
        final_u_pitch = u_pitch - pitch_comp; // 减法代表抬头.
    }

    //输出硬件LEDC PWM
    ledcWrite(LEDC_Throttle_PIN, mapToLedc(rc.thr_CMD, 1000, 2000, LEDC_FREQ_400, 1000, 2000)); 
    ledcWrite(LEDC_Roll1_PIN,    mapToLedc(final_u_roll,  -300, 300, LEDC_FREQ_50, 500, 2500));
    ledcWrite(LEDC_Roll2_PIN,    mapToLedc(final_u_roll,  -300, 300, LEDC_FREQ_50, 500, 2500));
    ledcWrite(LEDC_Pitch_PIN,    mapToLedc(final_u_pitch, -300, 300, LEDC_FREQ_50, 500, 2500));
    ledcWrite(LEDC_Yaw_PIN,      mapToLedc(final_u_yaw,   -300, 300, LEDC_FREQ_50, 500, 2500));

    //打印部分, 定期执行
    static uint32_t lastTick = 0;
    if (millis() - lastTick > 50) {

        //Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        //            pitch, roll, yaw, gx, gy, gz, err_pitch_rate, err_roll_rate, err_yaw_rate, u_pitch, u_roll, u_yaw, Kp_gain);
        Serial.printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            q_target.w, q_target.x, q_target.y, q_target.z, q_current.w, q_current.x, q_current.y, q_current.z, gx, gy, gz, err_pitch_rate, err_roll_rate, err_yaw_rate, final_u_pitch, final_u_roll, final_u_yaw, Kp_gain);

        lastTick = millis();
    }
    

    //Serial.print("\n");
}

//////////////////////////////////////////////////////////////////////////////
//////////////////FUNCTION////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// ---------------------------


//四元数串级 PID
void attitudeControlStep(
    Quaternion& q_target,
    Quaternion& q_current,
    float gx, float gy, float gz,
    float dt,
    float& err_pitch_rate, float& err_roll_rate, float& err_yaw_rate,
    float& u_roll, float& u_pitch, float& u_yaw)
{   
    
    // 1. 四元数姿态误差
    //
    //Quaternion q_err = q_target.getProduct(q_current.getConjugate());
    Quaternion q_err = q_current.getConjugate().getProduct(q_target);   //

    //确保误差永远取最短路径
    // 如果 w < 0，说明旋转超过了 180 度，我们取其相反数
    if (q_err.w < 0) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // 2. 姿态误差向量（目标角速度）
    float ex = 2.0f * q_err.x;  //sin(θ/2) 最大是 1，所以 ex 理论最大可以到 2. 
    float ey = 2.0f * q_err.y;
    float ez = 2.0f * q_err.z;
    //Serial.printf("%.4f,%.4f,%.4f ",ex, ey, ez);
    err_roll_rate = 0.0;
    err_pitch_rate = 0.0;
    err_yaw_rate  = 0.0;


    // 3. 角速度误差
    if (flymode == 2)   //easy模式启用协同转弯控制
    {
        //对于此模式,输入的q_target已经在函数外调整了
        //PID外环启用
        err_roll_rate  = ex * 10.0f - gx;
        err_pitch_rate = ey * 10.0f - gy;
        err_yaw_rate   = ez * 10.0f - gz;     //锁航向的话, 切换模式时有跳变, 目前无法解决.

    }else if (flymode == 1)   //easy和stable模式启用外环PID
    {   //对于ROll和PITCH, PID外环启用; YAW不启用外环
        err_roll_rate  = ex * 10.0f - gx;   //ex的范围在[-2.0, 2.0]之间,要给 ex 乘以一个比例系数。
        err_pitch_rate = ey * 10.0f - gy;
        err_yaw_rate   = -rc.yaw_CMD * 10.0f - gz;  //这里的外环控制目标是roll/pitch的角度，但yaw的外环控制目标是航向锁定（yaw_target），在当前的模式下不能进行航向锁定(否则只能使用yaw转弯),所以只用内环.

    }else if (flymode == 0)   //stable模式禁用外环
    {
        //禁用外环的情况下,角速度误差直接使用摇杆输入
        err_roll_rate  = rc.roll_CMD  * 10.0f - gx;      // 实际飞行中通常在 0-10 rad/s波动，极限可达±35rad/s
        err_pitch_rate = rc.pitch_CMD * 10.0f - gy;
        err_yaw_rate   = -rc.yaw_CMD   * 10.0f - gz;

    }
    //Serial.printf("%.4f,%.1f,%.1f\n",err_roll_rate, err_pitch_rate, err_yaw_rate);
    
    // 4. Rate PID（函数版）
    u_roll = pid_update(err_roll_rate, dt,
                        roll_i, roll_last,
                        Kp_gain * 15.0f, 0.1f, 0.02f,  //kp,ki,kd
                        -300.0f, 300.0f);

    u_pitch = pid_update(err_pitch_rate, dt,
                         pitch_i, pitch_last,
                         Kp_gain * 15.0f, 0.1f, 0.02f,  //kp,ki,kd
                         -300.0f, 300.0f);

    u_yaw = pid_update(err_yaw_rate, dt,
                       yaw_i, yaw_last,
                       Kp_gain * 10.0f, 0.0f, 0.01f,  //kp,ki,kd
                       -300.0f, 300.0f);
}



float pid_update(float err, float dt,
                 float &i_term, float &last_err,
                 float kp, float ki, float kd,
                 float out_min, float out_max)
{
    // 积分
    i_term += err * dt;
    // 抗积分饱和
    if (i_term > out_max/4.0f) i_term = out_max/4.0f;
    if (i_term < out_min/4.0f) i_term = out_min/4.0f;

    // --- 核心修正：处理最短路径导致的微分跳变 ---
    float delta_err = err - last_err;
    
    // 如果误差变化量极大（超过了物理上可能的转速），说明发生了正负号翻转
    // 我们强制让 delta_err 趋于 0，避免 D 项爆炸
    if (delta_err > 3.0f)  delta_err -= 4.0f; 
    if (delta_err < -3.0f) delta_err += 4.0f;

    // 微分计算使用修正后的 delta_err
    float d = delta_err / dt;
    last_err = err;

    // PID 输出
    float out = kp * err + ki * i_term + kd * d;

    // 输出限幅
    if (out > out_max) out = out_max;
    if (out < out_min) out = out_min;

    return out;
}




//欧拉角 → 四元数函数
Quaternion eulerToQuaternion(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

// 工具函数：弧度转角度
inline float rad2deg(float r) {
    return r * 180.0f / M_PI;
}


float mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int mapSwitch(uint16_t ch, int positions)
{
    if (positions == 2)
    {
        return (ch < 1500) ? 0 : 1;
    }
    else if (positions == 3)
    {
        if (ch < 1300) return 0;
        else if (ch < 1700) return 1;
        else return 2;
    }
    return -1; // 错误
}

// mymap函数：将输入值从一个范围线性映射到另一个范围，并返回uint32_t类型结果
// 参数：
//   value: 输入值
//   in_min: 输入范围的最小值
//   in_max: 输入范围的最大值
//   out_min: 输出范围的最小值
//   out_max: 输出范围的最大值
// 返回值：映射后的uint32_t类型值
uint32_t mymap(float value, float in_min, float in_max, float out_min, float out_max)
{
    // 限制输入值在输入范围内
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;

    // 映射到输出范围
    float result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    // 返回uint32_t类型
    return (uint32_t)result;
}

float mapKnobToKp_linear(float knob)
{
    if (knob >  1.0f) knob =  1.0f;
    if (knob < -1.0f) knob = -1.0f;

    const float P_min = 0.1f;
    const float P_mid = 1.0f;
    const float P_max = 10.0f;

    if (knob < 0)
    {
        // [-1,0] → [0.1,1]
        float u = (knob + 1.0f);   // -1→0, 0→1
        return P_min + (P_mid - P_min) * u;
    }
    else
    {
        // [0,1] → [1,10]
        float u = knob;            // 0→0, 1→1
        return P_mid + (P_max - P_mid) * u;
    }
}


/**
 * @brief 将输入映射为 12位 LEDC 占空比数值
 * 
 * @param IN        输入值
 * @param a         输入下限
 * @param b         输入上限
 * @param LEDC_FREQ PWM 频率 (Hz)
 * @param x         电调/舵机接受的最小脉宽 (微秒)
 * @param y         电调/舵机接受的最大脉宽 (微秒)
 * @return uint32_t 返回 0-4095 之间的 LEDC 占空比数值
 */
uint32_t mapToLedc(float IN, float a, float b, uint32_t LEDC_FREQ, float x, float y) {
    // 1. 约束输入范围
    if (IN < a) IN = a;
    if (IN > b) IN = b;

    // 2. 线性映射：将 IN 映射到 [x, y] 微秒脉宽
    // 公式: target = x + (IN - a) * (y - x) / (b - a)
    float targetPulseWidth = x + (IN - a) * (y - x) / (b - a);

    // 3. 计算当前频率下一个周期的总微秒数
    // 例如 50Hz, 则周期为 20000 微秒
    float periodUs = 1000000.0 / LEDC_FREQ;

    // 4. 将脉宽时间转换为 12 位分辨率 (0-4095) 的数值
    // 公式: (目标脉宽 / 周期) * 4095
    float outFloat = (targetPulseWidth / periodUs) * 4095.0;

    // 5. 最终限幅并转为整数
    uint32_t OUT = (uint32_t)outFloat;
    if (OUT > 4095) OUT = 4095;

    return OUT;
}


// 完整的传感器校准：包含陀螺仪和加速度计
void calibrateSensors() {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t ax, ay, az, gx, gy, gz;

    Serial.println("Calibrating... Ensure device is level and STILL.");

    const int samples = 1000; 
    for (int i = 0; i < samples; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; ay_sum += ay; az_sum += az;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delay(2);
    }

    // --- 核心修正点 ---
    // 硬件 Offset = 寄存器当前值 + (目标值 - 测量均值)
    // 假设当前寄存器 Offset 为 0，则：
    int16_t ax_off = - (ax_sum / samples);
    int16_t ay_off = - (ay_sum / samples);
    int16_t az_off = (8192 - (az_sum / samples)); // 修正为：目标(8192) - 测量值

    // 陀螺仪目标均为 0
    int16_t gx_off = - (gx_sum / samples);
    int16_t gy_off = - (gy_sum / samples);
    int16_t gz_off = - (gz_sum / samples);

    mpu.setXAccelOffset(ax_off);
    mpu.setYAccelOffset(ay_off);
    mpu.setZAccelOffset(az_off);
    mpu.setXGyroOffset(gx_off);
    mpu.setYGyroOffset(gy_off);
    mpu.setZGyroOffset(gz_off);

    Serial.println("Calibration Done.");
    //注意:关于 8192 精度：你的代码中设置了 mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4)。在 ±4g 量程下，1g 对应的数字量是 8192。如果你之后改为 ±2g，这个数字需要改为 16384。
}



//陀螺仪自动校准代码
void calibrateGyro() {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gx, gy, gz;

    Serial.println("Calibrating gyro... Keep the device absolutely still!");

    // 采样次数
    const int samples = 2000;

    for (int i = 0; i < samples; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(2);  // 500 Hz 采样
    }

    int16_t gx_offset = -(gx_sum / samples);
    int16_t gy_offset = -(gy_sum / samples);
    int16_t gz_offset = -(gz_sum / samples);

    // 写入 offset
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    Serial.println("Gyro calibration done!");
    Serial.print("Offsets: ");
    Serial.print(gx_offset); Serial.print(", ");
    Serial.print(gy_offset); Serial.print(", ");
    Serial.println(gz_offset);
}

Quaternion removeYaw(Quaternion& q)
{
    // 提取当前 yaw
    float yaw = atan2f(
        2.0f * (q.w * q.z + q.x * q.y),
        1.0f - 2.0f * (q.y * q.y + q.z * q.z)
    );

    // 构造当前 yaw 的四元数
    float hy = yaw * 0.5f;
    Quaternion q_yaw(cosf(hy), 0, 0, sinf(hy));

    Quaternion q_yaw_conj(q_yaw.w, -q_yaw.x, -q_yaw.y, -q_yaw.z);

    // 去掉 yaw：q_no_yaw = q * conj(q_yaw)
    Quaternion q_no_yaw = q.getProduct(q_yaw_conj);
    return q_no_yaw;
}





Quaternion tiltFromStick(float roll_cmd, float pitch_cmd)
//从摇杆的pitch和roll构造tile四元数(不带yaw)
{
    float roll  = roll_cmd  * max_roll;
    float pitch = pitch_cmd * max_pitch;

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    // 注意：这里 roll/pitch 是机体系旋转
    return Quaternion(
        cr*cp,
        sr*cp,
        cr*sp,
        -sr*sp
    );
}


/*

//pid算法
float pid_update_old(float err, float dt,
                 float &i_term, float &last_err,
                 float kp, float ki, float kd,
                 float out_min, float out_max)
{
    // 积分
    i_term += err * dt;

    // 抗积分饱和
    if (i_term > out_max/4.0f) i_term = out_max/4.0f;     //积分限幅为pid输出限幅的1/4
    if (i_term < out_min/4.0f) i_term = out_min/4.0f;

    // 微分
    float d = (err - last_err) / dt;
    last_err = err;

    // PID 输出
    float out = kp * err + ki * i_term + kd * d;

    // 输出限幅
    if (out > out_max) out = out_max;
    if (out < out_min) out = out_min;

    return out;
}
    */