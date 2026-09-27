#include "motor_driver.h"
#include <math.h>

static Adafruit_PWMServoDriver g_pwmDriver(PCA9685_I2C_ADDR);
static RoverMotorState g_state = {};

static void applyWheelOutput(uint8_t in1_ch, uint8_t in2_ch, float v, WheelStatus &ws) {
    ws.target_v = v;

    if (v > 0.001f) {
        ws.direction = 1; // 正転
        uint32_t raw_pwm = (uint32_t)(v * (float)PWM_DUTY_MAX_LIMIT + 0.5f);
        if (raw_pwm > PWM_DUTY_MAX_LIMIT) raw_pwm = PWM_DUTY_MAX_LIMIT;
        ws.pwm_val = (uint16_t)raw_pwm;

        if (g_state.initialized) {
            g_pwmDriver.setPWM(in1_ch, 0, ws.pwm_val);
            g_pwmDriver.setPWM(in2_ch, 0, 0);
        }
    } else if (v < -0.001f) {
        ws.direction = -1; // 逆転
        uint32_t raw_pwm = (uint32_t)(-v * (float)PWM_DUTY_MAX_LIMIT + 0.5f);
        if (raw_pwm > PWM_DUTY_MAX_LIMIT) raw_pwm = PWM_DUTY_MAX_LIMIT;
        ws.pwm_val = (uint16_t)raw_pwm;

        if (g_state.initialized) {
            g_pwmDriver.setPWM(in1_ch, 0, 0);
            g_pwmDriver.setPWM(in2_ch, 0, ws.pwm_val);
        }
    } else {
        ws.direction = 0; // 停止
        ws.pwm_val = 0;

        if (g_state.initialized) {
            g_pwmDriver.setPWM(in1_ch, 0, 0);
            g_pwmDriver.setPWM(in2_ch, 0, 0);
        }
    }
}

bool initMotorDriver(TwoWire &wireInstance) {
    // PCA9685 通信確認
    wireInstance.beginTransmission(PCA9685_I2C_ADDR);
    if (wireInstance.endTransmission() != 0) {
        g_state.initialized = false;
        return false;
    }

    g_pwmDriver = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR, wireInstance);
    g_pwmDriver.begin();
    g_pwmDriver.setPWMFreq(PCA9685_PWM_FREQ);

    g_state.initialized = true;

    // 全チャンネル初期化（停止）
    stopMotors();

    return true;
}

void setMotorVelocities(float vx, float vy, float omega) {
    // 指令値のクランプ (-1.0 〜 1.0)
    vx = constrain(vx, -1.0f, 1.0f);
    vy = constrain(vy, -1.0f, 1.0f);
    omega = constrain(omega, -1.0f, 1.0f);

    g_state.current_vx = vx;
    g_state.current_vy = vy;
    g_state.current_omega = omega;

    // メカナム運動学の計算
    // FL =  Vy + Vx - Omega
    // FR =  Vy - Vx + Omega
    // RL =  Vy - Vx - Omega
    // RR =  Vy + Vx + Omega
    float v_fl =  vy + vx - omega;
    float v_fr =  vy - vx + omega;
    float v_rl =  vy - vx - omega;
    float v_rr =  vy + vx + omega;

    // 最大値正規化処理
    float max_val = fabsf(v_fl);
    if (fabsf(v_fr) > max_val) max_val = fabsf(v_fr);
    if (fabsf(v_rl) > max_val) max_val = fabsf(v_rl);
    if (fabsf(v_rr) > max_val) max_val = fabsf(v_rr);

    if (max_val > 1.0f) {
        v_fl /= max_val;
        v_fr /= max_val;
        v_rl /= max_val;
        v_rr /= max_val;
    }

    // 各輪への適用
    applyWheelOutput(CH_FL_IN1, CH_FL_IN2, v_fl, g_state.fl);
    applyWheelOutput(CH_FR_IN3, CH_FR_IN4, v_fr, g_state.fr);
    applyWheelOutput(CH_RL_IN1, CH_RL_IN2, v_rl, g_state.rl);
    applyWheelOutput(CH_RR_IN3, CH_RR_IN4, v_rr, g_state.rr);
}

void stopMotors() {
    g_state.current_vx = 0.0f;
    g_state.current_vy = 0.0f;
    g_state.current_omega = 0.0f;

    applyWheelOutput(CH_FL_IN1, CH_FL_IN2, 0.0f, g_state.fl);
    applyWheelOutput(CH_FR_IN3, CH_FR_IN4, 0.0f, g_state.fr);
    applyWheelOutput(CH_RL_IN1, CH_RL_IN2, 0.0f, g_state.rl);
    applyWheelOutput(CH_RR_IN3, CH_RR_IN4, 0.0f, g_state.rr);
}

const RoverMotorState& getMotorState() {
    return g_state;
}

