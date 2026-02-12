#include <Wire.h>
#include <math.h>

#include "config.h"
#include "ACS712.h"
#include "Motor.h"
#include "Encoder.h"
#include "IMUManager.h"
#include "LoadCell.h"

// =====================================================
// 4) インスタンス設定（元コード踏襲）
// =====================================================
Motor motorL(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM);
Motor motorR(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM);

// ロードセル (DOUT, SCK, 係数)
LoadCell handleL(27, 29, 74.64f);
LoadCell handleR(23, 25, 74.64f);

// エンコーダ (PPR, 直径, 反転)
Encoder encL(200.0, 0.065, false);
Encoder encR(200.0, 0.065, true);

// 電流センサ（君の設定を踏襲）
// (ピン, 感度, オフセット, 平均回数, KT定数)
// ※ KT_VALUE がクラス内で何に使われているかは未確認なので据え置き
const double KT_VALUE = 2.7;
ACS712 sensorL(PIN_CUR_L, 13.0, 512.0, 9, KT_VALUE);
ACS712 sensorR(PIN_CUR_R, 13.0, 512.0, 9, KT_VALUE);

// IMU
IMUManager imu;

// =====================================================
// 5) 共有変数（割り込みとloopで共有）
// =====================================================
volatile float fL_filt_g = 0.0f;
volatile float fR_filt_g = 0.0f;

volatile float iL_filt_A = 0.0f;
volatile float iR_filt_A = 0.0f;

volatile float iL_ref_A = 0.0f;
volatile float iR_ref_A = 0.0f;

volatile int pwmL_cmd = 0;
volatile int pwmR_cmd = 0;

volatile float iL_integ = 0.0f;
volatile float iR_integ = 0.0f;

// 追加：ログ用の「押力」と「推定トルク」(制御loopで更新)
volatile float push_mag_N = 0.0f;
volatile int   push_dir   = 0;     // -1:後退, 0:無し, +1:前進

volatile float tauMotorL_est_Nm = 0.0f;
volatile float tauMotorR_est_Nm = 0.0f;
volatile float tauTireL_est_Nm  = 0.0f;
volatile float tauTireR_est_Nm  = 0.0f;


volatile float fL_hold_g = 0.0f, fR_hold_g = 0.0f;

volatile float IrefL_hold = 0.0f, IrefR_hold = 0.0f;
volatile float pwm_ff_L_hold = 0.0f, pwm_ff_R_hold = 0.0f;

volatile float pwm_trim_L = 0.0f, pwm_trim_R = 0.0f;
volatile float pwm_cmd_L_hold = 0.0f, pwm_cmd_R_hold = 0.0f;

volatile bool drive_enable = false;

volatile float fL_zero_g = 0.0f;
volatile float fR_zero_g = 0.0f;




// =====================================================
// エンコーダ割り込み
// =====================================================
void isrL() { encL.tick(digitalRead(PIN_ENC_L_B) == LOW); }
void isrR() { encR.tick(digitalRead(PIN_ENC_R_B) == LOW); }

// =====================================================
// ユーティリティ：符号付きデッドゾーン
// =====================================================
static inline float applyDeadzoneSigned(float x, float dz) {
  if (x > dz) return x - dz;
  if (x < -dz) return x + dz;
  return 0.0f;
}

// =====================================================
// 8) 核：荷重 -> 目標電流 -> 電流PI -> PWM
// =====================================================
void controlLoop() {
  static uint8_t lc_div = 0;

  // ========= 500Hz：保持PWMを出力（ゲート込み） =========
  if (!drive_enable) {
    motorL.drive(0);
    motorR.drive(0);
  } else {
    motorL.drive((int)pwm_cmd_L_hold);
    motorR.drive((int)pwm_cmd_R_hold);
  }

  // ========= 80Hz：6回に1回だけ更新 =========
  lc_div++;
  if (lc_div < 6) return;
  lc_div = 0;

  // ---- ロードセル取得＆LPF（80Hz側） ----
  float fL_raw = handleL.getForce();
  float fR_raw = handleR.getForce();

  fL_hold_g = (fL_raw * LPF_ALPHA_F) + (fL_hold_g * (1.0f - LPF_ALPHA_F));
  fR_hold_g = (fR_raw * LPF_ALPHA_F) + (fR_hold_g * (1.0f - LPF_ALPHA_F));

  // =====================================================
  // ドリフト対策：ゼロ点引き算 + 無入力時ゼロ追従
  // =====================================================
  float fL_corr_g = fL_hold_g - fL_zero_g;
  float fR_corr_g = fR_hold_g - fR_zero_g;

  float fL_eff_g = applyDeadzoneSigned(fL_corr_g, FORCE_DEADZONE_G);
  float fR_eff_g = applyDeadzoneSigned(fR_corr_g, FORCE_DEADZONE_G);

  // ---- 無入力時のゼロ追従条件 ----
  const float ZERO_TRACK_TH_G = 30.0f;
  bool noInputForZero = (fabs(fL_corr_g) < ZERO_TRACK_TH_G) &&
                        (fabs(fR_corr_g) < ZERO_TRACK_TH_G);

  if (noInputForZero) {
    const float ZERO_ALPHA = 0.002f;
    fL_zero_g = (fL_hold_g * ZERO_ALPHA) + (fL_zero_g * (1.0f - ZERO_ALPHA));
    fR_zero_g = (fR_hold_g * ZERO_ALPHA) + (fR_zero_g * (1.0f - ZERO_ALPHA));
  }

  // =====================================================
  // 入力ゲート（連続0でOFF）
  // =====================================================
  static uint8_t noInputCnt = 0;
  bool noInput = (fabs(fL_eff_g) < 1e-6f) && (fabs(fR_eff_g) < 1e-6f);

  if (noInput) {
    if (++noInputCnt >= 10) {
      drive_enable = false;
      pwm_trim_L = pwm_trim_R = 0.0f;
      pwm_cmd_L_hold = pwm_cmd_R_hold = 0.0f;
      IrefL_hold = IrefR_hold = 0.0f;
      noInputCnt = 0;
      return;
    }
  } else {
    noInputCnt = 0;
    drive_enable = true;
  }

  // ---- 外側：g -> F[N] ----
  float F_L_ref_N = K_F * ( fL_eff_g);
  float F_R_ref_N = K_F * (-fR_eff_g);

  // ---- F -> Iref ----
  float I_L = (F_L_ref_N * R_ROLLER / GEAR) / KT;
  float I_R = (F_R_ref_N * R_ROLLER / GEAR) / KT;

  if (fabs(I_L) < I_DEADZONE) I_L = 0.0f;
  if (fabs(I_R) < I_DEADZONE) I_R = 0.0f;

  I_L = constrain(I_L, -I_REF_LIMIT, I_REF_LIMIT);
  I_R = constrain(I_R, -I_REF_LIMIT, I_REF_LIMIT);

  IrefL_hold = I_L;
  IrefR_hold = I_R;

  // ---- 電流計測＆LPF ----
  float iL_meas = (float)sensorL.readCurrent();
  float iR_meas = (float)sensorR.readCurrent();

  iL_filt_A = (iL_meas * LPF_ALPHA_I) + (iL_filt_A * (1.0f - LPF_ALPHA_I));
  iR_filt_A = (iR_meas * LPF_ALPHA_I) + (iR_filt_A * (1.0f - LPF_ALPHA_I));

  float IabsL = fabs(iL_filt_A);
  float IabsR = fabs(iR_filt_A);

  // ---- メイン：PWMフィードフォワード ----
  float pwm_ff_L = K_PWM_I * I_L;
  float pwm_ff_R = K_PWM_I * I_R;

  // ---- サブ：不足分だけ補正（trim） ----
  const float Ts_trim = Ts * 6.0f;

  float eL = fabs(I_L) - IabsL;
  float eR = fabs(I_R) - IabsR;

  pwm_trim_L += KI_TRIM * eL * Ts_trim;
  pwm_trim_R += KI_TRIM * eR * Ts_trim;

  pwm_trim_L = constrain(pwm_trim_L, 0.0f, PWM_TRIM_MAX);
  pwm_trim_R = constrain(pwm_trim_R, 0.0f, PWM_TRIM_MAX);

  // ---- 合成PWM ----
  float pwm_cmd_L = pwm_ff_L + (I_L >= 0 ? +pwm_trim_L : -pwm_trim_L);
  float pwm_cmd_R = pwm_ff_R + (I_R >= 0 ? +pwm_trim_R : -pwm_trim_R);

  pwm_cmd_L = constrain(pwm_cmd_L, -PWM_LIMIT, PWM_LIMIT);
  pwm_cmd_R = constrain(pwm_cmd_R, -PWM_LIMIT, PWM_LIMIT);

  // ---- 低PWMガタ防止 ----
  const int PWM_MIN = 25;
  if (pwm_cmd_L > 0) pwm_cmd_L = max(pwm_cmd_L, (float)PWM_MIN);
  if (pwm_cmd_L < 0) pwm_cmd_L = min(pwm_cmd_L, (float)-PWM_MIN);
  if (pwm_cmd_R > 0) pwm_cmd_R = max(pwm_cmd_R, (float)PWM_MIN);
  if (pwm_cmd_R < 0) pwm_cmd_R = min(pwm_cmd_R, (float)-PWM_MIN);

  pwm_cmd_L_hold = pwm_cmd_L;
  pwm_cmd_R_hold = pwm_cmd_R;
}




// =====================================================
// 6) 500Hzタイマー割り込みループ
// =====================================================
ISR(TIMER4_COMPA_vect) {
  controlLoop();
}

// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.begin();
  handleL.begin();
  handleR.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrR, RISING);

  // --- Timer4 (500Hz) ---
  // 16MHz / 64 = 250kHz
  // 250kHz / 500Hz = 500 -> OCR4A = 499
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 249;
  TCCR4B |= (1 << WGM42);              // CTC
  TCCR4B |= (1 << CS41) | (1 << CS40); // prescaler 64
  TIMSK4 |= (1 << OCIE4A);             // enable compare match
  interrupts();

  if (ENABLE_LOG) {
    // Serial Plotterでも見やすい用（CSV列固定）
    Serial.println("t_s,push_mag_N,push_dir,tauMotorL_Nm,tauMotorR_Nm,tauTireL_Nm,tauTireR_Nm,IrefL_A,ImeasL_A,pwmL,IrefR_A,ImeasR_A,pwmR");
  }

  Serial.println("System Ready (Current/Torque Control @500Hz)");
}

// =====================================================
// loop：ロギング（押力Nとトルク推定が主役）
// =====================================================
void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 50) {   // 20Hzで表示
    lastPrint = now;

    // 80Hz側で更新している保持値をそのまま表示
    Serial.print(fL_hold_g, 2);
    Serial.print(",");
    Serial.println(fR_hold_g, 2);
  }
}

