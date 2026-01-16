#include <Wire.h>
#include "ACS712.h"
#include "Motor.h"
#include "Encoder.h"
#include "IMUManager.h"
#include "LoadCell.h"

// =====================================================
// 1) 物理・電気パラメータ（君の仕様ベース）
// =====================================================
const float Ts = 0.01f;              // 100Hz -> 0.01s
const float VBUS = 24.0f;            // 24V
const int   PWM_MAX_HW = 255;        // Arduino PWM最大

// 駆動系：Φ60ローラがΦ300タイヤに接触
const float R_TIRE   = 0.150f;       // タイヤ半径 [m] (Φ300)
const float R_ROLLER = 0.030f;       // ローラ半径 [m] (Φ60)
const float GEAR     = 10.0f;        // ギア比 10:1（モータ->出力軸）

// 推定トルク定数 kt [Nm/A]
// 60W/24V -> 2.5A程度、ギア後トルク1.35Nm -> ギア前0.135Nm なので
// kt ≈ 0.135/2.5 = 0.054 Nm/A（目安）
const float KT = 0.054f;             // [Nm/A] まずはこの値で開始（要最終調整）

// =====================================================
// 2) アシスト設計パラメータ
// =====================================================
// ロードセル出力は「g」っぽい想定（君の元コードを踏襲）
// g -> N 変換
const float G2N = 9.80665f / 1000.0f;   // [N/g]

// 荷重→推進力のゲイン（核の外側）
// 例：ハンドル1000gで推進力20N くらい出したいなら 20N/(1000g)=0.02
const float K_F = 0.02f;               // [N/g] ★調整ポイント

const float FORCE_DEADZONE_G = 50.0f; // [g] デッドゾーン（君の値）
const float LPF_ALPHA_F      = 0.1f;   // 荷重LPF

// 目標電流のデッドゾーン
const float I_DEADZONE = 0.15f;        // [A] ★ノイズで微小駆動しないため

// =====================================================
// 3) 電流PI（トルク制御の核）
// =====================================================
// まずは「安定寄り」初期値（あとで上げる）
const float KP_I = 60.0f;              // [PWM/A] ★調整ポイント
const float KI_I = 200.0f;             // [PWM/(A*s)] ★調整ポイント

// 電流LPF（ACS712がバタつくなら）
const float LPF_ALPHA_I = 0.3f;

// 安全制限
const int PWM_LIMIT = 200;             // 君の安全制限を踏襲
const float I_REF_LIMIT = 3.0f;         // [A] まずは控えめ（定格2.5A近辺）

// =====================================================
// 4) インスタンス設定
// =====================================================
// モーター (DIR, PWM)
Motor motorL(12, 11);
Motor motorR(10, 9);

// ロードセル (DOUT, SCK, 係数)
LoadCell handleL(27, 29, 74.64f);
LoadCell handleR(23, 25, 74.64f);

// エンコーダ (PPR, 直径, 反転)
Encoder encL(200.0, 0.065, false);
Encoder encR(200.0, 0.065, true);

// 電流センサ（君の設定を踏襲）
// (ピン, 感度, オフセット, 平均回数, KT定数)
const double KT_VALUE = 2.7l;  // ※このKT_VALUEはACS712クラス内で何に使っているか要確認
ACS712 sensorL(A3, 13.0, 512.0, 9, KT_VALUE);
ACS712 sensorR(A2, 13.0, 512.0, 9, KT_VALUE);

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

// PIの積分（PWM単位で持つ：u = KP*e + integ）
volatile float iL_integ = 0.0f;
volatile float iR_integ = 0.0f;

// エンコーダ割り込み用ブリッジ
void isrL() { encL.tick(digitalRead(4) == LOW); }
void isrR() { encR.tick(digitalRead(6) == LOW); }

// =====================================================
// 6) 100Hzタイマー割り込みループ
// =====================================================
ISR(TIMER4_COMPA_vect) {
  controlLoop();
}

// =====================================================
// 7) ユーティリティ：符号付きデッドゾーン
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
  // =================================================
  // ① ロードセル（生値 g）
  // =================================================
  float fL_raw_g = handleL.getForce();
  float fR_raw_g = handleR.getForce();

  // =================================================
  // ② ロードセルLPF（EMA）
  // =================================================
  fL_filt_g = (fL_raw_g * LPF_ALPHA_F) + (fL_filt_g * (1.0f - LPF_ALPHA_F));
  fR_filt_g = (fR_raw_g * LPF_ALPHA_F) + (fR_filt_g * (1.0f - LPF_ALPHA_F));

  // =================================================
  // ③ デッドゾーン（±500g以下は0、超えた分だけ通す）
  // =================================================
  float fL_eff_g = applyDeadzoneSigned(fL_filt_g, FORCE_DEADZONE_G);
  float fR_eff_g = applyDeadzoneSigned(fR_filt_g, FORCE_DEADZONE_G);

  // ★ここ追加：無操作なら完全停止（積分もクリア）
  const float STOP_EPS_G = 50.0f; // デッドゾーン後の残りがこれ以下なら無操作扱い
  if (fabs(fL_eff_g) < STOP_EPS_G && fabs(fR_eff_g) < STOP_EPS_G) {
    iL_ref_A = 0.0f;
    iR_ref_A = 0.0f;
    iL_integ = 0.0f;
    iR_integ = 0.0f;
    pwmL_cmd = 0;
    pwmR_cmd = 0;
    motorL.drive(0);
    motorR.drive(0);
    return;
  }
  // =================================================
  // ④ 外側：荷重(g) -> 推進力F[N] -> 目標電流Iref[A]
  // =================================================
  // 推進力（右は元コード踏襲で符号反転して前進方向を揃える）
  float F_L_ref_N = K_F * (fL_eff_g);     // [N]
  float F_R_ref_N = K_F * (-fR_eff_g);    // [N]

  // Iref換算：
  // tau_motor = F * R_ROLLER / GEAR
  // Iref = tau_motor / KT
  float I_L = (F_L_ref_N * R_ROLLER / GEAR) / KT;  // [A]
  float I_R = (F_R_ref_N * R_ROLLER / GEAR) / KT;  // [A]

  // 微小電流は0に潰す
  if (fabs(I_L) < I_DEADZONE) I_L = 0.0f;
  if (fabs(I_R) < I_DEADZONE) I_R = 0.0f;

  // 安全上限
  I_L = constrain(I_L, -I_REF_LIMIT, I_REF_LIMIT);
  I_R = constrain(I_R, -I_REF_LIMIT, I_REF_LIMIT);

  iL_ref_A = I_L;
  iR_ref_A = I_R;

  // =================================================
  // ⑤ 電流計測（ACS712）+ LPF
  // =================================================
  float iL_meas = (float)sensorL.readCurrent();  // [A]（符号が信用できない想定）
  float iR_meas = (float)sensorR.readCurrent();  // [A]

  iL_filt_A = (iL_meas * LPF_ALPHA_I) + (iL_filt_A * (1.0f - LPF_ALPHA_I));
  iR_filt_A = (iR_meas * LPF_ALPHA_I) + (iR_filt_A * (1.0f - LPF_ALPHA_I));

  // “符号付き実電流”を作る（符号はIrefの符号から付ける）
  float iL_signed = (I_L >= 0.0f ? 1.0f : -1.0f) * fabs(iL_filt_A);
  float iR_signed = (I_R >= 0.0f ? 1.0f : -1.0f) * fabs(iR_filt_A);

  // =================================================
  // ⑥ 内側：電流PI（Iref追従 → PWM生成）
  // =================================================
  float eL = I_L - iL_signed;
  float eR = I_R - iR_signed;

  // 積分（PWM単位で保持）
  iL_integ += (KI_I * eL * Ts);
  iR_integ += (KI_I * eR * Ts);

  float uL = KP_I * eL + iL_integ;
  float uR = KP_I * eR + iR_integ;

  // 飽和
  float uL_sat = constrain(uL, -PWM_LIMIT, PWM_LIMIT);
  float uR_sat = constrain(uR, -PWM_LIMIT, PWM_LIMIT);

  // 簡易アンチワインドアップ（飽和分だけ積分を戻す）
  iL_integ += (uL_sat - uL);
  iR_integ += (uR_sat - uR);

  pwmL_cmd = (int)uL_sat;
  pwmR_cmd = (int)uR_sat;

  // =================================================
  // ⑦ 出力
  // =================================================
  motorL.drive(-pwmL_cmd);
  motorR.drive(pwmR_cmd);
}

// =====================================================
// 9) setup
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.begin();
  handleL.begin();
  handleR.begin();

  attachInterrupt(digitalPinToInterrupt(2), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(3), isrR, RISING);

  // --- Timer4 (100Hz) ---
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 499;                      // 100Hz (16MHz / 64 / 100 - 1)
  TCCR4B |= (1 << WGM42);            // CTC
  TCCR4B |= (1 << CS41) | (1 << CS40); // prescaler 64
  TIMSK4 |= (1 << OCIE4A);           // enable compare match
  interrupts();

  Serial.println("System Ready (Torque(Current) Control @100Hz)");
}

// =====================================================
// 10) loop：ロギング（Iref/Imeasも出す）
// =====================================================
void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 50) {  // 20Hz
    lastPrint = now;

    imu.update();

    // CSV: t, fL_g, fR_g, distL, distR, IrefL, ImeasL, pwmL, IrefR, ImeasR, pwmR
    Serial.print(now / 1000.0, 3);   Serial.print(",");
    Serial.print(fL_filt_g, 1);      Serial.print(",");
    Serial.print(fR_filt_g, 1);      Serial.print(",");
    Serial.print(encL.getDistance(), 3); Serial.print(",");
    Serial.print(encR.getDistance(), 3); Serial.print(",");
    Serial.print(iL_ref_A, 3);       Serial.print(",");
    Serial.print(iL_filt_A, 3);      Serial.print(",");
    Serial.print(pwmL_cmd);          Serial.print(",");
    Serial.print(iR_ref_A, 3);       Serial.print(",");
    Serial.print(iR_filt_A, 3);      Serial.print(",");
    Serial.println(pwmR_cmd);
  }
}
