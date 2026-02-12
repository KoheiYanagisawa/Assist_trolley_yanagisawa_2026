#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// 制御周期
// =====================================================
constexpr float CTRL_HZ = 500.0f;
constexpr float Ts      = 1.0f / CTRL_HZ;

// =====================================================
// 電源・PWM
// =====================================================
constexpr float VBUS      = 24.0f;
constexpr int   PWM_MAX_HW = 255;
constexpr int   PWM_LIMIT  = 200;     // 安全制限（元コード踏襲）

// =====================================================
// 駆動系：Φ60ローラがΦ300タイヤに接触
// =====================================================
constexpr float R_TIRE   = 0.150f;   // [m] (Φ300)
constexpr float R_ROLLER = 0.030f;   // [m] (Φ60)
constexpr float GEAR     = 10.0f;    // ギア比

// 推定トルク定数 kt [Nm/A]（モータ軸）
constexpr float KT = 0.054f;

// =====================================================
// ロードセル・アシスト設計
// =====================================================
// g -> N 変換
constexpr float G2N = 9.80665f / 1000.0f;  // [N/g]

// 荷重(g)->推進力(N) のゲイン
constexpr float K_F = 0.2f;               // [N/g] ★調整ポイント

constexpr float FORCE_DEADZONE_G = 50.0f;  // [g] デッドゾーン（要望）
constexpr float LPF_ALPHA_F      = 0.01f;   // 荷重LPF

constexpr float I_DEADZONE   = 0.15f;      // [A]
constexpr float I_REF_LIMIT  = 3.0f;       // [A]

// =====================================================
// 電流PI
// =====================================================
constexpr float KP_I = 60.0f;              // [PWM/A]
constexpr float KI_I = 200.0f;             // [PWM/(A*s)]
constexpr float LPF_ALPHA_I = 0.3f;
// =====================================================
// PWM-FF + 電流トリム（電流センサをサブ運用）
// =====================================================
// Iref[A] -> PWM のフィードフォワード係数（まずは大雑把でOK）
constexpr float K_PWM_I = 60.0f;      // [PWM/A] ★調整ポイント

// トリム（不足分だけ"やんわり"足す）
// 80Hz側で積分するので強すぎ注意。まずは小さく。
constexpr float KI_TRIM = 30.0f;      // [PWM/(A*s)] ★調整ポイント

// トリム上限（大きすぎるとFFの意味がなくなる）
constexpr float PWM_TRIM_MAX = 40.0f; // [PWM]

// =====================================================
// ピン（現状の.ino踏襲）
// =====================================================
// モーター(DIR, PWM)
constexpr int PIN_MOTOR_L_DIR = 12;
constexpr int PIN_MOTOR_L_PWM = 11;
constexpr int PIN_MOTOR_R_DIR = 10;
constexpr int PIN_MOTOR_R_PWM = 9;

// エンコーダ割り込み入力（attachInterruptで使う方）
constexpr int PIN_ENC_L_A = 2;
constexpr int PIN_ENC_R_A = 3;

// isr内で参照してたB相入力（digitalReadしてる方）
constexpr int PIN_ENC_L_B = 4;
constexpr int PIN_ENC_R_B = 6;

// ACS712
constexpr int PIN_CUR_L = A3;
constexpr int PIN_CUR_R = A2;

// ログ
constexpr bool ENABLE_LOG = true;
constexpr int  LOG_PERIOD_MS = 50;  // 20Hz

#endif
