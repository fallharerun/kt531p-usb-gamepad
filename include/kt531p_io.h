#pragma once

#include <Arduino.h>

// ============================================================================
// ポテンショメータ読み取りの共通定義
//
// キャリブレーション時と通常動作時で読み取り条件がずれると、採取した
// min/neutral/max が実動作と食い違い、デッドゾーンの設定値も意味をなさなくなる。
// ピン割り当てとオーバーサンプル数はここだけで定義し、どちらの経路も
// readAveraged() を通す。
// ============================================================================

// J3 ステアリング ワイパー(緑) → D0 = A0 = GP26
// J4 スロットル   ワイパー(緑) → D1 = A1 = GP27
constexpr int PIN_STEER    = A0;
constexpr int PIN_THROTTLE = A1;

constexpr int ADC_BITS = 12;
constexpr int ADC_MAX  = (1 << ADC_BITS) - 1;   // 4095

constexpr int OVERSAMPLE = 16;

inline void adcBegin() {
  analogReadResolution(ADC_BITS);
}

// RP2040 の ADC は 1 個の SAR を 4 チャンネルで共有し、arduino-pico の analogRead()
// はチャンネル切替直後に整定待ちなしで変換を開始する。サンプル＆ホールドに残った
// 前チャンネルの電荷が 1 サンプル目に乗るため、2 軸を交互に読むと「片方の軸を
// 動かすともう片方のニュートラル値がずれる」軸間クロストークになる。平均に入れる
// 前に 1 回捨て読みして、サンプル＆ホールドを当該チャンネルの電圧で充電しておく。
//
// 平均は四捨五入する。切り捨てのままだと常に約 -0.5LSB の系統誤差が乗り、16 回読む
// 計算コストに見合わない。
inline int readAveraged(int pin) {
  (void)analogRead(pin);   // チャンネル切替後の捨て読み。値は使わない

  long sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) {
    sum += analogRead(pin);
  }
  return static_cast<int>((sum + OVERSAMPLE / 2) / OVERSAMPLE);
}
