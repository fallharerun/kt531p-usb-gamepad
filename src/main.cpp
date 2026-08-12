// KT-531P（MINI-Z プロポ）のステアリングとスロットルを USB HID ゲームパッドの
// 2 軸として出力する。
//
//   ボード : Seeed XIAO RP2040
//   コア   : earlephilhower/arduino-pico（USB スタックは Pico SDK）
//
// キャリブレーション値は XIAO 側で採取して EEPROM に保存する。個体差を XIAO 側が
// 持つので、プロポが違っても同じバイナリで動く。キャリブレーションモードへは
// B(BOOT) の長押し、またはシリアルの 'c' で入る。

#include <Arduino.h>
#include <Joystick.h>
#include <EEPROM.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#include "kt531p_io.h"

// HID エンドポイントの bInterval[ms]。arduino-pico 側が weak シンボルで定義して
// いるので、同名のグローバル変数を置けば上書きできる。
//
// これは単なる上限ではなく、loop() 全体の周期を決める値である。send_now() が
// ホストの次のポーリングまでスピン待ちするため、ループ側に何 ms のタイマを組んでも
// これより速くはならない。既定の 10 のままでは 100Hz 前後しか出ない。
int usb_hid_poll_interval = 4;   // 250Hz

// use16bit() 時の軸の値域。ディスクリプタが LOGICAL_MIN(-32767) 宣言なので、
// -32768 は内部でクランプされて使えない。
constexpr long HID_FULL = 32767;

// キャリブレーションで記録した端点の何割で全開に到達させるか。1.00 にすると毎回メカ
// ストッパーへ強く当てないと 100% が出ない。
constexpr float END_MARGIN = 0.98f;

// 断線・短絡の判定しきい値。ADC のレール(0 / ADC_MAX)からこの幅の内側に入った値は、
// ポテンショメータが分圧回路として機能していないことを示すので異常とみなす。
//
// このしきい値をキャリブレーション値から導出してはいけない。「記録した min/max の
// 外側は異常」とすると、端まで振り切らずにキャリブレーションした個体で、記録値より
// 奥へ動かした瞬間に誤検出して軸が 0 に固着する（全開にした瞬間スロットルが
// 切れる）。ポテンショメータの可動域はリンケージの都合で狭く、両端ともレールから
// 十分離れているので、固定しきい値でも正常操作には干渉しない。
constexpr int FAULT_RAIL_MARGIN = 64;
constexpr int GUARD_LO = FAULT_RAIL_MARGIN;
constexpr int GUARD_HI = ADC_MAX - FAULT_RAIL_MARGIN;

// 単発ノイズでの誤検出を避けるため、連続で外れた回数がこれに達したら確定する。
constexpr uint8_t FAULT_TRIP = 5;   // 250Hz なので 20ms

constexpr uint32_t SEND_INTERVAL_MS      = 4;    // 250Hz
constexpr uint32_t HOUSEKEEP_INTERVAL_MS = 20;
constexpr uint32_t PRINT_INTERVAL_MS     = 100;  // デバッグ表示 10Hz

constexpr int LINE_MAX = 96;   // 出力 1 行あたりの上限

// ============================================================================
// キャリブレーションパラメータ
// ============================================================================

constexpr uint32_t CAL_HOLD_MS         = 3000;  // B ボタン長押しの判定時間
constexpr uint32_t CAL_SETTLE_MS       = 3000;  // ニュートラル採取までの待ち
constexpr uint32_t CAL_IDLE_FINISH_MS  = 5000;  // min/max 更新停止から確定まで
constexpr uint32_t CAL_RESULT_MS       = 2000;  // 結果 LED の表示時間
constexpr int      CAL_NEUTRAL_SAMPLES = 64;

// キャリブレーション結果の妥当性チェック。壊れたキャリブレーションを保存して
// 操作不能になるのを防ぐ。
//
//   下限 → 振り切り不足。可動域が狭いと ADC 1 カウントあたりの出力量が大きくなり、
//          ノイズがそのまま出る粗い軸になる
//   上限 → キャリブレーション中にワイパーが浮いて 0 や ADC_MAX を記録した異常。
//          通すと全開に到達しなくなるうえ、可動域が断線判定のしきい値と重なる
//
// 手元のプロポの可動域は約 3400 カウント、片側は最小 1100 程度。いずれも
// その 6 割を目安にしている。機構が異なるプロポへ流用するときは見直すこと。
constexpr int CAL_MIN_RANGE = 2000;
constexpr int CAL_MIN_SIDE  = 700;
constexpr int CAL_MAX_RANGE = 3800;

// デッドゾーンと反転方向は個体差ではなく機構と配線で決まるので、キャリブレーション
// 対象にせず定数で持つ。軸ごとに値を変えているのは、手を離したときの戻り位置の
// ばらつきが軸によって異なるため。ステアリングは確実にセンターへ戻るが、スロットル
// はトリガー操作後の静止位置が無操作時よりずれる。共通の小さな値にすると、
// 手を離しても出力が残り続ける。
constexpr long STEER_DEADZONE = 20;
constexpr long THR_DEADZONE   = 50;

// 生値の増加方向が HID の慣例（X は右がプラス、Y は前進がプラス）と逆なので
// 反転する。
constexpr bool STEER_INVERT = true;
constexpr bool THR_INVERT   = true;

// キャリブレーション未実施・EEPROM 破損時のフォールバック。手元のプロポの実測値
// であり、キャリブレーションしていない個体でもとりあえず操作できるようにするための
// もの。正確な操作感が要るならキャリブレーションすること。
constexpr int DEF_STEER_LO = 466, DEF_STEER_NEU = 2168, DEF_STEER_HI = 3882;
constexpr int DEF_THR_LO   = 554, DEF_THR_NEU   = 2731, DEF_THR_HI   = 3912;

// ============================================================================
// 軸のスケーリング設定
// ============================================================================

struct AxisCal {
  int  neutral  = 0;
  long deadzone = 0;
  long spanLo   = 0;   // ニュートラルから下側全開までの有効カウント数（デッドゾーンを除く）
  long spanHi   = 0;   // ニュートラルから上側全開までの有効カウント数
  bool invert   = false;

  // END_MARGIN の乗算はここでしか行わない。呼ばれるのは起動時と
  // キャリブレーション確定時だけなので、FPU を持たない Cortex-M0+ でも 250Hz の
  // 経路に浮動小数点は乗らない。
  void configure(int lo, int neu, int hi, long dz, bool inv) {
    neutral  = neu;
    deadzone = dz;
    spanLo   = static_cast<long>((neu - lo) * END_MARGIN) - dz;
    spanHi   = static_cast<long>((hi - neu) * END_MARGIN) - dz;
    invert   = inv;
  }
};

static AxisCal calSteer;
static AxisCal calThrottle;

struct AxisState {
  uint8_t faultCount = 0;
  bool    faulted    = false;
};

static AxisState stState;
static AxisState thState;

// ============================================================================
// キャリブレーション値の永続化
//
// RP2040 に EEPROM は載っていない。arduino-pico の実装はフラッシュ最終セクタ(4KB)
// のエミュレーションで、プログラム領域の外にあるため電源断でもファームウェアの
// 再書き込みでも残る（フラッシュ全消去では消える）。
//
// commit() は消去と書き込みで数十ms かかり、その間割り込みが止まる。
// キャリブレーションの確定時に 1 回だけ呼ぶこと。ループ内で呼んではいけない。
// ============================================================================

constexpr uint32_t CAL_MAGIC   = 0x4B353331UL;   // "K531"
constexpr uint16_t CAL_VERSION = 1;              // 構造体レイアウトの版
constexpr int      EEPROM_SIZE = 256;

struct StoredCal {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  int16_t  steerLo, steerNeu, steerHi;
  int16_t  thrLo,   thrNeu,   thrHi;
  uint32_t crc;      // ここより前の全バイトに対する CRC32
};

// CRC は構造体の生バイト列に対して取る。フィールドを足してパディングが生まれると
// 保存時と読み込み時で対象バイトがずれ、毎起動 CRC 不一致 → 既定値、という分かり
// にくい壊れ方をする。変更に気づけるようレイアウトを固定する（変更時は
// CAL_VERSION も上げること）。
static_assert(sizeof(StoredCal) == 24, "StoredCal layout changed: bump CAL_VERSION");
static_assert(offsetof(StoredCal, crc) == 20, "StoredCal layout changed: bump CAL_VERSION");

static uint32_t crc32(const uint8_t* p, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  return ~crc;
}

// マジック・版・CRC がすべて一致したときだけ有効とみなす。版が変われば自動的に
// 無効扱いになるので、構造体を変更しても古いレコードを取り違えない。
static bool loadStored(StoredCal& rec) {
  EEPROM.get(0, rec);
  if (rec.magic != CAL_MAGIC || rec.version != CAL_VERSION) {
    return false;
  }
  const uint32_t want =
      crc32(reinterpret_cast<const uint8_t*>(&rec), offsetof(StoredCal, crc));
  return want == rec.crc;
}

static bool saveStored(StoredCal rec) {
  rec.magic    = CAL_MAGIC;
  rec.version  = CAL_VERSION;
  rec.reserved = 0;
  rec.crc = crc32(reinterpret_cast<const uint8_t*>(&rec), offsetof(StoredCal, crc));
  EEPROM.put(0, rec);
  if (!EEPROM.commit()) {
    return false;
  }

  // commit() は書き込みの成否を見ずに常に true を返すので、戻り値だけでは失敗を
  // 検出できない。EEPROM.get() は RAM バッファを読むため、begin() でフラッシュ
  // から読み直してから照合する。
  EEPROM.begin(EEPROM_SIZE);
  StoredCal back{};
  return loadStored(back) && memcmp(&back, &rec, sizeof(StoredCal)) == 0;
}

// 保存時と読み込み時の両方で使う。CRC が一致していても値が異常なレコード（版を
// 上げ忘れた古いビルドが書いたもの等）を弾くため、読み込み側でも必ず検査する。
static bool rangeValid(int lo, int neu, int hi) {
  const int range = hi - lo;
  return range >= CAL_MIN_RANGE && range <= CAL_MAX_RANGE &&
         (neu - lo) >= CAL_MIN_SIDE &&
         (hi - neu) >= CAL_MIN_SIDE &&
         // 端点がレール際 = キャリブレーション中にワイパーが浮いた等の異常。
         // 通すと可動域が断線判定のしきい値と重なる。
         lo >= GUARD_LO && hi <= GUARD_HI;
}

static bool storedValid(const StoredCal& rec) {
  return rangeValid(rec.steerLo, rec.steerNeu, rec.steerHi) &&
         rangeValid(rec.thrLo,   rec.thrNeu,   rec.thrHi);
}

// 適用中のキャリブレーション値。AxisCal は span などの導出値しか持たないので、
// 何が保存されているかを表示するために元の値を保持する。
static StoredCal activeCal{};
static bool      usingDefaults = true;

static void applyStored(const StoredCal& rec) {
  activeCal = rec;
  calSteer.configure(rec.steerLo, rec.steerNeu, rec.steerHi,
                     STEER_DEADZONE, STEER_INVERT);
  calThrottle.configure(rec.thrLo, rec.thrNeu, rec.thrHi,
                        THR_DEADZONE, THR_INVERT);
}

static void applyDefaults() {
  StoredCal rec{};
  rec.steerLo  = DEF_STEER_LO;
  rec.steerNeu = DEF_STEER_NEU;
  rec.steerHi  = DEF_STEER_HI;
  rec.thrLo    = DEF_THR_LO;
  rec.thrNeu   = DEF_THR_NEU;
  rec.thrHi    = DEF_THR_HI;
  applyStored(rec);
}

// ============================================================================
// LED（XIAO RP2040 の RGB LED はアノードコモンで LOW 点灯）
// ============================================================================

// pinMode(OUTPUT) 直後の出力レジスタは LOW＝点灯側なので、先に消灯レベルを書いて
// おかないと起動時に 3 色同時点灯して白く光る。LED はキャリブレーション時の唯一の
// UI であり、結果表示の緑／赤と紛らわしい。
static void ledInit() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
}

static void ledSet(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

// ============================================================================
// シリアル出力
//
// SerialUSB::write は CDC の送信 FIFO が埋まっていると空くまでスピン待ちする
// （最大 1 秒）。その間 loop() ごと止まって HID 送出も凍結するので、直接書いては
// いけない（フルスロットル中なら全開のまま固着する）。
//
// かといって「空きがあるときだけ書く」方式では、FIFO(256B) より長いまとまった出力
// で後半の行が捨てられる。いったん行キューへ積み、ハウスキープごとに空きぶんだけ
// 流す。行と行の間に FIFO の排出時間ができるので、ブロックせず、かつ欠落もしない。
// ============================================================================

constexpr int OUT_QUEUE_LINES = 16;

static char    outQueue[OUT_QUEUE_LINES][LINE_MAX];
static uint8_t outHead  = 0;   // 次に送る行
static uint8_t outCount = 0;   // 溜まっている行数

static void outLine(const char* s) {
  if (outCount >= OUT_QUEUE_LINES) {
    return;   // ホストが読んでいない。溢れた分は捨てる
  }
  const uint8_t tail = (outHead + outCount) % OUT_QUEUE_LINES;
  strncpy(outQueue[tail], s, LINE_MAX - 1);
  outQueue[tail][LINE_MAX - 1] = '\0';
  outCount++;
}

static void outLinef(const char* fmt, ...) {
  char buf[LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  outLine(buf);
}

static void outClear() {
  outHead  = 0;
  outCount = 0;
}

// FIFO に 1 行ぶんの空きがある限り流す。LINE_MAX を下回った時点で止めるので、
// SerialUSB::write がスピン待ちに入ることはない。
static void outFlush() {
  while (outCount > 0 && Serial && Serial.availableForWrite() >= LINE_MAX) {
    Serial.println(outQueue[outHead]);
    outHead = (outHead + 1) % OUT_QUEUE_LINES;
    outCount--;
  }
}

static void queueBanner() {
  outLine("");
  outLine("KT-531P HID gamepad  (16bit signed, manual send, 250Hz)");
  outLine("commands:  c = calibrate   q = abort   d = toggle debug   h / ? = help");
  outLine("           B button hold 3s = calibrate (no PC needed)");
  outLinef("calibration source: %s",
           usingDefaults ? "BUILT-IN DEFAULTS (not calibrated)" : "EEPROM (calibrated)");
  outLinef("  STEER     min %4d  neutral %4d  max %4d   (range %4d)",
           activeCal.steerLo, activeCal.steerNeu, activeCal.steerHi,
           activeCal.steerHi - activeCal.steerLo);
  outLinef("  THROTTLE  min %4d  neutral %4d  max %4d   (range %4d)",
           activeCal.thrLo, activeCal.thrNeu, activeCal.thrHi,
           activeCal.thrHi - activeCal.thrLo);
}

// ============================================================================
// 動作モード
// ============================================================================

enum class Mode : uint8_t {
  Run,        // 通常動作
  CalSettle,  // 手を離してもらいニュートラルを採取する
  CalRange,   // 端まで動かしてもらい min/max を追う
  CalResult,  // 結果を LED で提示する
};

static Mode     mode          = Mode::Run;
static uint32_t modeStart     = 0;
static uint32_t calLastUpdate = 0;
static bool     calSaved      = false;   // CalResult の表示内容（成功／失敗）

struct CalTrack {
  int lo = 0;
  int hi = 0;
  int neutral = 0;
};

static CalTrack trkSteer;
static CalTrack trkThrottle;

static bool     debugPrint = true;
static uint32_t sendCount  = 0;   // 実測送出レートの算出用

// ============================================================================
// 出力の計算
// ============================================================================

// ニュートラルを境に上下で別々にスケーリングする。スロットルは可動域が
// ニュートラルの上下で大きく偏っている（前進側が長い）ため、単一の係数では
// ニュートラルが 0 にならない。
static int16_t scaleAxis(int raw, const AxisCal& c) {
  const long d = static_cast<long>(raw) - c.neutral;

  if (d >= -c.deadzone && d <= c.deadzone) {
    return 0;
  }

  long v;
  if (d > 0) {
    if (c.spanHi <= 0) {
      return 0;
    }
    v = (d - c.deadzone) * HID_FULL / c.spanHi;
    v = constrain(v, 0L, HID_FULL);
  } else {
    if (c.spanLo <= 0) {
      return 0;
    }
    v = (d + c.deadzone) * HID_FULL / c.spanLo;
    v = constrain(v, -HID_FULL, 0L);
  }

  return static_cast<int16_t>(c.invert ? -v : v);
}

// ワイパーが外れると ADC ピンはフローティングになる。その値をそのままスケールすると
// 全開・全ロックをランダムに出し続けるので、異常を検出したらニュートラル(0)へ
// 落とす。前回値保持にしないのは、全開中に発生した場合に固着させないため。
//
// 完全な断線検出ではない。レールへ張り付く故障は捉えられるが、ワイパーが浮いた
// まま可動域内の値を示す場合は検出できない。
static int16_t updateAxis(int raw, const AxisCal& c, AxisState& s) {
  const bool bad = (raw <= GUARD_LO) || (raw >= GUARD_HI);

  if (bad) {
    if (s.faultCount < FAULT_TRIP) {
      s.faultCount++;
    }
    if (s.faultCount >= FAULT_TRIP) {
      s.faulted = true;
    }
  } else {
    s.faultCount = 0;
    s.faulted = false;
  }

  return s.faulted ? 0 : scaleAxis(raw, c);
}

// ============================================================================
// キャリブレーション
// ============================================================================

// modeStart / calLastUpdate には呼び出し側の now を使う。ここで millis() を取り
// 直すと、直前の ADC バーストぶん loop() の now より進んでしまい、同じ反復内の
// (now - calLastUpdate) が符号なし減算でアンダーフローする。
static void enterCalibration(uint32_t now) {
  mode = Mode::CalSettle;
  modeStart = now;
  outLine("");
  outLine("== CALIBRATION ==");
  outLine("step 1/2: release both controls and wait (blue slow blink)");
}

static void beginRangePhase(uint32_t now) {
  long sumSt = 0;
  long sumTh = 0;
  for (int i = 0; i < CAL_NEUTRAL_SAMPLES; i++) {
    sumSt += readAveraged(PIN_STEER);
    sumTh += readAveraged(PIN_THROTTLE);
  }
  trkSteer.neutral =
      static_cast<int>((sumSt + CAL_NEUTRAL_SAMPLES / 2) / CAL_NEUTRAL_SAMPLES);
  trkThrottle.neutral =
      static_cast<int>((sumTh + CAL_NEUTRAL_SAMPLES / 2) / CAL_NEUTRAL_SAMPLES);

  trkSteer.lo    = trkSteer.hi    = trkSteer.neutral;
  trkThrottle.lo = trkThrottle.hi = trkThrottle.neutral;

  mode          = Mode::CalRange;
  modeStart     = now;
  calLastUpdate = now;

  outLinef("neutral captured:  ST %4d   TH %4d",
           trkSteer.neutral, trkThrottle.neutral);
  outLine("step 2/2: move both controls to their limits a few times");
  outLine("          (blue fast blink; finishes 5s after you stop)");
}

static bool trackRange(CalTrack& t, int raw) {
  bool updated = false;
  if (raw < t.lo) {
    t.lo = raw;
    updated = true;
  }
  if (raw > t.hi) {
    t.hi = raw;
    updated = true;
  }
  return updated;
}

static void finishCalibration(uint32_t now) {
  const bool ok = rangeValid(trkSteer.lo, trkSteer.neutral, trkSteer.hi) &&
                  rangeValid(trkThrottle.lo, trkThrottle.neutral, trkThrottle.hi);

  outLinef("STEER     min %4d   neutral %4d   max %4d",
           trkSteer.lo, trkSteer.neutral, trkSteer.hi);
  outLinef("THROTTLE  min %4d   neutral %4d   max %4d",
           trkThrottle.lo, trkThrottle.neutral, trkThrottle.hi);

  if (ok) {
    StoredCal rec{};
    rec.steerLo  = static_cast<int16_t>(trkSteer.lo);
    rec.steerNeu = static_cast<int16_t>(trkSteer.neutral);
    rec.steerHi  = static_cast<int16_t>(trkSteer.hi);
    rec.thrLo    = static_cast<int16_t>(trkThrottle.lo);
    rec.thrNeu   = static_cast<int16_t>(trkThrottle.neutral);
    rec.thrHi    = static_cast<int16_t>(trkThrottle.hi);

    calSaved = saveStored(rec);
    if (calSaved) {
      applyStored(rec);
      usingDefaults = false;
      outLine("saved to EEPROM");
    } else {
      outLine("EEPROM write FAILED - previous calibration kept");
    }
  } else {
    // キャリブレーションモードに入ったまま動かさなかった場合もここへ来る。前の
    // キャリブレーションを維持したまま戻るので、誤って入っても実害は出ない。
    calSaved = false;
    outLine("REJECTED (implausible travel) - previous calibration kept");
  }

  stState = AxisState{};
  thState = AxisState{};

  mode      = Mode::CalResult;
  modeStart = now;
}

// キャリブレーション中は両軸の出力を 0 に固定するため、中断手段が無いと誤って
// 入ったときに最短 10 秒間どちらも効かなくなる。B の短押しとシリアル 'q' で
// いつでも戻す。前のキャリブレーションはそのまま維持される。
static void abortCalibration() {
  mode = Mode::Run;
  stState = AxisState{};
  thState = AxisState{};
  outLine("== calibration ABORTED - previous calibration kept ==");
}

static void updateLed(uint32_t now) {
  const uint32_t since = now - modeStart;

  switch (mode) {
    case Mode::CalSettle:
      ledSet(false, false, (since / 500) % 2 == 0);   // 青ゆっくり点滅
      break;
    case Mode::CalRange:
      ledSet(false, false, (since / 100) % 2 == 0);   // 青速く点滅
      break;
    case Mode::CalResult:
      ledSet(!calSaved, calSaved, false);             // 成功=緑 / 失敗=赤
      break;
    case Mode::Run:
      // 断線を検出している間だけ赤点滅。それ以外は消灯。
      if (stState.faulted || thState.faulted) {
        ledSet((now / 200) % 2 == 0, false, false);
      } else {
        ledSet(false, false, false);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  adcBegin();
  ledInit();

  EEPROM.begin(EEPROM_SIZE);

  StoredCal rec{};
  if (loadStored(rec) && storedValid(rec)) {
    applyStored(rec);
    usingDefaults = false;
  } else {
    applyDefaults();
    usingDefaults = true;
  }

  // 既定は 10bit 符号なし(0..1023) で、しかも軸を書くたびに自動送信される。
  // use16bit() を忘れると 16bit のつもりで書いた値が上限クランプされ、軸が
  // 振り切ったまま固定される。
  //
  // begin() は USB を切断して再列挙するため、この直後に Serial へ書いても CDC
  // 未接続で捨てられる。起動バナーは loop() 側で接続を検出してから出す。
  Joystick.use16bit();
  Joystick.useManualSend(true);
  Joystick.begin();
}

void loop() {
  const uint32_t now = millis();

  static uint32_t nextSend = 0;
  static int16_t  outSt = 0;
  static int16_t  outTh = 0;
  static int      rawSt = 0;
  static int      rawTh = 0;

  // --- HID 送出（250Hz）---
  if (static_cast<int32_t>(now - nextSend) >= 0) {
    // 周期は now 代入ではなく加算で刻む。now を入れると 1 周期あたりの処理時間
    // ぶん必ず遅れ、実効レートが公称値に届かない。
    nextSend += SEND_INTERVAL_MS;
    if (static_cast<int32_t>(now - nextSend) >= static_cast<int32_t>(SEND_INTERVAL_MS)) {
      // 大きく遅れたときは連続送出で取り戻そうとせず、now から刻み直す
      nextSend = now + SEND_INTERVAL_MS;
    }

    rawSt = readAveraged(PIN_STEER);
    rawTh = readAveraged(PIN_THROTTLE);

    if (mode == Mode::Run) {
      outSt = updateAxis(rawSt, calSteer,    stState);
      outTh = updateAxis(rawTh, calThrottle, thState);
    } else {
      // キャリブレーション中は端まで大きく動かすので、その動きをゲームへ送らない
      outSt = 0;
      outTh = 0;
      if (mode == Mode::CalRange) {
        // ピークを取り逃さないよう、追従はサンプルレートで行う
        const bool a = trackRange(trkSteer,    rawSt);
        const bool b = trackRange(trkThrottle, rawTh);
        if (a || b) {
          calLastUpdate = now;
        }
      }
    }

    Joystick.X(outSt);
    Joystick.Y(outTh);
    Joystick.send_now();   // ホストがポーリングするまでここでブロックする
    sendCount++;
  }

  // --- 低頻度の処理（20ms）---
  // Serial と BOOTSEL の読み取りは USB mutex 取得や割り込み禁止を伴うので間引く。
  static uint32_t nextHousekeep = 0;
  if (static_cast<int32_t>(now - nextHousekeep) < 0) {
    return;
  }
  nextHousekeep = now + HOUSEKEEP_INTERVAL_MS;

  updateLed(now);

  // --- キャリブレーションモードの遷移 ---
  switch (mode) {
    case Mode::CalSettle:
      if (now - modeStart >= CAL_SETTLE_MS) {
        beginRangePhase(now);
      }
      break;
    case Mode::CalRange:
      if (now - calLastUpdate >= CAL_IDLE_FINISH_MS) {
        finishCalibration(now);
      }
      break;
    case Mode::CalResult:
      if (now - modeStart >= CAL_RESULT_MS) {
        mode = Mode::Run;
        outLine("== back to normal operation ==");
      }
      break;
    case Mode::Run:
      break;
  }

  // --- B(BOOT) ボタン長押しでキャリブレーションへ ---
  //
  // 「押しながら起動」は使えない。それはブート ROM への指示であり、スケッチが
  // 動き出す前に USB マスストレージのブートローダーへ入ってしまう。通常起動した
  // あとの長押しで判定する。BOOTSEL の実行時読み取りは QSPI のチップセレクト
  // 切り替えと割り込み禁止を伴うので、250Hz のループではなくここで読む。
  //
  // bTriggered は 1 押下につき 1 回しかトリガさせないためのラッチ。これが無いと、
  // 押しっぱなしのままキャリブレーションが終わった時点で長押し条件が再び成立し、
  // 指を離すまでキャリブレーションに入り直し続ける。
  static bool     bWasDown   = false;
  static bool     bTriggered = false;
  static uint32_t bDownAt    = 0;
  const bool bDown = BOOTSEL;

  if (bDown && !bWasDown) {
    bDownAt    = now;
    bTriggered = false;
    if (mode != Mode::Run) {
      abortCalibration();
      bTriggered = true;   // 中断に使った押下でキャリブレーションに入り直さない
    }
  } else if (bDown && !bTriggered && mode == Mode::Run &&
             (now - bDownAt) >= CAL_HOLD_MS) {
    enterCalibration(now);
    bTriggered = true;
  }
  bWasDown = bDown;

  // --- シリアル ---
  // モニタが接続された瞬間にバナーを出す。切断から再接続でも出し直す。実際の送出
  // は outFlush() が行うので、接続直後に FIFO が空いていなくても取りこぼさない。
  static bool wasConnected = false;
  const bool connected = static_cast<bool>(Serial);
  if (connected && !wasConnected) {
    queueBanner();
  }
  if (!connected) {
    outClear();   // 切断中に溜め込んだ古い行を持ち越さない
  }
  wasConnected = connected;

  while (Serial.available()) {
    const int ch = Serial.read();
    if (ch == 'c' || ch == 'C') {
      if (mode == Mode::Run) {
        enterCalibration(now);
      }
    } else if (ch == 'q' || ch == 'Q') {
      if (mode != Mode::Run) {
        abortCalibration();
      }
    } else if (ch == 'd' || ch == 'D') {
      debugPrint = !debugPrint;
      outLinef("-- debug print %s --", debugPrint ? "ON" : "OFF");
    } else if (ch == 'h' || ch == 'H' || ch == '?') {
      queueBanner();
    }
  }

  // --- デバッグ表示 ---
  static uint32_t nextPrint = 0;
  static uint32_t rateStamp = 0;
  if (debugPrint && static_cast<int32_t>(now - nextPrint) >= 0) {
    nextPrint = now + PRINT_INTERVAL_MS;

    const uint32_t elapsed = now - rateStamp;
    const uint32_t hz = elapsed ? (sendCount * 1000UL + elapsed / 2) / elapsed : 0;
    sendCount = 0;
    rateStamp = now;

    if (mode == Mode::CalRange) {
      outLinef("CAL  ST [%4d..%4d]  TH [%4d..%4d]   idle %lus",
               trkSteer.lo, trkSteer.hi, trkThrottle.lo, trkThrottle.hi,
               static_cast<unsigned long>((now - calLastUpdate) / 1000));
    } else if (mode == Mode::Run) {
      outLinef("ST raw %4d -> %6d%s   |   TH raw %4d -> %6d%s   | %3luHz",
               rawSt, outSt, stState.faulted ? " FAULT" : "",
               rawTh, outTh, thState.faulted ? " FAULT" : "",
               static_cast<unsigned long>(hz));
    }
  }

  outFlush();
}
