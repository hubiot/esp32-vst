// =============================================================================
// VST-01 / VST-100 統合ファームウェア (1Chip 版)
// 測定(MCP3424 / 雨量計 / 統計計算) と 通信(WiFi / SoftAP / AWS IoT MQTT)
// の統合
// =============================================================================

// 0:通常動作
// 1:騒音・振動デバッグ用(インクリメント)

#define NOISE_VIB_DEBUG 0

// -----------------------------------------------------------------------------
// 機種定義 (VST-100 / VST-01 / VST-01R)
// platformio.ini の build_flags (-D VST100, -D VST01, -D VST01R) で定義されます。
// platformio.ini 側で未指定の場合のみ、デフォルトとして VST100 を定義します。
// -----------------------------------------------------------------------------
#if !defined(VST100) && !defined(VST01) && !defined(VST01R)
#define VST100 // デフォルト: VST-100
#endif

#include "aws.h" // AWS証明書
#include "esp_sntp.h"
#include "esp_system.h"
#include "time.h"

#include <EEPROM.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <algorithm>
#include <math.h>

// -----------------------------------------------------------------------------
// ハードウェア・ピン定義
// -----------------------------------------------------------------------------
const int SDA_PIN = 21; // I2C MCP3424 SDA
const int SCL_PIN = 22; // I2C MCP3424 SCL
#define PULSE_IN 4      // 雨量計パルス入力
#define STATUS_LED 32   // ステータスLED
#define RELAY_OUT 33    // リレー出力
#define XAP_BTN 35      // APモード切替ボタン (入力のみ・プルアップなし)
#define CXS_PIN 25      // CXS
#define RX_PIN 16       // RX (GPIO16)
#define TX_PIN 17       // TX (GPIO17)
#define IO18_PIN 18     // GPIO18 (VST-01用)
#define IO19_PIN 19     // GPIO19 (VST-01用)
#define IO23_PIN 23     // GPIO23 (VST-01用)

#define JST (3600 * 9)

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v1.0.0"
#endif


// -----------------------------------------------------------------------------
// パラメータ・データ構造体定義
// -----------------------------------------------------------------------------
struct trans_para {
  unsigned int meas_large;
  unsigned int meas_small;
  float para_large;
  float para_small;
};

struct para_d {
  int model_no; // 0: rex noise/vibration, 1: 4ch normal cloud, 2: 4ch normal
                // local, 3: rex rain, 4: rex noise/vibration (00,10,,,50)
  String s_n_xave_flg[4];   // 演算 0:ave 1:normal (ch1-ch4)
  String host_ip;           // host ip (Local Server用)
  float shreshold;          // スレッシュホールド (雨量警報用)
  unsigned int meas_period; // 測定・通信周期(秒)
  int use_custom_client_id; // 0: ESP32 Hardware MAC, 1: Custom Specified Client
                            // ID
  String custom_client_id;  // 指定したカスタムClient ID
  String pub_topic;         // MQTT Publish Topic ("pub_prod", "pub01" 等)
  float pulse_weight;       // 1パルスあたりの雨量(mm), デフォルト 0.5
};

// 統合EEPROM保存用構造体 (固定長バイナリ)
struct UnifiedEepromSettings {
  char magic[8]; // "VST_U04"
  int model_no;
  char s_n_xave_flg[4][4];
  char host_ip[32];
  float shreshold;
  unsigned int meas_period;
  trans_para t_para[4];
  int use_custom_client_id;
  char custom_client_id[32];
  char pub_topic[32];
  float pulse_weight;
};

// -----------------------------------------------------------------------------
// グローバル変数
// -----------------------------------------------------------------------------
para_d PARA;
trans_para T_PARA[4];

// 通信・Web関連変数
const char *pubTopic =
    "pub_prod"; // デフォルト製品版 ("pub01" はクラウドデバッグ用)
const char ntp_server[][30] = {"ntp.nict.jp", "pool.ntp.org",
                               "ntp.jst.mfeed.ad.jp"};
time_t CUR_TIME;
struct tm TIMEINFO;

String ap_ssid = "TIC-AP";   // ESP32 softAP SSID
String ap_pass = "12345678"; // ESP32 softAP password
IPAddress LIP;               // Local IP address
WiFiServer server(80);
WiFiClient client;
uint8_t ssid_num;
String ssid_rssi_str[30];
String ssid_str[30];
String Selected_SSID_str;
String Sel_SSID_PASS_str;
String CLIENT_ID;        // MACアドレスベースのユニークID
boolean AP_MODE = false; // true:アクセスポイントモード false:通常モード
int PAGE_NUM = 0;        // 0:wifi set 1:parameter set

int CHATTERING_AP[3] = {1, 1, 1}; // APボタンチャタリング対策
int CHATTERING_CNT = 0;

int CUR_MIN = 0;
int PRE_MIN = 0;
int CUR_SEC = 0;
int PRE_SEC = 0;
float RAIN_OTH = 0.0; // 正時1時間雨量 (Rain on the hour)
int RCNT = 0;

String S_CH_NUM = "1";       // Calibration画面用
String S_LARGE_SMALL = "0";  // 0:large 1:small
boolean RELAY_STATE = false; // リレー出力状態保持フラグ

// AWS IoT
const char *awsEndpoint = "a24t2172v8g5ia-ats.iot.ap-northeast-1.amazonaws.com";
const int awsPort = 8883;
WiFiClientSecure httpsClient;
PubSubClient mqttClient(httpsClient);
boolean mqtt_error_flag = false; // MQTT送信失敗・未接続フラグ

hw_timer_t *timer = NULL; // Watchdog Timer用

// 測定関連変数
unsigned int SMPL_TIME = 100; // サンプリングタイム (通常100ms, 設定時1000ms)
unsigned int RAIN_SMPL_TIME = 10;   // 雨量計パルスサンプリングタイム 10ms
unsigned long RAIN_DETECT_TIME = 0; // 雨量計パルスチェック時刻
unsigned long MEAS_TIME = 0;        // ADCサンプリング時刻
unsigned int MCNT = 0;              // サンプリング回数カウンタ
double LEQ[2] = {0, 0};
boolean FIRST_FLAG = true;
boolean RAIN_FLAG = false; // 雨量測定データ送信トリガー

int RAIN_CNT = 0;              // 雨量カウント数
int RAIN_PULSE[3] = {0, 0, 0}; // チャタリング除去用
int PLS = 0;
int PRE_PLS = 0;

int RAW_MD[4];     // MCP3424 Raw測定値
int PRE_RAW_MD[4]; // エラー時代替用前回値
#if NOISE_VIB_DEBUG == 1
int debug_raw_val = 1; // 1から6000へカウントアップ (完全逆順ストレステスト)
#endif
// [DEBUG 1 期待値] (1〜6000 カウントアップ, val/2):
// L5=2850.50, L10=2700.50, L50=1500.50, L90=300.50, L95=150.50, MIN=0.50,
// MAX=3000.00, LEQ=2971.85 [DEBUG 2 期待値] (1〜3000 固定乱数, srand=12345):
// 起動時に disp_info() より
// 6000サンプルのシミュレーション結果がシリアル出力されます 理論値: AVE≈1500.5,
// L5≈2850.5, L10≈2700.5, L50≈1500.5, L90≈300.5, L95≈150.5, MIN≈1.0, MAX≈3000.0,
// LEQ≈2962〜2970
unsigned int md_max[4], md_min[4];
unsigned long md_sum[4];
uint16_t SORT_DATA[2][6000]; // 騒音・振動パーセンタイル計算用ソートバッファ
float SEND_DATA[25];

// Core間データ受渡し用構造体・キュー
struct MeasSendData {
  float data[25];
};
QueueHandle_t sendDataQueue = NULL;
TaskHandle_t measTaskHandle = NULL;

// -----------------------------------------------------------------------------
// HTML / Web UI テンプレート
// -----------------------------------------------------------------------------
const char *str_calibration = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>キャリブレーション - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 680px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 13px; font-weight: 700; color: #475569; margin-bottom: 14px;
        letter-spacing: 0.5px;
      }
      table {
        width: 100%; border-collapse: collapse; margin-bottom: 6px;
      }
      th {
        padding: 10px 8px; background-color: #f8fafc; color: #475569;
        font-size: 12px; font-weight: 700; border-bottom: 2px solid #e2e8f0; text-align: center;
      }
      td {
        padding: 12px 8px; border-bottom: 1px solid #f1f5f9; text-align: center;
        font-size: 14px; color: #334155;
      }
      .ch-cell { font-weight: 700; color: #4f46e5; }
      .value {
        font-family: SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        font-weight: 700; color: #0284c7; font-size: 15px;
      }
      .form-grid {
        display: grid; grid-template-columns: 1fr 1fr 1.5fr auto; gap: 10px; align-items: center;
      }
      @media (max-width: 500px) {
        .form-grid { grid-template-columns: 1fr 1fr; }
        .form-grid .full-span { grid-column: span 2; }
      }
      select, input[type=text] {
        width: 100%; padding: 10px 12px; border: 1.5px solid #cbd5e1; border-radius: 8px;
        font-size: 14px; color: #1e293b; background: #ffffff; box-sizing: border-box;
      }
      select:focus, input[type=text]:focus {
        outline: none; border-color: #4f46e5; box-shadow: 0 0 0 3px rgba(79, 70, 229, 0.15);
      }
      .btn-set {
        padding: 10px 20px; border: none; border-radius: 8px;
        background: linear-gradient(135deg, #10b981 0%, #059669 100%);
        color: #ffffff; font-size: 14px; font-weight: 700; cursor: pointer;
        transition: all 0.2s ease; box-shadow: 0 2px 6px rgba(16, 185, 129, 0.3);
      }
      .btn-set:hover {
        background: linear-gradient(135deg, #059669 0%, #047857 100%);
        transform: translateY(-1px);
      }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-secondary:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px);
      }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover {
        background: #fecaca; border-color: #f87171; transform: translateY(-1px);
      }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">キャリブレーション</div>

      <div class="card">
        <div class="card-title">測定データ &amp; パラメータ (LARGE / SMALL)</div>
        <table>
          <thead>
            <tr><th>CH</th><th>測定データ</th><th>LARGE</th><th>SMALL</th></tr>
          </thead>
          <tbody>
            <tr><td class="ch-cell">CH1</td><td><span id="val_ch1" class="value">-</span></td><td><span id="pl_ch1" class="value">-</span></td><td><span id="ps_ch1" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH2</td><td><span id="val_ch2" class="value">-</span></td><td><span id="pl_ch2" class="value">-</span></td><td><span id="ps_ch2" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH3</td><td><span id="val_ch3" class="value">-</span></td><td><span id="pl_ch3" class="value">-</span></td><td><span id="ps_ch3" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH4</td><td><span id="val_ch4" class="value">-</span></td><td><span id="pl_ch4" class="value">-</span></td><td><span id="ps_ch4" class="value">-</span></td></tr>
          </tbody>
        </table>
      </div>

      <div class="card">
        <div class="card-title">スケーリング パラメータ設定</div>
        <form name="paremeter_set" action="/param_set/" method="GET">
          <div class="form-grid">
            <div>
              <select name="channel_number">
                <option value="1">CH1</option>
                <option value="2">CH2</option>
                <option value="3">CH3</option>
                <option value="4">CH4</option>
              </select>
            </div>
            <div>
              <select name="large_small">
                <option value="0">LARGE</option>
                <option value="1">SMALL</option>
              </select>
            </div>
            <div class="full-span">
              <input type="text" name="conv_param" placeholder="設定値入力">
            </div>
            <div class="full-span">
              <button type="submit" name="param_submit" value="send" class="btn-set">Set</button>
            </div>
          </div>
        </form>
      </div>

      <div class="card">
        <div class="card-title">パルスカウント &amp; リレー動作確認</div>
        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:14px; background:#f8fafc; padding:12px 14px; border-radius:10px; border:1px solid #e2e8f0;">
          <div style="font-size:14px; font-weight:600; color:#334155;">パルスカウント数:</div>
          <div style="display:flex; align-items:center; gap:10px;">
            <div><span id="rain_cnt_val" class="value" style="font-size:18px; color:#16a34a;">0</span> <span style="font-size:12px; color:#64748b; font-weight:600;">counts</span></div>
            <button type="button" onclick="resetRainCount()" style="padding:4px 10px; border-radius:6px; font-size:12px; font-weight:600; cursor:pointer; border:1px solid #cbd5e1; background:#ffffff; color:#64748b;">クリア</button>
          </div>
        </div>
        <div style="display:flex; justify-content:space-between; align-items:center; background:#f8fafc; padding:12px 14px; border-radius:10px; border:1px solid #e2e8f0;">
          <div style="font-size:14px; font-weight:600; color:#334155;">リレー : <span id="relay_status_text" style="font-weight:700; color:#dc2626;">OFF</span></div>
          <button type="button" id="btn_relay_toggle" onclick="toggleRelay()" style="padding:8px 16px; border-radius:8px; font-size:13px; font-weight:700; cursor:pointer; border:1.5px solid #86efac; background:#dcfce7; color:#16a34a; transition:all 0.2s;">リレー ON にする</button>
        </div>
      </div>

      <div class="nav-group">
        <a href="/" class="btn btn-secondary">Home</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_trans_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let val = this.responseText.split(',');
          document.getElementById("val_ch1").innerHTML = val[0];
          document.getElementById("pl_ch1").innerHTML = val[2];
          document.getElementById("ps_ch1").innerHTML = val[4];
          document.getElementById("val_ch2").innerHTML = val[6];
          document.getElementById("pl_ch2").innerHTML = val[8];
          document.getElementById("ps_ch2").innerHTML = val[10];
          document.getElementById("val_ch3").innerHTML = val[12];
          document.getElementById("pl_ch3").innerHTML = val[14];
          document.getElementById("ps_ch3").innerHTML = val[16];
          document.getElementById("val_ch4").innerHTML = val[18];
          document.getElementById("pl_ch4").innerHTML = val[20];
          document.getElementById("ps_ch4").innerHTML = val[22];
          if (val.length >= 25) {
            let rc = document.getElementById("rain_cnt_val");
            if (rc) rc.innerHTML = val[24];
          }
          if (val.length >= 26) {
            let rState = Number(val[25]);
            updateRelayUI(rState);
          }
        }
      };
      xhr.open("GET", "/disp_trans_param", true);
      xhr.send(null);
    }
    function updateRelayUI(rState) {
      let rText = document.getElementById("relay_status_text");
      let rBtn = document.getElementById("btn_relay_toggle");
      if (rText && rBtn) {
        if (rState === 1) {
          rText.innerHTML = "ON";
          rText.style.color = "#16a34a";
          rBtn.innerHTML = "リレー OFF にする";
          rBtn.style.background = "#fee2e2";
          rBtn.style.color = "#dc2626";
          rBtn.style.borderColor = "#fca5a5";
        } else {
          rText.innerHTML = "OFF";
          rText.style.color = "#dc2626";
          rBtn.innerHTML = "リレー ON にする";
          rBtn.style.background = "#dcfce7";
          rBtn.style.color = "#16a34a";
          rBtn.style.borderColor = "#86efac";
        }
      }
    }
    function toggleRelay() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let s = Number(this.responseText);
          if (!isNaN(s)) {
            updateRelayUI(s);
          } else {
            disp_trans_param();
          }
        }
      };
      xhr.open("GET", "/relay_toggle", true);
      xhr.send(null);
    }
    function resetRainCount() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          disp_trans_param();
        }
      };
      xhr.open("GET", "/rain_cnt_reset", true);
      xhr.send(null);
    }
    var ch_ls_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          let elements = document.getElementsByName('channel_number');
          if (elements.length > 0 && Number(cmd[0]) >= 1) {
            elements[0].options[Number(cmd[0])-1].selected = true;
          }
          elements = document.getElementsByName('large_small');
          if (elements.length > 0 && Number(cmd[1]) >= 0) {
            elements[0].options[Number(cmd[1])].selected = true;
          }
        }
      };
      xhr.open("GET", "/ch_ls_param", true);
      xhr.send(null);
    }
    setInterval(disp_trans_param, 1000);
    window.onload = function() {
      disp_trans_param();
      ch_ls_param();
    };
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_factory_calibration = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>キャリブレーション (工場設定) - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 680px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 13px; font-weight: 700; color: #475569; margin-bottom: 14px;
        letter-spacing: 0.5px;
      }
      table {
        width: 100%; border-collapse: collapse; margin-bottom: 6px;
      }
      th {
        padding: 10px 8px; background-color: #f8fafc; color: #475569;
        font-size: 12px; font-weight: 700; border-bottom: 2px solid #e2e8f0; text-align: center;
      }
      td {
        padding: 12px 8px; border-bottom: 1px solid #f1f5f9; text-align: center;
        font-size: 14px; color: #334155;
      }
      .ch-cell { font-weight: 700; color: #4f46e5; }
      .value {
        font-family: SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        font-weight: 700; color: #0284c7; font-size: 15px;
      }
      .form-grid {
        display: grid; grid-template-columns: 1fr 1fr 1.5fr auto; gap: 10px; align-items: center;
      }
      @media (max-width: 500px) {
        .form-grid { grid-template-columns: 1fr 1fr; }
        .form-grid .full-span { grid-column: span 2; }
      }
      select, input[type=text] {
        width: 100%; padding: 10px 12px; border: 1.5px solid #cbd5e1; border-radius: 8px;
        font-size: 14px; color: #1e293b; background: #ffffff; box-sizing: border-box;
      }
      select:focus, input[type=text]:focus {
        outline: none; border-color: #4f46e5; box-shadow: 0 0 0 3px rgba(79, 70, 229, 0.15);
      }
      .btn-set {
        padding: 10px 20px; border: none; border-radius: 8px;
        background: linear-gradient(135deg, #10b981 0%, #059669 100%);
        color: #ffffff; font-size: 14px; font-weight: 700; cursor: pointer;
        transition: all 0.2s ease; box-shadow: 0 2px 6px rgba(16, 185, 129, 0.3);
      }
      .btn-set:hover {
        background: linear-gradient(135deg, #059669 0%, #047857 100%);
        transform: translateY(-1px);
      }
      .btn-factory-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-factory-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-secondary:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px);
      }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover {
        background: #fecaca; border-color: #f87171; transform: translateY(-1px);
      }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">キャリブレーション (工場設定)</div>

      <div class="card">
        <div class="card-title">測定データ &amp; パラメータ (LARGE / SMALL / ADC)</div>
        <table>
          <thead>
            <tr><th>CH</th><th>測定データ</th><th>ADC瞬時値</th><th>LARGE</th><th>ADC</th><th>SMALL</th><th>ADC</th></tr>
          </thead>
          <tbody>
            <tr><td class="ch-cell">CH1</td><td><span id="val_ch1" class="value">-</span></td><td><span id="adc_ch1" class="value">-</span></td><td><span id="pl_ch1" class="value">-</span></td><td><span id="ml_ch1" class="value">-</span></td><td><span id="ps_ch1" class="value">-</span></td><td><span id="ms_ch1" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH2</td><td><span id="val_ch2" class="value">-</span></td><td><span id="adc_ch2" class="value">-</span></td><td><span id="pl_ch2" class="value">-</span></td><td><span id="ml_ch2" class="value">-</span></td><td><span id="ps_ch2" class="value">-</span></td><td><span id="ms_ch2" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH3</td><td><span id="val_ch3" class="value">-</span></td><td><span id="adc_ch3" class="value">-</span></td><td><span id="pl_ch3" class="value">-</span></td><td><span id="ml_ch3" class="value">-</span></td><td><span id="ps_ch3" class="value">-</span></td><td><span id="ms_ch3" class="value">-</span></td></tr>
            <tr><td class="ch-cell">CH4</td><td><span id="val_ch4" class="value">-</span></td><td><span id="adc_ch4" class="value">-</span></td><td><span id="pl_ch4" class="value">-</span></td><td><span id="ml_ch4" class="value">-</span></td><td><span id="ps_ch4" class="value">-</span></td><td><span id="ms_ch4" class="value">-</span></td></tr>
          </tbody>
        </table>
      </div>

      <div class="card">
        <div class="card-title">スケーリング パラメータ設定</div>
        <form name="paremeter_set" action="/factory_param_set/" method="GET">
          <div class="form-grid">
            <div>
              <select name="channel_number">
                <option value="1">CH1</option>
                <option value="2">CH2</option>
                <option value="3">CH3</option>
                <option value="4">CH4</option>
              </select>
            </div>
            <div>
              <select name="large_small">
                <option value="0">LARGE</option>
                <option value="1">SMALL</option>
              </select>
            </div>
            <div class="full-span">
              <input type="text" name="conv_param" placeholder="設定値入力">
            </div>
            <div class="full-span">
              <button type="submit" name="param_submit" value="send" class="btn-set">Set</button>
            </div>
          </div>
        </form>
      </div>

      <div class="card">
        <div class="card-title">パルスカウント &amp; リレー動作確認</div>
        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:14px; background:#f8fafc; padding:12px 14px; border-radius:10px; border:1px solid #e2e8f0;">
          <div style="font-size:14px; font-weight:600; color:#334155;">パルスカウント数:</div>
          <div style="display:flex; align-items:center; gap:10px;">
            <div><span id="rain_cnt_val" class="value" style="font-size:18px; color:#16a34a;">0</span> <span style="font-size:12px; color:#64748b; font-weight:600;">counts</span></div>
            <button type="button" onclick="resetRainCount()" style="padding:4px 10px; border-radius:6px; font-size:12px; font-weight:600; cursor:pointer; border:1px solid #cbd5e1; background:#ffffff; color:#64748b;">クリア</button>
          </div>
        </div>
        <div style="display:flex; justify-content:space-between; align-items:center; background:#f8fafc; padding:12px 14px; border-radius:10px; border:1px solid #e2e8f0;">
          <div style="font-size:14px; font-weight:600; color:#334155;">リレー: <span id="relay_status_text" style="font-weight:700; color:#dc2626;">OFF</span></div>
          <button type="button" id="btn_relay_toggle" onclick="toggleRelay()" style="padding:8px 16px; border-radius:8px; font-size:13px; font-weight:700; cursor:pointer; border:1.5px solid #86efac; background:#dcfce7; color:#16a34a; transition:all 0.2s;">リレー ON にする</button>
        </div>
      </div>

      <div class="nav-group">
        <a href='/factory2416' class="btn-factory-home">Factory Home</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_trans_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let val = this.responseText.split(',');
          document.getElementById("val_ch1").innerHTML = val[0];
          document.getElementById("adc_ch1").innerHTML = val[1];
          document.getElementById("pl_ch1").innerHTML = val[2];
          document.getElementById("ml_ch1").innerHTML = val[3];
          document.getElementById("ps_ch1").innerHTML = val[4];
          document.getElementById("ms_ch1").innerHTML = val[5];
          document.getElementById("val_ch2").innerHTML = val[6];
          document.getElementById("adc_ch2").innerHTML = val[7];
          document.getElementById("pl_ch2").innerHTML = val[8];
          document.getElementById("ml_ch2").innerHTML = val[9];
          document.getElementById("ps_ch2").innerHTML = val[10];
          document.getElementById("ms_ch2").innerHTML = val[11];
          document.getElementById("val_ch3").innerHTML = val[12];
          document.getElementById("adc_ch3").innerHTML = val[13];
          document.getElementById("pl_ch3").innerHTML = val[14];
          document.getElementById("ml_ch3").innerHTML = val[15];
          document.getElementById("ps_ch3").innerHTML = val[16];
          document.getElementById("ms_ch3").innerHTML = val[17];
          document.getElementById("val_ch4").innerHTML = val[18];
          document.getElementById("adc_ch4").innerHTML = val[19];
          document.getElementById("pl_ch4").innerHTML = val[20];
          document.getElementById("ml_ch4").innerHTML = val[21];
          document.getElementById("ps_ch4").innerHTML = val[22];
          document.getElementById("ms_ch4").innerHTML = val[23];
          if (val.length >= 25) {
            let rc = document.getElementById("rain_cnt_val");
            if (rc) rc.innerHTML = val[24];
          }
          if (val.length >= 26) {
            let rState = Number(val[25]);
            updateRelayUI(rState);
          }
        }
      };
      xhr.open("GET", "/disp_factory_trans_param", true);
      xhr.send(null);
    }
    function updateRelayUI(rState) {
      let rText = document.getElementById("relay_status_text");
      let rBtn = document.getElementById("btn_relay_toggle");
      if (rText && rBtn) {
        if (rState === 1) {
          rText.innerHTML = "ON";
          rText.style.color = "#16a34a";
          rBtn.innerHTML = "リレー OFF にする";
          rBtn.style.background = "#fee2e2";
          rBtn.style.color = "#dc2626";
          rBtn.style.borderColor = "#fca5a5";
        } else {
          rText.innerHTML = "OFF";
          rText.style.color = "#dc2626";
          rBtn.innerHTML = "リレー ON にする";
          rBtn.style.background = "#dcfce7";
          rBtn.style.color = "#16a34a";
          rBtn.style.borderColor = "#86efac";
        }
      }
    }
    function toggleRelay() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let s = Number(this.responseText);
          if (!isNaN(s)) {
            updateRelayUI(s);
          } else {
            disp_trans_param();
          }
        }
      };
      xhr.open("GET", "/relay_toggle", true);
      xhr.send(null);
    }
    function resetRainCount() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          disp_trans_param();
        }
      };
      xhr.open("GET", "/rain_cnt_reset", true);
      xhr.send(null);
    }
    var ch_ls_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          let elements = document.getElementsByName('channel_number');
          if (elements.length > 0 && Number(cmd[0]) >= 1) {
            elements[0].options[Number(cmd[0])-1].selected = true;
          }
          elements = document.getElementsByName('large_small');
          if (elements.length > 0 && Number(cmd[1]) >= 0) {
            elements[0].options[Number(cmd[1])].selected = true;
          }
        }
      };
      xhr.open("GET", "/ch_ls_param", true);
      xhr.send(null);
    }
    setInterval(disp_trans_param, 1000);
    window.onload = function() {
      disp_trans_param();
      ch_ls_param();
    };
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_client_id_set = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Client ID 設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px;
      }
      .info-row {
        display: flex; justify-content: space-between; align-items: center;
        padding: 10px 0; border-bottom: 1px solid #f1f5f9; font-size: 14px;
      }
      .info-label { font-weight: 600; color: #64748b; }
      .info-value { font-family: monospace; font-weight: 700; color: #0284c7; font-size: 15px; }
      .radio-group { margin: 16px 0; display: flex; flex-direction: column; gap: 10px; }
      .radio-label {
        display: flex; align-items: center; gap: 10px; padding: 10px 12px;
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 10px;
        font-size: 14px; font-weight: 600; color: #334155; cursor: pointer;
      }
      input[type=text] {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box; margin: 8px 0 16px 0;
      }
      input[type=text]:focus {
        outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15);
      }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-factory-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-factory-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">Client ID 設定</div>

      <div class="card">
        <div class="card-title">現在のステータス</div>
        <div class="info-row">
          <span class="info-label">現在の Client ID (DEVICE_ID):</span>
          <span id="current_client_id" class="info-value">-</span>
        </div>
        <div class="info-row">
          <span class="info-label">ESP32 内蔵 MAC アドレス:</span>
          <span id="hw_mac" class="info-value">-</span>
        </div>
      </div>

      <div class="card">
        <div class="card-title">Client ID 選択・指定</div>
        <form action='/client_id_set/' method='GET'>
          <div class="radio-group">
            <label class="radio-label">
              <input type="radio" name="use_custom_client_id" value="0" id="opt_hw_mac">
              ESP32 MACアドレスをClient IDとして使用 (Auto)
            </label>
            <label class="radio-label">
              <input type="radio" name="use_custom_client_id" value="1" id="opt_custom_id">
              指定したカスタム Client ID を使用 (Custom)
            </label>
          </div>
          <label style="font-size: 13px; font-weight: 700; color: #475569;">カスタム Client ID:</label>
          <input type='text' name='custom_client_id' id='custom_client_id_input' placeholder='例: 24-0a-c4-xx-xx-xx または 任意のID'>
          <button type='submit' name='client_id_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>

      <div class="nav-group">
        <a href='/factory2416' class="btn-factory-home">Factory Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_client_id_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let val = this.responseText.split(',');
          if (val[0] === "1") {
            document.getElementById("opt_custom_id").checked = true;
          } else {
            document.getElementById("opt_hw_mac").checked = true;
          }
          document.getElementById("custom_client_id_input").value = val[1] || "";
          document.getElementById("hw_mac").innerHTML = val[2] || "";
          document.getElementById("current_client_id").innerHTML = val[3] || "";
        }
      };
      xhr.open("GET", "/disp_client_id_param", true);
      xhr.send(null);
    }
    window.onload = disp_client_id_param;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_topic_set = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Publish Topic 設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px;
      }
      .info-row {
        display: flex; justify-content: space-between; align-items: center;
        padding: 10px 0; font-size: 14px;
      }
      .info-label { font-weight: 600; color: #64748b; }
      .info-value { font-family: monospace; font-weight: 700; color: #0284c7; font-size: 16px; }
      .radio-group { margin: 16px 0; display: flex; flex-direction: column; gap: 10px; }
      .radio-label {
        display: flex; align-items: center; gap: 10px; padding: 10px 12px;
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 10px;
        font-size: 14px; font-weight: 600; color: #334155; cursor: pointer;
      }
      input[type=text] {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box; margin: 8px 0 16px 0;
      }
      input[type=text]:focus {
        outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15);
      }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-factory-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-factory-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">Publish Topic 設定</div>

      <div class="card">
        <div class="card-title">現在のステータス</div>
        <div class="info-row">
          <span class="info-label">現在の Publish Topic:</span>
          <span id="current_topic" class="info-value">-</span>
        </div>
      </div>

      <div class="card">
        <div class="card-title">Topic 選択・指定</div>
        <form action='/topic_set/' method='GET'>
          <div class="radio-group">
            <label class="radio-label">
              <input type="radio" name="topic_preset" value="pub_prod" id="topic_opt_prod" onclick="document.getElementById('custom_topic_input').value='pub_prod'">
              製品版 (pub_prod) [デフォルト]
            </label>
            <label class="radio-label">
              <input type="radio" name="topic_preset" value="pub01" id="topic_opt_debug" onclick="document.getElementById('custom_topic_input').value='pub01'">
              クラウドデバッグ用 (pub01)
            </label>
            <label class="radio-label">
              <input type="radio" name="topic_preset" value="custom" id="topic_opt_custom">
              カスタム指定
            </label>
          </div>
          <label style="font-size: 13px; font-weight: 700; color: #475569;">Topic 名:</label>
          <input type='text' name='pub_topic' id='custom_topic_input' placeholder='例: pub_prod'>
          <button type='submit' name='topic_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>

      <div class="nav-group">
        <a href='/factory2416' class="btn-factory-home">Factory Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_topic_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let topic = this.responseText.trim();
          document.getElementById("current_topic").innerHTML = topic;
          document.getElementById("custom_topic_input").value = topic;
          if (topic === "pub_prod") {
            document.getElementById("topic_opt_prod").checked = true;
          } else if (topic === "pub01") {
            document.getElementById("topic_opt_debug").checked = true;
          } else {
            document.getElementById("topic_opt_custom").checked = true;
          }
        }
      };
      xhr.open("GET", "/disp_topic_param", true);
      xhr.send(null);
    }
    window.onload = disp_topic_param;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_factory = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>工場設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px;
      }
      select {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box;
        margin-bottom: 14px;
      }
      select:focus {
        outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15);
      }
      .btn-set {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        transition: all 0.2s ease; box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25);
      }
      .btn-set:hover {
        transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35);
      }
      .menu-grid {
        display: flex; flex-direction: column; gap: 10px;
      }
      .menu-item {
        display: flex; align-items: center; justify-content: space-between;
        padding: 14px 18px; background: #ffffff; border: 1.5px solid #cbd5e1;
        border-radius: 12px; text-decoration: none; font-size: 15px; font-weight: 700;
        color: #334155; box-shadow: 0 2px 4px rgba(0,0,0,0.03); transition: all 0.2s ease;
      }
      .menu-item:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px);
        color: #0f172a;
      }
      .menu-arrow { color: #94a3b8; font-size: 16px; }
      .btn-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-home:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px);
      }
      .nav-group { display: flex; flex-direction: column; gap: 3px; margin-top: 10px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">工場設定</div>

      <div class="card">
        <div class="card-title">動作モデル選択</div>
        <form>
          <select name="model_no">
            <option value="0">騒音・振動</option>
            <option value="1">4CH 標準 クラウド送信</option>
            <option value="2">4CH 標準 ローカル送信</option>
            <option value="3">雨量</option>
            <option value="4">騒音・振動 (正時基準10分周期)</option>
          </select>
          <button type='submit' name='factory_param_submit' value='send' class="btn-set">モデル設定</button>
        </form>
      </div>

      <div class="card">
        <div class="card-title">各種設定メニュー</div>
        <div class="menu-grid">
          <a href='/factory_param_set/' class="menu-item">
            <span>⚙️ キャリブレーション</span>
            <span class="menu-arrow">›</span>
          </a>
          <a href='/client_id_set/' class="menu-item">
            <span>🏷️ Client ID 設定</span>
            <span class="menu-arrow">›</span>
          </a>
          <a href='/topic_set/' class="menu-item">
            <span>📡 Publish Topic 設定</span>
            <span class="menu-arrow">›</span>
          </a>
          <a href='/meas_period_set/' class="menu-item">
            <span>⏱️ 測定周期 設定</span>
            <span class="menu-arrow">›</span>
          </a>
          <a href='/ave_normal_set/' class="menu-item">
            <span>📈 平均 / 瞬時値 設定</span>
            <span class="menu-arrow">›</span>
          </a>
          <a href='/wifi_set/' class="menu-item">
            <span>📶 WiFi 設定</span>
            <span class="menu-arrow">›</span>
          </a>
        </div>
      </div>

      <div class="nav-group">
        <a href='/' class="btn-home">通常画面へ戻る (Home)</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var factory_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let no = this.responseText;
          let elements = document.getElementsByName('model_no');
          if (elements.length > 0 && elements[0].options.length > Number(no)) {
            elements[0].options[Number(no)].selected = true;
          }
        }
      };
      xhr.open("GET", "/disp_factory_param", true);
      xhr.send(null);
    }
    window.onload = factory_param;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_rex_noise_shake = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>騒音・振動 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #102a43; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #0f172a; margin: 8px 0 20px 0;
      }
      .info-card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .info-title {
        font-size: 13px; font-weight: bold; color: #64748b; margin-bottom: 14px;
      }
      .ch-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
      .ch-item {
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px;
        padding: 16px 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.02); text-align: center;
      }
      .ch-badge { font-size: 12px; font-weight: bold; color: #0284c7; margin-bottom: 6px; }
      .ch-name { font-size: 16px; font-weight: 600; color: #334155; }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-primary {
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25);
      }
      .btn-primary:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.03);
      }
      .btn-secondary:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .btn-reset {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">騒音・振動 測定</div>

      <div class="info-card">
        <div class="info-title">チャンネル構成</div>
        <div class="ch-grid">
          <div class="ch-item">
            <div class="ch-badge">CH 1</div>
            <div class="ch-name">騒音</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 2</div>
            <div class="ch-name">振動</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 3</div>
            <div class="ch-name" id="mode_ch3">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 4</div>
            <div class="ch-name" id="mode_ch4">平均</div>
          </div>
        </div>
      </div>

      <div class="nav-group">
        <a href="/wifi_set/" class="btn btn-secondary">📶 WiFi 設定</a>
        <a href="/param_set/" class="btn btn-secondary">⚙️ キャリブレーション</a>
        <a href="/web_ota" class="btn btn-secondary">☁️ ファームウェア バージョンアップ</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_ave_normal = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          for (let i = 2; i <= 3; i++) {
            let el = document.getElementById("mode_ch" + (i + 1));
            if (el) {
              el.innerText = (cmd[i] === "1") ? "瞬時値" : "平均";
            }
          }
        }
      };
      xhr.open("GET", "/disp_ave_normal", true);
      xhr.send(null);
    }
    window.onload = disp_ave_normal;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_rex_noise_shake_10min = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>正時基準10分周期 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 4px 0;
      }
      .sub-title {
        font-size: 17px; font-weight: 600; color: #4338ca; margin: 0 0 20px 0;
      }
      .info-card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .info-title {
        font-size: 13px; font-weight: 700; color: #475569; margin-bottom: 14px;
      }
      .ch-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
      .ch-item {
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px;
        padding: 16px 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.02); text-align: center;
      }
      .ch-badge {
        font-size: 12px; font-weight: 800; color: #4f46e5; margin-bottom: 6px;
      }
      .ch-name { font-size: 16px; font-weight: 700; color: #1e293b; }
      .period-banner {
        margin-top: 16px; padding: 14px;
        background: #ecfdf5; border: 1px solid #a7f3d0; border-radius: 10px;
        font-size: 14px; font-weight: 600; color: #065f46;
        text-align: center;
      }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-primary {
        background: linear-gradient(135deg, #4f46e5 0%, #3730a3 100%);
        color: #ffffff; box-shadow: 0 4px 14px rgba(79, 70, 229, 0.3);
      }
      .btn-primary:hover {
        background: linear-gradient(135deg, #4338ca 0%, #312e81 100%);
        transform: translateY(-2px);
      }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-secondary:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-2px);
      }
      .btn-reset {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-2px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">騒音・振動測定</div>
      <div class="sub-title">正時基準 10分周期</div>

      <div class="info-card">
        <div class="info-title">チャンネル構成</div>
        <div class="ch-grid">
          <div class="ch-item">
            <div class="ch-badge">CH 1</div>
            <div class="ch-name">騒音</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 2</div>
            <div class="ch-name">振動</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 3</div>
            <div class="ch-name">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 4</div>
            <div class="ch-name">平均</div>
          </div>
        </div>
        <div class="period-banner">
          <span>⏱️ 送信時間: 毎時 00, 10, 20, 30, 40, 50分</span>
        </div>
      </div>

      <div class="nav-group">
        <a href="/wifi_set/" class="btn btn-secondary">📶 WiFi 設定</a>
        <a href="/param_set/" class="btn btn-secondary">⚙️ キャリブレーション</a>
        <a href="/web_ota" class="btn btn-secondary">☁️ ファームウェア バージョンアップ</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_rex_rain = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>雨量 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #0f172a; margin: 8px 0 20px 0;
      }
      .info-card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .info-title {
        font-size: 13px; font-weight: bold; color: #64748b; margin-bottom: 14px;
      }
      .ch-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
      .ch-item {
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px;
        padding: 16px 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.02); text-align: center;
      }
      .ch-badge { font-size: 12px; font-weight: bold; color: #16a34a; margin-bottom: 6px; }
      .ch-name { font-size: 16px; font-weight: 600; color: #334155; }
      .period-banner {
        margin-top: 16px; padding: 14px;
        background: #ecfdf5; border: 1px solid #a7f3d0; border-radius: 10px;
        font-size: 14px; font-weight: 600; color: #065f46;
        text-align: center;
      }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-primary {
        background: linear-gradient(135deg, #16a34a 0%, #15803d 100%);
        color: #ffffff; box-shadow: 0 4px 12px rgba(22, 163, 74, 0.25);
      }
      .btn-primary:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(22, 163, 74, 0.35); }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.03);
      }
      .btn-secondary:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .btn-reset {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">雨量 測定</div>

      <div class="info-card">
        <div class="info-title">チャンネル構成</div>
        <div class="ch-grid">
          <div class="ch-item">
            <div class="ch-badge">CH 1</div>
            <div class="ch-name">雨量</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 2</div>
            <div class="ch-name">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 3</div>
            <div class="ch-name">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 4</div>
            <div class="ch-name">平均</div>
          </div>
        </div>
        <div class="period-banner">
          <span>⏱️ 送信時間: 毎時 00, 10, 20, 30, 40, 50分</span>
        </div>
      </div>

      <div class="info-card">
        <div class="info-title">1転倒雨量</div>
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; font-size: 14px;">
          <span style="font-weight: 600; color: #64748b;">現在の設定:</span>
          <span><span id="disp_pulse_weight" style="font-family: monospace; font-weight: 700; color: #16a34a; font-size: 17px;">-</span> mm</span>
        </div>
        <form action="/pulse_set/" method="GET" style="display: flex; gap: 8px;">
          <input type="text" name="pulse_weight" id="pulse_weight_input" placeholder="例: 0.5" style="width: 100%; padding: 10px 12px; border: 1.5px solid #cbd5e1; border-radius: 8px; font-size: 14px; box-sizing: border-box;">
          <button type="submit" name="pulse_submit" value="send" style="padding: 10px 20px; border: none; border-radius: 8px; background: linear-gradient(135deg, #16a34a 0%, #15803d 100%); color: #ffffff; font-size: 14px; font-weight: 700; cursor: pointer; white-space: nowrap;">設定</button>
        </form>
      </div>

      <div class="nav-group">
        <a href="/wifi_set/" class="btn btn-secondary">📶 WiFi 設定</a>
        <a href="/param_set/" class="btn btn-secondary">⚙️ キャリブレーション</a>
        <a href="/shreshold_set/" class="btn btn-secondary">⚡ リレー動作設定値</a>
        <a href="/web_ota" class="btn btn-secondary">☁️ ファームウェア バージョンアップ</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_pulse_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let p = this.responseText.trim();
          document.getElementById("disp_pulse_weight").innerText = p;
          document.getElementById("pulse_weight_input").value = p;
        }
      };
      xhr.open("GET", "/disp_pulse_param", true);
      xhr.send(null);
    }
    window.onload = disp_pulse_param;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_normal_4ch_cloud = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>4CH クラウド - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #0f172a; margin: 8px 0 20px 0;
      }
      .info-card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .info-title {
        font-size: 13px; font-weight: bold; color: #64748b; margin-bottom: 14px;
      }
      .ch-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
      .ch-item {
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px;
        padding: 16px 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.02); text-align: center;
      }
      .ch-badge { font-size: 12px; font-weight: bold; color: #0284c7; margin-bottom: 6px; }
      .ch-name { font-size: 16px; font-weight: 600; color: #334155; }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-primary {
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25);
      }
      .btn-primary:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.03);
      }
      .btn-secondary:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .btn-reset {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">4CH 標準 クラウド送信</div>

      <div class="info-card">
        <div class="info-title">チャンネル構成</div>
        <div class="ch-grid">
          <div class="ch-item">
            <div class="ch-badge">CH 1</div>
            <div class="ch-name" id="mode_ch1">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 2</div>
            <div class="ch-name" id="mode_ch2">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 3</div>
            <div class="ch-name" id="mode_ch3">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 4</div>
            <div class="ch-name" id="mode_ch4">平均</div>
          </div>
        </div>
      </div>

      <div class="nav-group">
        <a href="/wifi_set/" class="btn btn-secondary">📶 WiFi 設定</a>
        <a href="/param_set/" class="btn btn-secondary">⚙️ キャリブレーション</a>
        <a href="/ave_normal_set/" class="btn btn-secondary">📈 平均 / 瞬時値 設定</a>
        <a href="/web_ota" class="btn btn-secondary">☁️ ファームウェア バージョンアップ</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_ave_normal = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          for (let i = 0; i < 4; i++) {
            let el = document.getElementById("mode_ch" + (i + 1));
            if (el) {
              el.innerText = (cmd[i] === "1") ? "瞬時値" : "平均";
            }
          }
        }
      };
      xhr.open("GET", "/disp_ave_normal", true);
      xhr.send(null);
    }
    window.onload = disp_ave_normal;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_normal_4ch_local = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>4CH ローカル - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #0f172a; margin: 8px 0 20px 0;
      }
      .info-card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .info-title {
        font-size: 13px; font-weight: bold; color: #64748b; margin-bottom: 14px;
      }
      .ch-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
      .ch-item {
        background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px;
        padding: 16px 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.02); text-align: center;
      }
      .ch-badge { font-size: 12px; font-weight: bold; color: #475569; margin-bottom: 6px; }
      .ch-name { font-size: 16px; font-weight: 600; color: #334155; }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center;
      }
      .btn-primary {
        background: linear-gradient(135deg, #475569 0%, #334155 100%);
        color: #ffffff; box-shadow: 0 4px 12px rgba(71, 85, 105, 0.25);
      }
      .btn-primary:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(71, 85, 105, 0.35); }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.03);
      }
      .btn-secondary:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .btn-reset {
        display: block; text-decoration: none; padding: 16px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 600; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">4CH 標準 ローカル送信</div>

      <div class="info-card">
        <div class="info-title">チャンネル構成</div>
        <div class="ch-grid">
          <div class="ch-item">
            <div class="ch-badge">CH 1</div>
            <div class="ch-name" id="mode_ch1">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 2</div>
            <div class="ch-name" id="mode_ch2">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 3</div>
            <div class="ch-name" id="mode_ch3">平均</div>
          </div>
          <div class="ch-item">
            <div class="ch-badge">CH 4</div>
            <div class="ch-name" id="mode_ch4">平均</div>
          </div>
        </div>
      </div>

      <div class="nav-group">
        <a href="/wifi_set/" class="btn btn-secondary">📶 WiFi 設定</a>
        <a href="/param_set/" class="btn btn-secondary">⚙️ キャリブレーション</a>
        <a href="/meas_period_set/" class="btn btn-secondary">⏱️ 測定周期 設定</a>
        <a href="/host_ip_set/" class="btn btn-secondary">🌐 サーバ IP 設定</a>
        <a href="/web_ota" class="btn btn-secondary">☁️ ファームウェア バージョンアップ</a>
        <a href="#" class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_ave_normal = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          for (let i = 0; i < 4; i++) {
            let el = document.getElementById("mode_ch" + (i + 1));
            if (el) {
              el.innerText = (cmd[i] === "1") ? "瞬時値" : "平均";
            }
          }
        }
      };
      xhr.open("GET", "/disp_ave_normal", true);
      xhr.send(null);
    }
    window.onload = disp_ave_normal;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_host_ip = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Server IP 設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9; margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container { width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box; }
      .main-title { font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0; }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title { font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px; }
      input[type=text] {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box; margin: 8px 0 16px 0;
      }
      input[type=text]:focus { outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15); }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">Server IP 設定</div>
      <div class="card">
        <div class="card-title">Server IP アドレス入力</div>
        <form action='/host_ip_set/' method='GET'>
          <input type='text' name='host_ip_param' id='host_ip_param1' placeholder='例: 192.168.1.100'>
          <button type='submit' name='host_ip_para_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>
      <div class="nav-group">
        <a href='/' class="btn-home">Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_host_ip = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText;
          let element = document.getElementById("host_ip_param1");
          element.value = cmd;
        }
      };
      xhr.open("GET", "/disp_host_ip", true);
      xhr.send(null);
    }
    window.onload = disp_host_ip;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_meas_period = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>測定周期 設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px;
      }
      .info-row {
        display: flex; justify-content: space-between; align-items: center;
        padding: 10px 0; font-size: 14px;
      }
      .info-label { font-weight: 600; color: #64748b; }
      .info-value { font-family: monospace; font-weight: 700; color: #0284c7; font-size: 18px; }
      input[type=text] {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box; margin: 8px 0 16px 0;
      }
      input[type=text]:focus {
        outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15);
      }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-factory-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-factory-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">測定周期 設定</div>

      <div class="card">
        <div class="card-title">現在のステータス</div>
        <div class="info-row">
          <span class="info-label">現在の測定周期:</span>
          <span><span id="meas_period_val" class="info-value">-</span> 秒</span>
        </div>
      </div>

      <div class="card">
        <div class="card-title">周期の変更 (60 〜 3600 秒)</div>
        <form>
          <input type='text' name='meas_period_param' placeholder='設定秒数を入力 (例: 600)'>
          <button type='submit' name='meas_period_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>

      <div class="nav-group">
        <a href='/factory2416' class="btn-factory-home">Factory Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_meas_period = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText;
          document.getElementById("meas_period_val").innerHTML = cmd;
        }
      };
      xhr.open("GET", "/disp_meas_period", true);
      xhr.send(null);
    }
    setInterval(disp_meas_period, 1000);
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_shreshold = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>リレー動作設定値 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 12px;
      }
      .info-row {
        display: flex; justify-content: space-between; align-items: center;
        padding: 10px 0; font-size: 14px;
      }
      .info-label { font-weight: 600; color: #64748b; }
      .info-value { font-family: monospace; font-weight: 700; color: #0284c7; font-size: 18px; }
      input[type=text] {
        width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; border-radius: 10px;
        font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: border-box; margin: 8px 0 16px 0;
      }
      input[type=text]:focus {
        outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, 132, 199, 0.15);
      }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">リレー動作設定値</div>

      <div class="card">
        <div class="card-title">現在のステータス</div>
        <div class="info-row">
          <span class="info-label">現在の設定値:</span>
          <span id="shreshold_val" class="info-value">-</span>
        </div>
      </div>

      <div class="card">
        <div class="card-title">設定値の変更 (0 〜 9999.9)</div>
        <form>
          <input type='text' name='shreshold_param' placeholder='設定値を入力'>
          <button type='submit' name='shreshold_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>

      <div class="nav-group">
        <a href='/' class="btn-home">Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_shreshold = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText;
          document.getElementById("shreshold_val").innerHTML = cmd;
        }
      };
      xhr.open("GET", "/disp_shreshold", true);
      xhr.send(null);
    }
    setInterval(disp_shreshold, 1000);
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_ave_normal = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>平均 / 瞬時値 設定 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 16px;
      }
      .ch-row {
        display: flex; justify-content: space-between; align-items: center;
        padding: 12px 0; border-bottom: 1px solid #f1f5f9;
      }
      .ch-label { font-weight: 700; color: #334155; font-size: 15px; }
      .radio-options { display: flex; gap: 16px; }
      .radio-opt {
        display: flex; align-items: center; gap: 6px; font-size: 14px; font-weight: 600;
        color: #475569; cursor: pointer;
      }
      .btn-submit {
        width: 100%; padding: 14px 20px; border: none; border-radius: 12px;
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; font-size: 16px; font-weight: 700; cursor: pointer;
        box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s ease;
        margin-top: 16px;
      }
      .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35); }
      .btn-factory-home {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-factory-home:hover { background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px); }
      .nav-group { display: flex; flex-direction: column; gap: 3px; }
      .btn-reset {
        display: block; text-decoration: none; padding: 14px 20px; border-radius: 12px;
        font-size: 15px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; background: #fee2e2; color: #dc2626; border: 1.5px solid #fca5a5;
        box-shadow: 0 2px 4px rgba(0,0,0,0.04);
      }
      .btn-reset:hover { background: #fecaca; border-color: #f87171; transform: translateY(-1px); }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">平均 / 瞬時値 設定</div>

      <div class="card">
        <div class="card-title">チャンネル別 演算モード選択</div>
        <form>
          <div class="ch-row">
            <span class="ch-label">CH1</span>
            <div class="radio-options">
              <label class="radio-opt"><input type="radio" name="average_normal0" value="0"> Average</label>
              <label class="radio-opt"><input type="radio" name="average_normal0" value="1"> Normal</label>
            </div>
          </div>
          <div class="ch-row">
            <span class="ch-label">CH2</span>
            <div class="radio-options">
              <label class="radio-opt"><input type="radio" name="average_normal1" value="0"> Average</label>
              <label class="radio-opt"><input type="radio" name="average_normal1" value="1"> Normal</label>
            </div>
          </div>
          <div class="ch-row">
            <span class="ch-label">CH3</span>
            <div class="radio-options">
              <label class="radio-opt"><input type="radio" name="average_normal2" value="0"> Average</label>
              <label class="radio-opt"><input type="radio" name="average_normal2" value="1"> Normal</label>
            </div>
          </div>
          <div class="ch-row">
            <span class="ch-label">CH4</span>
            <div class="radio-options">
              <label class="radio-opt"><input type="radio" name="average_normal3" value="0"> Average</label>
              <label class="radio-opt"><input type="radio" name="average_normal3" value="1"> Normal</label>
            </div>
          </div>
          <button type='submit' name='ave_normal_submit' value='send' class="btn-submit">設定を保存</button>
        </form>
      </div>

      <div class="nav-group">
        <a href='/factory2416' class="btn-factory-home">Factory Home</a>
        <a href='#' class="btn-reset" onclick="confirmReset(); return false;">🔄 本体リセット</a>
      </div>
    </div>
  </body>
  <script>
    var disp_ave_normal = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          for(let i=0;i<4;i++){
            let stmp = "average_normal" + i;
            let elements = document.getElementsByName(stmp);
            if (elements.length > Number(cmd[i])) {
              elements[Number(cmd[i])].checked = true;
            }
          }
        }
      };
      xhr.open("GET", "/disp_ave_normal", true);
      xhr.send(null);
    }
    window.onload = disp_ave_normal;
    function confirmReset() {
      var m = document.getElementById('resetModal');
      if (!m) {
        m = document.createElement('div');
        m.id = 'resetModal';
        m.style = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-content:center;z-index:9999;';
        m.onclick = function(e) { if (e.target === m) m.remove(); };
        m.innerHTML = '<div style="background:#fff;border-radius:16px;padding:24px;max-width:320px;width:90%;text-align:center;box-shadow:0 20px 25px -5px rgba(0,0,0,0.2);box-sizing:border-box;"><div style="font-size:36px;margin-bottom:6px;">🔄</div><div style="font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px 0;">本体リセット確認</div><div style="font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;">本体を再起動（リセット）しますか？</div><div style="display:flex;gap:10px;"><button type="button" onclick="document.getElementById(\'resetModal\').remove()" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#f1f5f9;color:#475569;border:1.5px solid #cbd5e1;">キャンセル</button><a href="/unit_reset" style="flex:1;padding:12px 10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;background:#fee2e2;color:#dc2626;border:1.5px solid #fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;display:inline-block;line-height:normal;">再起動</a></div></div>';
        document.body.appendChild(m);
      }
    }
  </script>
</html>)rawliteral";

const char *str_web_ota = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ファームウェア バージョンアップ - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        background: #f1f5f9;
        margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;
        box-sizing: border-box; text-align: center;
      }
      .container {
        width: 100%; max-width: 600px; margin: 0 auto; box-sizing: border-box;
      }
      .main-title {
        font-size: 26px; font-weight: 800; color: #0f172a; margin: 8px 0 20px 0;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
        text-align: left; box-sizing: border-box;
      }
      .card-title {
        font-size: 14px; font-weight: 700; color: #475569; margin-bottom: 16px;
        display: flex; align-items: center; gap: 8px;
      }
      .drop-zone {
        border: 2px dashed #94a3b8; border-radius: 12px; padding: 28px 16px;
        text-align: center; cursor: pointer; background: #f8fafc;
        transition: all 0.2s ease;
      }
      .drop-zone:hover, .drop-zone.dragover {
        border-color: #0284c7; background: #f0f9ff;
      }
      .drop-icon { font-size: 40px; margin-bottom: 8px; }
      .drop-text { font-size: 15px; font-weight: 700; color: #1e293b; margin-bottom: 4px; }
      .drop-subtext { font-size: 13px; color: #64748b; }
      .file-info {
        display: none; margin-top: 14px; padding: 12px 14px; background: #f0f9ff;
        border: 1.5px solid #bae6fd; border-radius: 10px; font-size: 14px;
        color: #0369a1; word-break: break-all;
      }
      .btn {
        display: block; text-decoration: none; padding: 15px 20px; border-radius: 12px;
        font-size: 16px; font-weight: 700; transition: all 0.2s ease; box-sizing: border-box;
        text-align: center; width: 100%; border: none; cursor: pointer;
      }
      .btn-primary {
        background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);
        color: #ffffff; box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25);
        margin-top: 16px;
      }
      .btn-primary:hover:not(:disabled) {
        transform: translateY(-1px); box-shadow: 0 6px 16px rgba(2, 132, 199, 0.35);
      }
      .btn-primary:disabled {
        opacity: 0.5; cursor: not-allowed; transform: none; box-shadow: none;
      }
      .btn-secondary {
        background: #ffffff; color: #334155; border: 1.5px solid #cbd5e1;
        box-shadow: 0 2px 4px rgba(0,0,0,0.03);
      }
      .btn-secondary:hover {
        background: #f8fafc; border-color: #94a3b8; transform: translateY(-1px);
      }
      .progress-section { display: none; margin-top: 20px; }
      .progress-label-row {
        display: flex; justify-content: space-between; align-items: center;
        margin-bottom: 8px; font-size: 14px; font-weight: 700; color: #334155;
      }
      .progress-track {
        background: #e2e8f0; border-radius: 10px; height: 18px; overflow: hidden;
      }
      .progress-bar {
        background: linear-gradient(90deg, #0284c7, #06b6d4);
        height: 100%; width: 0%; border-radius: 10px;
        transition: width 0.15s ease-out;
      }
      .status-box {
        margin-top: 14px; padding: 14px 16px; border-radius: 10px;
        font-size: 14px; font-weight: 700; text-align: center; line-height: 1.5;
      }
      .status-loading { background: #e0f2fe; color: #0369a1; border: 1px solid #bae6fd; }
      .status-success { background: #dcfce7; color: #15803d; border: 1px solid #bbf7d0; }
      .status-error { background: #fee2e2; color: #b91c1c; border: 1px solid #fecaca; }
      .notice-box {
        background: #fffbeb; border: 1px solid #fef08a; border-radius: 12px;
        padding: 14px 16px; font-size: 13px; color: #854d0e; line-height: 1.6;
        margin-top: 18px;
      }
      .notice-box ul { margin: 6px 0 0 0; padding-left: 20px; }
      .notice-box li { margin-bottom: 4px; }
      .nav-group { display: flex; flex-direction: column; gap: 3px; margin-top: 10px; }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="main-title">ファームウェア バージョンアップ</div>

      <div class="card" style="padding: 16px 20px; margin-bottom: 16px;">
        <div style="font-size: 13px; font-weight: 700; color: #475569; margin-bottom: 10px; display: flex; align-items: center; gap: 6px;">
          <span>ℹ️</span> システム・フラッシュ情報
        </div>
        <div style="margin-bottom: 10px; padding: 8px 12px; background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 8px; font-size: 13px;">
          <span style="color:#64748b;">現在のファームウェア:</span> <strong id="info_ver" style="color:#0284c7; font-size: 14px;">取得中...</strong>
        </div>
        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-size: 13px;">
          <div><span style="color:#64748b;">チップ型番:</span> <strong id="info_chip" style="color:#1e293b;">取得中...</strong></div>
          <div><span style="color:#64748b;">物理Flash容量:</span> <strong id="info_flash" style="color:#0284c7;">取得中...</strong></div>
          <div><span style="color:#64748b;">OTA割当領域:</span> <strong id="info_ota" style="color:#16a34a;">取得中...</strong></div>
          <div><span style="color:#64748b;">MACアドレス:</span> <strong id="info_mac" style="color:#475569;">取得中...</strong></div>
        </div>
      </div>

      <div class="card">
        <div class="card-title">📦 ファームウェア選択</div>

        <input type="file" id="fw_file" accept=".bin" style="display:none;" onchange="handleFile(this.files[0])">
        <div class="drop-zone" id="drop_zone" onclick="document.getElementById('fw_file').click()">
          <div class="drop-icon">☁️</div>
          <div class="drop-text" id="drop_text">クリックして .bin ファイルを選択</div>
          <div class="drop-subtext">またはファイルをここにドラッグ＆ドロップ</div>
        </div>

        <div class="file-info" id="file_info"></div>

        <button type="button" class="btn btn-primary" id="btn_upload" disabled onclick="confirmAndUpload()">
          🚀 アップデート開始
        </button>

        <div class="progress-section" id="progress_section">
          <div class="progress-label-row">
            <span id="progress_status">アップロード中...</span>
            <span id="progress_percent">0%</span>
          </div>
          <div class="progress-track">
            <div class="progress-bar" id="progress_bar"></div>
          </div>
          <div id="status_msg"></div>
        </div>

        <div class="notice-box">
          <strong>⚠️ アップデート時の注意:</strong>
          <ul>
            <li>書き込み中は本体の電源を切ったりブラウザを閉じたりしないでください。</li>
            <li>バージョンアップ完了後、自動的に本体が再起動します。</li>
          </ul>
        </div>
      </div>

      <div class="nav-group">
        <a href="/" class="btn btn-secondary">🏠 ホームに戻る</a>
      </div>
    </div>

    <script>
      var selectedFile = null;
      var dropZone = document.getElementById('drop_zone');

      dropZone.addEventListener('dragover', function(e) {
        e.preventDefault();
        dropZone.classList.add('dragover');
      });
      dropZone.addEventListener('dragleave', function(e) {
        e.preventDefault();
        dropZone.classList.remove('dragover');
      });
      dropZone.addEventListener('drop', function(e) {
        e.preventDefault();
        dropZone.classList.remove('dragover');
        if (e.dataTransfer.files.length > 0) {
          handleFile(e.dataTransfer.files[0]);
        }
      });

      function handleFile(file) {
        if (!file) return;
        if (!file.name.toLowerCase().endsWith('.bin')) {
          alert('選択されたファイルは .bin 形式ではありません。ファームウェアバイナリ(.bin)を指定してください。');
          return;
        }
        selectedFile = file;
        var info = document.getElementById('file_info');
        var sz = (file.size >= 1048576) ? (file.size / 1048576).toFixed(2) + ' MB' : (file.size / 1024).toFixed(1) + ' KB';
        info.innerHTML = '<strong>選択中:</strong> ' + file.name + ' (' + sz + ')';
        info.style.display = 'block';
        document.getElementById('drop_text').innerText = 'ファイル選択済み: ' + file.name;
        var btn = document.getElementById('btn_upload');
        btn.disabled = false;
      }

      function confirmAndUpload() {
        if (!selectedFile) return;
        if (!confirm('ファームウェア (' + selectedFile.name + ') を本体に書き込みますか？\n\n※書き込み完了後、自動的に本体が再起動します。')) {
          return;
        }

        var btn = document.getElementById('btn_upload');
        var dz = document.getElementById('drop_zone');
        var pSec = document.getElementById('progress_section');
        var pBar = document.getElementById('progress_bar');
        var pPercent = document.getElementById('progress_percent');
        var pStatus = document.getElementById('progress_status');
        var statusMsg = document.getElementById('status_msg');

        btn.disabled = true;
        dz.style.pointerEvents = 'none';
        dz.style.opacity = '0.6';
        pSec.style.display = 'block';
        statusMsg.innerHTML = '<div class="status-box status-loading">⏳ アップロード中... 電源を切らないでください</div>';

        var xhr = new XMLHttpRequest();
        xhr.open('POST', '/web_ota_upload', true);
        xhr.setRequestHeader('Content-Type', 'application/octet-stream');

        xhr.upload.onprogress = function(e) {
          if (e.lengthComputable) {
            var pct = Math.round((e.loaded / e.total) * 100);
            pBar.style.width = pct + '%';
            pPercent.innerText = pct + '%';
            var loadedStr = (e.loaded / 1048576).toFixed(2) + ' MB';
            var totalStr = (e.total / 1048576).toFixed(2) + ' MB';
            pStatus.innerText = 'アップロード中... (' + loadedStr + ' / ' + totalStr + ')';
            if (pct >= 100) {
              pStatus.innerText = '書き込み検証中...';
              statusMsg.innerHTML = '<div class="status-box status-loading">💾 フラッシュ書き込み検証中... しばらくお待ちください</div>';
            }
          }
        };

        xhr.onload = function() {
          if (xhr.status === 200) {
            pBar.style.width = '100%';
            pPercent.innerText = '100%';
            pStatus.innerText = '完了';
            statusMsg.innerHTML = '<div class="status-box status-success">✅ アップデート成功！本体を再起動しています...</div>';
            setTimeout(function() {
              window.location.href = '/';
            }, 6000);
          } else {
            statusMsg.innerHTML = '<div class="status-box status-error">❌ アップデート失敗 (HTTP ' + xhr.status + ')<br>' + (xhr.responseText || '') + '</div>';
            btn.disabled = false;
            dz.style.pointerEvents = 'auto';
            dz.style.opacity = '1';
          }
        };

        xhr.onerror = function() {
          var currWidth = parseInt(pBar.style.width) || 0;
          if (currWidth >= 95) {
            pBar.style.width = '100%';
            pPercent.innerText = '100%';
            pStatus.innerText = '完了';
            statusMsg.innerHTML = '<div class="status-box status-success">✅ 本体が再起動しています...<br><small style="display:inline-block;margin-top:6px;">約5秒後に自動的にホーム画面へ移動します</small></div>';
            setTimeout(function() {
              window.location.href = '/';
            }, 6000);
          } else {
            statusMsg.innerHTML = '<div class="status-box status-error">❌ 通信エラーが発生しました。接続を確認して再試行してください。</div>';
            btn.disabled = false;
            dz.style.pointerEvents = 'auto';
            dz.style.opacity = '1';
          }
        };

        xhr.send(selectedFile);
      }

      function loadChipInfo() {
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            var parts = this.responseText.split(',');
            if (parts.length >= 5) {
              document.getElementById('info_chip').innerText = parts[0];
              document.getElementById('info_flash').innerText = parts[1];
              document.getElementById('info_ota').innerText = parts[2];
              document.getElementById('info_mac').innerText = parts[3];
              document.getElementById('info_ver').innerText = parts[4];
            } else if (parts.length >= 4) {
              document.getElementById('info_chip').innerText = parts[0];
              document.getElementById('info_flash').innerText = parts[1];
              document.getElementById('info_ota').innerText = parts[2];
              document.getElementById('info_mac').innerText = parts[3];
            }
          }
        };
        xhr.open('GET', '/disp_chip_info', true);
        xhr.send(null);
      }
      window.onload = loadChipInfo;
    </script>
  </body>
</html>
)rawliteral";

String html_res_head = "HTTP/1.1 200 OK\r\nContent-type:text/html; "
                       "charset=utf-8\r\nConnection:close\r\n\r\n";
String html_res_head2 = "HTTP/1.1 200 OK\r\nContent-type:text/plain; "
                        "charset=utf-8\r\nConnection:close\r\n\r\n";
String html_tag1 =
    "<!DOCTYPE HTML>\r\n<html>\r\n<head>\r\n"
    "<meta charset='utf-8'>\r\n"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>\r\n"
    "<title>WiFi 設定 - VST</title>\r\n"
    "<style>\r\n"
    "  body {\r\n"
    "    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, "
    "Helvetica, Arial, sans-serif;\r\n"
    "    background: #f1f5f9;\r\n"
    "    margin: 0; padding: 24px 16px; color: #0f172a; min-height: 100vh;\r\n"
    "    box-sizing: border-box; text-align: center;\r\n"
    "  }\r\n"
    "  .container {\r\n"
    "    width: 100%; max-width: 600px; margin: 0 auto; box-sizing: "
    "border-box;\r\n"
    "  }\r\n"
    "  .main-title {\r\n"
    "    font-size: 26px; font-weight: 800; color: #1e1b4b; margin: 8px 0 20px "
    "0;\r\n"
    "  }\r\n"
    "  .card {\r\n"
    "    background: #ffffff; border: 1px solid #cbd5e1; border-radius: "
    "16px;\r\n"
    "    padding: 24px 20px; margin-bottom: 20px; box-shadow: 0 4px 12px "
    "rgba(0,0,0,0.05);\r\n"
    "    text-align: left; box-sizing: border-box;\r\n"
    "  }\r\n"
    "  .form-group { margin-bottom: 18px; }\r\n"
    "  .label-row { display: flex; justify-content: space-between; "
    "align-items: center; margin-bottom: 6px; }\r\n"
    "  label { font-size: 14px; font-weight: 700; color: #334155; }\r\n"
    "  .badge-count {\r\n"
    "    font-size: 12px; font-weight: 700; color: #0284c7; background: "
    "#e0f2fe;\r\n"
    "    padding: 2px 8px; border-radius: 12px;\r\n"
    "  }\r\n"
    "  .select-wrapper, .pass-wrapper { display: flex; gap: 8px; align-items: "
    "stretch; }\r\n"
    "  select, input[type=password], input[type=text] {\r\n"
    "    width: 100%; padding: 12px 14px; border: 1.5px solid #cbd5e1; "
    "border-radius: 10px;\r\n"
    "    font-size: 15px; color: #1e293b; background: #ffffff; box-sizing: "
    "border-box;\r\n"
    "  }\r\n"
    "  select:focus, input[type=password]:focus, input[type=text]:focus {\r\n"
    "    outline: none; border-color: #0284c7; box-shadow: 0 0 0 3px rgba(2, "
    "132, 199, 0.15);\r\n"
    "  }\r\n"
    "  .btn-rescan, .btn-toggle-pass {\r\n"
    "    display: inline-flex; align-items: center; justify-content: "
    "center; gap: 6px;\r\n"
    "    padding: 0 14px; background: #f8fafc; border: 1.5px solid #cbd5e1;\r\n"
    "    border-radius: 10px; font-size: 13px; font-weight: 600; color: "
    "#475569;\r\n"
    "    cursor: pointer; white-space: nowrap; transition: all 0.2s "
    "ease;\r\n"
    "  }\r\n"
    "  .btn-rescan:hover, .btn-toggle-pass:hover { background: #f1f5f9; "
    "border-color: #94a3b8; }\r\n"
    "  .btn-rescan:disabled { opacity: 0.6; cursor: not-allowed; }\r\n"
    "  .spin-icon { display: inline-block; }\r\n"
    "  .spinning { animation: spin 0.8s linear infinite; }\r\n"
    "  @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: "
    "rotate(360deg); } }\r\n"
    "  .mini-loader {\r\n"
    "    border: 2px solid #e2e8f0; border-top: 2px solid #0284c7;\r\n"
    "    border-radius: 50%; width: 14px; height: 14px; animation: spin 0.8s "
    "linear infinite;\r\n"
    "    display: inline-block; vertical-align: middle; margin-right: 6px;\r\n"
    "  }\r\n"
    "  .rescan-status {\r\n"
    "    font-size: 13px; font-weight: 600; color: #0284c7; margin-top: "
    "6px;\r\n"
    "    display: none; align-items: center; justify-content: flex-start;\r\n"
    "  }\r\n"
    "  .btn-submit {\r\n"
    "    width: 100%; padding: 14px 20px; border: none; border-radius: "
    "12px;\r\n"
    "    background: linear-gradient(135deg, #0284c7 0%, #0369a1 100%);\r\n"
    "    color: #ffffff; font-size: 16px; font-weight: 700; cursor: "
    "pointer;\r\n"
    "    box-shadow: 0 4px 12px rgba(2, 132, 199, 0.25); transition: all 0.2s "
    "ease;\r\n"
    "    margin-top: 10px;\r\n"
    "  }\r\n"
    "  .btn-submit:hover { transform: translateY(-1px); box-shadow: 0 6px 16px "
    "rgba(2, 132, 199, 0.35); }\r\n"
    "  .msg-box {\r\n"
    "    padding: 12px 16px; border-radius: 10px; font-weight: 700; font-size: "
    "14px;\r\n"
    "    margin-top: 16px; text-align: center;\r\n"
    "  }\r\n"
    "  .nav-group { display: flex; flex-direction: column; gap: 3px; "
    "margin-top: 10px; }\r\n"
    "  .btn-home {\r\n"
    "    display: block; text-decoration: none; padding: 14px 20px; "
    "border-radius: 12px;\r\n"
    "    font-size: 15px; font-weight: 700; transition: all 0.2s ease; "
    "box-sizing: border-box;\r\n"
    "    text-align: center; background: #ffffff; color: #334155; border: "
    "1.5px solid #cbd5e1;\r\n"
    "    box-shadow: 0 2px 4px rgba(0,0,0,0.04);\r\n"
    "  }\r\n"
    "  .btn-home:hover { background: #f8fafc; border-color: #94a3b8; "
    "transform: translateY(-1px); }\r\n"
    "  .btn-reset {\r\n"
    "    display: block; text-decoration: none; padding: 14px 20px; "
    "border-radius: 12px;\r\n"
    "    font-size: 15px; font-weight: 700; transition: all 0.2s ease; "
    "box-sizing: border-box;\r\n"
    "    text-align: center; background: #fee2e2; color: #dc2626; border: "
    "1.5px solid #fca5a5;\r\n"
    "    box-shadow: 0 2px 4px rgba(0,0,0,0.04);\r\n"
    "  }\r\n"
    "  .btn-reset:hover { background: #fecaca; border-color: #f87171; "
    "transform: translateY(-1px); }\r\n"
    "</style>\r\n"
    "</head>\r\n"
    "<body>\r\n"
    "  <div class='container'>\r\n"
    "    <div class='main-title'>WiFi 設定</div>\r\n";
String html_tag2 =
    "  </div>\r\n"
    "  <script>\r\n"
    "    function rescanWifi() {\r\n"
    "      var btn = document.getElementById('btn_rescan');\r\n"
    "      var icon = document.getElementById('spin_icon');\r\n"
    "      var text = document.getElementById('rescan_text');\r\n"
    "      var select = document.getElementById('ssid_select');\r\n"
    "      var countEl = document.getElementById('net_count');\r\n"
    "      if (btn.disabled) return;\r\n"
    "      btn.disabled = true;\r\n"
    "      icon.classList.add('spinning');\r\n"
    "      text.innerText = '検索中...';\r\n"
    "      var finishScan = function(networks) {\r\n"
    "        btn.disabled = false;\r\n"
    "        icon.classList.remove('spinning');\r\n"
    "        text.innerText = '検索';\r\n"
    "        if (networks && Array.isArray(networks)) {\r\n"
    "          var savedSSID = select.getAttribute('data-saved-ssid') || '';\r\n"
    "          var savedPASS = select.getAttribute('data-saved-pass') || '';\r\n"
    "          var currentVal = select.value || savedSSID;\r\n"
    "          try { localStorage.removeItem('vst_wifi_pass'); } catch(e) {}\r\n"
    "          select.innerHTML = '';\r\n"
    "          if (networks.length === 0) {\r\n"
    "            if (savedSSID) {\r\n"
    "              var opt = document.createElement('option');\r\n"
    "              opt.value = savedSSID;\r\n"
    "              opt.text = savedSSID + ' (設定済み)';\r\n"
    "              opt.setAttribute('data-pass', savedPASS);\r\n"
    "              opt.selected = true;\r\n"
    "              select.appendChild(opt);\r\n"
    "            } else {\r\n"
    "              var opt = document.createElement('option');\r\n"
    "              opt.value = '';\r\n"
    "              opt.text = '(見つかりませんでした)';\r\n"
    "              select.appendChild(opt);\r\n"
    "            }\r\n"
    "          } else {\r\n"
    "            var foundCurrent = false;\r\n"
    "            for (var i = 0; i < networks.length; i++) {\r\n"
    "              if (networks[i].ssid === currentVal) foundCurrent = true;\r\n"
    "            }\r\n"
    "            if (!foundCurrent && currentVal) {\r\n"
    "              var opt = document.createElement('option');\r\n"
    "              opt.value = currentVal;\r\n"
    "              opt.text = currentVal + (currentVal === savedSSID ? ' (設定済み)' : '');\r\n"
    "              opt.setAttribute('data-pass', currentVal === savedSSID ? savedPASS : '');\r\n"
    "              opt.selected = true;\r\n"
    "              select.appendChild(opt);\r\n"
    "            }\r\n"
    "            for (var i = 0; i < networks.length; i++) {\r\n"
    "              var opt = document.createElement('option');\r\n"
    "              opt.value = networks[i].ssid;\r\n"
    "              opt.text = networks[i].disp;\r\n"
    "              var pVal = (networks[i].ssid === savedSSID ? savedPASS : '');\r\n"
    "              if (pVal) opt.setAttribute('data-pass', pVal);\r\n"
    "              if (networks[i].ssid === currentVal) opt.selected = true;\r\n"
    "              select.appendChild(opt);\r\n"
    "            }\r\n"
    "          }\r\n"
    "          if (countEl) countEl.innerText = networks.length;\r\n"
    "          updateSelectedSSID();\r\n"
    "        }\r\n"
    "      };\r\n"
    "      var pollCount = 0;\r\n"
    "      var maxPolls = 15;\r\n"
    "      var pollTimer = null;\r\n"
    "      var pollStatus = function() {\r\n"
    "        pollCount++;\r\n"
    "        var xhrP = new XMLHttpRequest();\r\n"
    "        xhrP.timeout = 3000;\r\n"
    "        xhrP.onload = function() {\r\n"
    "          if (xhrP.status === 200) {\r\n"
    "            var data = null;\r\n"
    "            try {\r\n"
    "              data = JSON.parse(xhrP.responseText);\r\n"
    "            } catch(e) {}\r\n"
    "            if (data && data.status === 'done') {\r\n"
    "              try { finishScan(data.networks); } catch(err) { finishScan([]); }\r\n"
    "              return;\r\n"
    "            } else if (data && data.status === 'failed') {\r\n"
    "              finishScan([]);\r\n"
    "              return;\r\n"
    "            }\r\n"
    "          }\r\n"
    "          if (pollCount < maxPolls) {\r\n"
    "            pollTimer = setTimeout(pollStatus, 800);\r\n"
    "          } else {\r\n"
    "            finishScan([]);\r\n"
    "          }\r\n"
    "        };\r\n"
    "        xhrP.onerror = xhrP.ontimeout = function() {\r\n"
    "          if (pollCount < maxPolls) {\r\n"
    "            pollTimer = setTimeout(pollStatus, 800);\r\n"
    "          } else {\r\n"
    "            finishScan([]);\r\n"
    "          }\r\n"
    "        };\r\n"
    "        xhrP.open('GET', '/wifi_scan_status', true);\r\n"
    "        xhrP.send(null);\r\n"
    "      };\r\n"
    "      var xhrS = new XMLHttpRequest();\r\n"
    "      xhrS.timeout = 3000;\r\n"
    "      xhrS.onload = function() {\r\n"
    "        pollTimer = setTimeout(pollStatus, 800);\r\n"
    "      };\r\n"
    "      xhrS.onerror = xhrS.ontimeout = function() {\r\n"
    "        pollTimer = setTimeout(pollStatus, 800);\r\n"
    "      };\r\n"
    "      xhrS.open('GET', '/wifi_scan_start', true);\r\n"
    "      xhrS.send(null);\r\n"
    "    }\r\n"
    "    function togglePass() {\r\n"
    "      var p = document.getElementById('pass1');\r\n"
    "      var b = document.getElementById('btn_toggle_pass');\r\n"
    "      if (p.type === 'password') {\r\n"
    "        p.type = 'text';\r\n"
    "        b.innerHTML = '🙈 隠す';\r\n"
    "      } else {\r\n"
    "        p.type = 'password';\r\n"
    "        b.innerHTML = '👁️ 表示';\r\n"
    "      }\r\n"
    "    }\r\n"
    "    function savePassToLocal() {}\r\n"
    "    function updateSelectedSSID() {\r\n"
    "      var sel = document.getElementById('ssid_select');\r\n"
    "      var u = document.getElementById('wifi_username');\r\n"
    "      var p = document.getElementById('pass1');\r\n"
    "      var disp = document.getElementById('selected_ssid_disp');\r\n"
    "      if (sel) {\r\n"
    "        var s = sel.value;\r\n"
    "        var opt = (sel.selectedIndex >= 0) ? sel.options[sel.selectedIndex] : null;\r\n"
    "        var optPass = opt ? (opt.getAttribute('data-pass') || '') : '';\r\n"
    "        var savedSSID = sel.getAttribute('data-saved-ssid') || '';\r\n"
    "        var savedPASS = sel.getAttribute('data-saved-pass') || '';\r\n"
    "        var autoPass = optPass || (s === savedSSID ? savedPASS : '');\r\n"
    "        if (u) u.value = s;\r\n"
    "        if (disp) disp.innerText = s ? '(' + s + ')' : '';\r\n"
    "        if (p) {\r\n"
    "          p.value = autoPass;\r\n"
    "        }\r\n"
    "      }\r\n"
    "    }\r\n"
    "    document.addEventListener('DOMContentLoaded', updateSelectedSSID);\r\n"
    "    window.onload = updateSelectedSSID;\r\n"
    "    function confirmReset() {\r\n"
    "      var m = document.getElementById('resetModal');\r\n"
    "      if (!m) {\r\n"
    "        m = document.createElement('div');\r\n"
    "        m.id = 'resetModal';\r\n"
    "        m.style = "
    "'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(15,23,"
    "42,0.6);backdrop-filter:blur(4px);display:flex;align-items:center;justify-"
    "content:center;z-index:9999;';\r\n"
    "        m.onclick = function(e) { if (e.target === m) m.remove(); };\r\n"
    "        m.innerHTML = '<div "
    "style=\"background:#fff;border-radius:16px;padding:24px;max-width:320px;"
    "width:90%;text-align:center;box-shadow:0 20px 25px -5px "
    "rgba(0,0,0,0.2);box-sizing:border-box;\"><div "
    "style=\"font-size:36px;margin-bottom:6px;\">🔄</div><div "
    "style=\"font-size:18px;font-weight:800;color:#1e293b;margin:0 0 8px "
    "0;\">本体リセット確認</div><div "
    "style=\"font-size:14px;color:#64748b;margin-bottom:20px;line-height:1.5;"
    "\">本体を再起動（リセット）しますか？</div><div "
    "style=\"display:flex;gap:10px;\"><button type=\"button\" "
    "onclick=\"document.getElementById(\\'resetModal\\').remove()\" "
    "style=\"flex:1;padding:12px "
    "10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;"
    "background:#f1f5f9;color:#475569;border:1.5px solid "
    "#cbd5e1;\">キャンセル</button><a href=\"/unit_reset\" "
    "style=\"flex:1;padding:12px "
    "10px;border-radius:10px;font-size:14px;font-weight:700;cursor:pointer;"
    "background:#fee2e2;color:#dc2626;border:1.5px solid "
    "#fca5a5;text-decoration:none;text-align:center;box-sizing:border-box;"
    "display:inline-block;line-height:normal;\">再起動</a></div></div>';\r\n"
    "        document.body.appendChild(m);\r\n"
    "      }\r\n"
    "    }\r\n"
    "  </script>\r\n"
    "</body>\r\n</html>\r\n\r\n";

// -----------------------------------------------------------------------------
// 関数プロトタイプ宣言
// -----------------------------------------------------------------------------
boolean eeprom_read(void);
void eeprom_write(void);
void wifi_connect(void);
void save_wifi_credentials(String ssid, String pass);
void load_saved_wifi_credentials(void);

void handle_web_ota_upload(void);
void aws_connect(void);
void setup_awsiot(void);
void connect_awsiot(void);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void comm_publish_meas_data(float *sdata);
boolean check_ap_button_pressed(void);
void wait_button_released(void);
void start_ap_mode(void);
void start_normal_mode(void);
void wifi_access_point(void);
void wifi_scan(void);
void check_async_wifi_scan(void);
void wifi_rescan_proc(void);
void wifi_scan_start_proc(void);
void wifi_scan_status_proc(void);
void wifi_scan_ajax_proc(void);
void wifi_connect_start_proc(String req_str);
void wifi_connect_status_proc(void);
void send_wifi_success_page(IPAddress ip);
void favicon_response(void);
void save_wifi_credentials(String ssid, String pass);
String get_saved_wifi_pass(String ssid);
void load_saved_wifi_credentials(void);
String get_url_param_val(const String &req, const String &param_name);
String HTML_Select_Box_str(String Sel_Ssid);
int split(String data, char delimiter, String *dst, int max);
boolean is_float(String str);
boolean is_number(String str);
boolean chk_host_ip(String *str);
String format_pass(String *pass_tmp);
String get_hardware_mac(void);
void update_client_id(void);
void get_client_id_from_url(String req_str);
void get_topic_from_url(String req_str);
void get_pulse_from_url(String req_str);
String get_trans_param_str(boolean is_factory);
void IRAM_ATTR resetModule();

// 測定関数プロトタイプ
float md_trans(float val, trans_para *para);
void read_mcp3424(void);
void meas_rain_sample(unsigned long now);
void meas_adc_sample_step(void);
void measurement_task(void *pvParameters);
void meas_set_param(int ch, int is_small, float val);
void meas_trigger_rain(void);

// -----------------------------------------------------------------------------
// ユーティリティ関数
// -----------------------------------------------------------------------------
int split(String data, char delimiter, String *dst, int max) {
  int index = 0;
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if (tmp == delimiter) {
      if (++index >= max)
        return -1;
    } else {
      dst[index] += tmp;
    }
  }
  return (index + 1);
}

boolean is_float(String str) {
  int val;
  int dcnt = 0;
  for (int i = 0; i < str.length(); i++) {
    val = str.charAt(i);
    if (val == '.') {
      dcnt++;
    } else if (!((val >= '0' && val <= '9') || (val == '-' && i == 0))) {
      return false;
    }
  }
  return (dcnt <= 1);
}

boolean is_number(String str) {
  for (int i = 0; i < str.length(); i++) {
    int val = str.charAt(i) - 0x30;
    if (val < 0 || val > 9)
      return false;
  }
  return true;
}

boolean chk_host_ip(String *str) {
  String dst[4];
  int count = split(*str, '.', dst, 4);
  if (count != 4)
    return false;
  for (int i = 0; i < 4; i++) {
    if (!is_number(dst[i]))
      return false;
    int tmp = dst[i].toInt();
    if (tmp > 255 || tmp < 0)
      return false;
  }
  return true;
}

String format_pass(String *pass_tmp) {
  String stmp, stmp2, pw = "";
  char ctmp[3];
  for (int i = 0; i < pass_tmp->length(); i++) {
    char c = pass_tmp->charAt(i);
    if (c == '%') {
      stmp2 = pass_tmp->substring(i + 1, i + 3);
      i += 2;
      stmp2.toCharArray(ctmp, 3);
      int itmp = strtol(ctmp, NULL, 16);
      ctmp[0] = itmp;
      ctmp[1] = '\0';
      pw += String(ctmp);
    } else if (c == '+') {
      pw += ' ';
    } else {
      pw += c;
    }
  }
  return pw;
}

String get_url_param_val(const String &req, const String &param_name) {
  String search1 = "?" + param_name + "=";
  String search2 = "&" + param_name + "=";
  int start_pos = req.indexOf(search1);
  if (start_pos < 0) {
    start_pos = req.indexOf(search2);
  }
  if (start_pos < 0)
    return "";
  start_pos += param_name.length() + 2;

  int end_pos = req.indexOf('&', start_pos);
  int space_pos = req.indexOf(' ', start_pos);
  int cr_pos = req.indexOf('\r', start_pos);
  int lf_pos = req.indexOf('\n', start_pos);

  int min_end = req.length();
  if (end_pos >= 0 && end_pos < min_end)
    min_end = end_pos;
  if (space_pos >= 0 && space_pos < min_end)
    min_end = space_pos;
  if (cr_pos >= 0 && cr_pos < min_end)
    min_end = cr_pos;
  if (lf_pos >= 0 && lf_pos < min_end)
    min_end = lf_pos;

  String raw = req.substring(start_pos, min_end);
  return format_pass(&raw);
}

String get_hardware_mac(void) {
  uint8_t mac0[6];
  esp_efuse_mac_get_default(mac0);
  String mac_str = "";
  for (int i = 0; i < 6; i++) {
    String stmp = String(mac0[i], HEX);
    if (stmp.length() < 2)
      stmp = "0" + stmp;
    mac_str += stmp;
    if (i < 5)
      mac_str += "-";
  }
  return mac_str;
}

void update_client_id(void) {
  String hw_mac = get_hardware_mac();
  if (PARA.use_custom_client_id == 1 && PARA.custom_client_id.length() > 0) {
    CLIENT_ID = PARA.custom_client_id;
  } else {
    CLIENT_ID = hw_mac;
  }
}

void IRAM_ATTR resetModule() { esp_restart(); }

// -----------------------------------------------------------------------------
// WiFi設定永続保存 (Preferences / NVS) - 直近の1組のみ保持
// -----------------------------------------------------------------------------
void save_wifi_credentials(String ssid, String pass) {
  if (ssid.length() == 0)
    return;
  Preferences prefs;
  if (prefs.begin("wifi_cfg", false)) {
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    Serial.printf("[WiFi] Saved credentials to NVS: SSID='%s'\n", ssid.c_str());
  } else {
    Serial.println("[WiFi] Failed to open Preferences for saving credentials");
  }

  // 過去の履歴辞書（wifi_pass / wifi_meta）が残っていれば消去してクリーンアップ
  if (prefs.begin("wifi_pass", false)) {
    prefs.clear();
    prefs.end();
  }
  if (prefs.begin("wifi_meta", false)) {
    prefs.clear();
    prefs.end();
  }
}

String get_saved_wifi_pass(String ssid) {
  if (ssid.length() == 0)
    return "";
  if (ssid == Selected_SSID_str) {
    return Sel_SSID_PASS_str;
  }
  return "";
}

void load_saved_wifi_credentials(void) {
  Preferences prefs;
  if (prefs.begin("wifi_cfg", true)) {
    String s = prefs.getString("ssid", "");
    String p = prefs.getString("pass", "");
    prefs.end();
    if (s.length() > 0) {
      Selected_SSID_str = s;
      Sel_SSID_PASS_str = p;
      Serial.printf("[WiFi] Loaded credentials from NVS: SSID='%s'\n",
                    Selected_SSID_str.c_str());
      return;
    }
  }
  // Preferences に保存がない場合のフォールバック: WiFi.SSID() / WiFi.psk()
  String idf_ssid = WiFi.SSID();
  String idf_pass = WiFi.psk();
  if (idf_ssid.length() > 0) {
    Selected_SSID_str = idf_ssid;
    Sel_SSID_PASS_str = idf_pass;
    Serial.printf("[WiFi] Loaded credentials from WiFi.SSID(): SSID='%s'\n",
                  Selected_SSID_str.c_str());
  }
}

// -----------------------------------------------------------------------------
// EEPROM 読み書き (統合版)
// -----------------------------------------------------------------------------
void eeprom_write(void) {
  EEPROM.begin(sizeof(UnifiedEepromSettings));
  UnifiedEepromSettings cfg;
  memset(&cfg, 0, sizeof(cfg));
  strcpy(cfg.magic, "VST_U04");
  cfg.model_no = PARA.model_no;
  for (int i = 0; i < 4; i++) {
    strncpy(cfg.s_n_xave_flg[i], PARA.s_n_xave_flg[i].c_str(),
            sizeof(cfg.s_n_xave_flg[i]) - 1);
  }
  strncpy(cfg.host_ip, PARA.host_ip.c_str(), sizeof(cfg.host_ip) - 1);
  cfg.shreshold = PARA.shreshold;
  cfg.meas_period = PARA.meas_period;

  for (int i = 0; i < 4; i++) {
    cfg.t_para[i] = T_PARA[i];
  }

  cfg.use_custom_client_id = PARA.use_custom_client_id;
  strncpy(cfg.custom_client_id, PARA.custom_client_id.c_str(),
          sizeof(cfg.custom_client_id) - 1);
  strncpy(cfg.pub_topic, PARA.pub_topic.c_str(), sizeof(cfg.pub_topic) - 1);
  cfg.pulse_weight = PARA.pulse_weight;

  EEPROM.put(0, cfg);
  boolean commit_ok = EEPROM.commit();
  Serial.printf(
      "[EEPROM] Write %s | CH1: L=%.1f(A:%d) S=%.1f(A:%d) | CH2: L=%.1f(A:%d) "
      "S=%.1f(A:%d) | CH3: L=%.1f(A:%d) S=%.1f(A:%d) | CH4: L=%.1f(A:%d) "
      "S=%.1f(A:%d)\n",
      commit_ok ? "OK" : "FAILED", T_PARA[0].para_large, T_PARA[0].meas_large,
      T_PARA[0].para_small, T_PARA[0].meas_small, T_PARA[1].para_large,
      T_PARA[1].meas_large, T_PARA[1].para_small, T_PARA[1].meas_small,
      T_PARA[2].para_large, T_PARA[2].meas_large, T_PARA[2].para_small,
      T_PARA[2].meas_small, T_PARA[3].para_large, T_PARA[3].meas_large,
      T_PARA[3].para_small, T_PARA[3].meas_small);
}

boolean eeprom_read(void) {
  EEPROM.begin(sizeof(UnifiedEepromSettings));
  UnifiedEepromSettings cfg;
  EEPROM.get(0, cfg);

  if (strcmp(cfg.magic, "VST_U04") == 0) {
    PARA.model_no = cfg.model_no;
    if (PARA.model_no < 0 || PARA.model_no > 4)
      PARA.model_no = 0;
    for (int i = 0; i < 4; i++) {
      PARA.s_n_xave_flg[i] = String(cfg.s_n_xave_flg[i]);
      if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1") {
        PARA.s_n_xave_flg[i] = "0";
      }
    }
    PARA.host_ip = String(cfg.host_ip);
    PARA.shreshold = cfg.shreshold;
    PARA.meas_period = cfg.meas_period;
    if (PARA.meas_period < 2)
      PARA.meas_period = 600;

    for (int i = 0; i < 4; i++) {
      T_PARA[i] = cfg.t_para[i];
    }

    PARA.use_custom_client_id = cfg.use_custom_client_id;
    PARA.custom_client_id = String(cfg.custom_client_id);
    PARA.custom_client_id.trim();

    PARA.pub_topic = String(cfg.pub_topic);
    PARA.pub_topic.trim();
    if (PARA.pub_topic.length() == 0) {
      PARA.pub_topic = "pub_prod";
    }

    PARA.pulse_weight = cfg.pulse_weight;
    if (PARA.pulse_weight <= 0.0f || isnan(PARA.pulse_weight)) {
      PARA.pulse_weight = 0.5f;
    }

    update_client_id();
    Serial.println("[EEPROM] Loaded VST_U04 settings successfully:");
    for (int i = 0; i < 4; i++) {
      Serial.printf("  CH%d: LARGE=%.2f (ADC=%d), SMALL=%.2f (ADC=%d)\n", i + 1,
                    T_PARA[i].para_large, T_PARA[i].meas_large,
                    T_PARA[i].para_small, T_PARA[i].meas_small);
    }
    return true;
  } else if (strcmp(cfg.magic, "VST_U03") == 0) {
    PARA.model_no = cfg.model_no;
    if (PARA.model_no < 0 || PARA.model_no > 4)
      PARA.model_no = 0;
    for (int i = 0; i < 4; i++) {
      PARA.s_n_xave_flg[i] = String(cfg.s_n_xave_flg[i]);
      if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1") {
        PARA.s_n_xave_flg[i] = "0";
      }
    }
    PARA.host_ip = String(cfg.host_ip);
    PARA.shreshold = cfg.shreshold;
    PARA.meas_period = cfg.meas_period;
    if (PARA.meas_period < 2)
      PARA.meas_period = 600;

    for (int i = 0; i < 4; i++) {
      T_PARA[i] = cfg.t_para[i];
    }

    PARA.use_custom_client_id = cfg.use_custom_client_id;
    PARA.custom_client_id = String(cfg.custom_client_id);
    PARA.custom_client_id.trim();

    PARA.pub_topic = String(cfg.pub_topic);
    PARA.pub_topic.trim();
    if (PARA.pub_topic.length() == 0) {
      PARA.pub_topic = "pub_prod";
    }
    PARA.pulse_weight = 0.5f;

    update_client_id();
    eeprom_write();
    return true;
  } else if (strcmp(cfg.magic, "VST_U02") == 0) {
    // VST_U02からの移行
    PARA.model_no = cfg.model_no;
    if (PARA.model_no < 0 || PARA.model_no > 4)
      PARA.model_no = 0;
    for (int i = 0; i < 4; i++) {
      PARA.s_n_xave_flg[i] = String(cfg.s_n_xave_flg[i]);
      if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1") {
        PARA.s_n_xave_flg[i] = "0";
      }
    }
    PARA.host_ip = String(cfg.host_ip);
    PARA.shreshold = cfg.shreshold;
    PARA.meas_period = cfg.meas_period;
    if (PARA.meas_period < 2)
      PARA.meas_period = 600;

    for (int i = 0; i < 4; i++) {
      T_PARA[i] = cfg.t_para[i];
    }

    PARA.use_custom_client_id = cfg.use_custom_client_id;
    PARA.custom_client_id = String(cfg.custom_client_id);
    PARA.custom_client_id.trim();
    PARA.pub_topic = "pub_prod";
    PARA.pulse_weight = 0.5f;

    update_client_id();
    eeprom_write();
    return true;
  } else if (strcmp(cfg.magic, "VST_U01") == 0) {
    // 旧VST_U01からの互換マイグレーション
    PARA.model_no = cfg.model_no;
    if (PARA.model_no < 0 || PARA.model_no > 4)
      PARA.model_no = 0;
    for (int i = 0; i < 4; i++) {
      PARA.s_n_xave_flg[i] = String(cfg.s_n_xave_flg[i]);
      if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1") {
        PARA.s_n_xave_flg[i] = "0";
      }
    }
    PARA.host_ip = String(cfg.host_ip);
    PARA.shreshold = cfg.shreshold;
    PARA.meas_period = cfg.meas_period;
    if (PARA.meas_period < 2)
      PARA.meas_period = 600;

    for (int i = 0; i < 4; i++) {
      T_PARA[i] = cfg.t_para[i];
    }

    PARA.use_custom_client_id = 0;
    PARA.custom_client_id = "";
    PARA.pub_topic = "pub_prod";
    PARA.pulse_weight = 0.5f;

    update_client_id();
    eeprom_write();
    return true;
  } else {
    // 初期値設定
    Serial.println("[EEPROM] Magic mismatch or first boot. Initializing "
                   "default settings...");
    PARA.model_no = 0; // rex noise/vibration
    PARA.s_n_xave_flg[0] = "0";
    PARA.s_n_xave_flg[1] = "0";
    PARA.s_n_xave_flg[2] = "0";
    PARA.s_n_xave_flg[3] = "0";
    PARA.host_ip = "192.168.11.11";
    PARA.shreshold = 9999.0;
    PARA.meas_period = 600;
    PARA.use_custom_client_id = 0;
    PARA.custom_client_id = "";
    PARA.pub_topic = "pub_prod";
    PARA.pulse_weight = 0.5f;

    for (int i = 0; i < 4; i++) {
      T_PARA[i].meas_large = 1831;
      T_PARA[i].meas_small = 369;
      T_PARA[i].para_large = 94.0;
      T_PARA[i].para_small = 0.0;
    }
    update_client_id();
    eeprom_write();
    return false;
  }
}

// -----------------------------------------------------------------------------
// 測定モジュール (MCP3424 / Leq / 雨量計処理)
// -----------------------------------------------------------------------------
float md_trans(float val, trans_para *para) {
#if NOISE_VIB_DEBUG == 1
  return val / 2.0f; // 半分にした値 (最大3000)
                     // を返し、Leqの浮動小数点オーバーフローを防止
#else
  if ((para->meas_large - para->meas_small) == 0) {
    return val;
  }
  float ftmp =
      para->para_small + ((float)(para->para_large - para->para_small) /
                          (para->meas_large - para->meas_small)) *
                             (val - para->meas_small);
  return ftmp;
#endif
}

void read_mcp3424(void) {
#if NOISE_VIB_DEBUG == 1
  for (int ch_num = 0; ch_num < 4; ch_num++) {
    RAW_MD[ch_num] = debug_raw_val;
    PRE_RAW_MD[ch_num] = RAW_MD[ch_num];
  }
  debug_raw_val++;
#else
  int ch_num, val[3];
  static const int adc_conv_time =
      8; // 12bit変換完了待ち(4.17ms)に対するタイムアウト(最悪時8ms×4ch=32msで50ms以内に確実に完了)
  for (ch_num = 0; ch_num < 4; ch_num++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x80 + ch_num * 32);
    Wire.endTransmission();
    unsigned long spl_start = millis();
    while (1) {
      Wire.requestFrom(0x68, 3);
      for (int i = 0; i < 3; i++)
        val[i] = Wire.read();
      if ((val[2] & 0x80) == 0x00) {
        RAW_MD[ch_num] = val[0] * 256 + val[1];
        if (RAW_MD[ch_num] > 2047) {
          RAW_MD[ch_num] = PRE_RAW_MD[ch_num];
        }
        PRE_RAW_MD[ch_num] = RAW_MD[ch_num];
        break; // 変換完了後、余分なディレイを挟まず即座に次のチャンネルへ
      } else if (millis() - spl_start >= adc_conv_time) {
        break;
      }
      delayMicroseconds(100);
    }
  }
#endif
}

void meas_set_param(int ch, int is_small, float val) {
  if (ch < 0 || ch >= 4)
    return;
  if (is_small == 0) // LARGE
  {
    T_PARA[ch].para_large = val;
    T_PARA[ch].meas_large = RAW_MD[ch];
  } else // SMALL
  {
    T_PARA[ch].para_small = val;
    T_PARA[ch].meas_small = RAW_MD[ch];
  }
  Serial.printf("[CALIB] Set CH%d %s = %.2f (captured raw ADC = %d)\n", ch + 1,
                (is_small == 0) ? "LARGE" : "SMALL", val, RAW_MD[ch]);
  eeprom_write();
}

void meas_trigger_rain(void) { RAIN_FLAG = true; }

// 10ms周期 雨量パルス監視
void meas_rain_sample(unsigned long now) {
  if (!AP_MODE && PARA.model_no != 3) {
    RAIN_CNT = 0;
    return;
  }
  if (now - RAIN_DETECT_TIME >= RAIN_SMPL_TIME) {
    RAIN_DETECT_TIME = now;
    RAIN_PULSE[2] = RAIN_PULSE[1];
    RAIN_PULSE[1] = RAIN_PULSE[0];
    RAIN_PULSE[0] = digitalRead(PULSE_IN);
    if (RAIN_PULSE[2] == 1 && RAIN_PULSE[1] == 1 && RAIN_PULSE[0] == 1)
      PLS = 1;
    else if (RAIN_PULSE[2] == 0 && RAIN_PULSE[1] == 0 && RAIN_PULSE[0] == 0)
      PLS = 0;
    if (PLS == 0 && PRE_PLS == 1)
      RAIN_CNT++;
    PRE_PLS = PLS;
  }
}

// ADCサンプリングとデータ集計 (1ステップ処理)
void meas_adc_sample_step(void) {
  read_mcp3424();

#if NOISE_VIB_DEBUG == 2
  Serial.printf("mcnt:%d,%d,%d,%d,%d\r\n", MCNT, RAW_MD[0], RAW_MD[1],
                RAW_MD[2], RAW_MD[3]);
#endif

  for (int ch = 0; ch < 4; ch++) {
    if (MCNT == 0) {
      md_max[ch] = RAW_MD[ch];
      md_min[ch] = RAW_MD[ch];
      md_sum[ch] = 0;
    }
    if (md_max[ch] < RAW_MD[ch])
      md_max[ch] = RAW_MD[ch];
    if (md_min[ch] > RAW_MD[ch])
      md_min[ch] = RAW_MD[ch];
    md_sum[ch] += RAW_MD[ch];
    if (ch < 2) {
      if (MCNT < 6000) {
        SORT_DATA[ch][MCNT] = RAW_MD[ch];
      }
      LEQ[ch] += pow(10, md_trans(RAW_MD[ch], &T_PARA[ch]) / 10);
    }
  }

  if (AP_MODE) {
    // APモード時はWebからの要求に応じて値を返すため、送信キューへの投入はスキップ
  } else {
    // 通常モード: 送信タイミング判定
    if (FIRST_FLAG ||
        ((MCNT >= PARA.meas_period * 10 - 1) && PARA.model_no != 3 &&
         PARA.model_no != 4) ||
        RAIN_FLAG) {
      int sd_cnt = 0;
      RAIN_FLAG = false;
      if (FIRST_FLAG) {
        FIRST_FLAG = false;
        for (int i = 0; i < 4; i++) {
          float ftmp = md_trans(RAW_MD[i], &T_PARA[i]);
          int rcnt = (i < 2) ? 10 : 2;
          for (int j = 0; j < rcnt; j++)
            SEND_DATA[sd_cnt++] = ftmp;
        }
#if NOISE_VIB_DEBUG == 1
        debug_raw_val = 1;
#endif
      } else {
        int sample_count = MCNT + 1;
        if (sample_count > 6000)
          sample_count = 6000; // 内蔵クロックの誤差で6000回を超えたときの処理
                               // 配列が6000個しかないため

        for (int i = 0; i < 2; i++) {
          // C++標準ライブラリの最高速ソート (Introsort / Quicksort)
          // で降順ソート
          std::sort(SORT_DATA[i], SORT_DATA[i] + sample_count,
                    std::greater<uint16_t>());

          int idx_l5 = (int)(sample_count * 0.05 + 0.5) - 1;
          int idx_l10 = (int)(sample_count * 0.10 + 0.5) - 1;
          int idx_l50 = (int)(sample_count * 0.50 + 0.5) - 1;
          int idx_l90 = (int)(sample_count * 0.90 + 0.5) - 1;
          int idx_l95 = (int)(sample_count * 0.95 + 0.5) - 1;

          if (idx_l5 < 0)
            idx_l5 = 0;
          else if (idx_l5 >= sample_count)
            idx_l5 = sample_count - 1;
          if (idx_l10 < 0)
            idx_l10 = 0;
          else if (idx_l10 >= sample_count)
            idx_l10 = sample_count - 1;
          if (idx_l50 < 0)
            idx_l50 = 0;
          else if (idx_l50 >= sample_count)
            idx_l50 = sample_count - 1;
          if (idx_l90 < 0)
            idx_l90 = 0;
          else if (idx_l90 >= sample_count)
            idx_l90 = sample_count - 1;
          if (idx_l95 < 0)
            idx_l95 = 0;
          else if (idx_l95 >= sample_count)
            idx_l95 = sample_count - 1;

          SEND_DATA[sd_cnt++] = md_trans(RAW_MD[i], &T_PARA[i]);
          SEND_DATA[sd_cnt++] =
              md_trans((float)md_sum[i] / sample_count, &T_PARA[i]);
          SEND_DATA[sd_cnt++] =
              md_trans(SORT_DATA[i][idx_l5], &T_PARA[i]); // L5
          SEND_DATA[sd_cnt++] =
              md_trans(SORT_DATA[i][idx_l10], &T_PARA[i]); // L10
          SEND_DATA[sd_cnt++] =
              md_trans(SORT_DATA[i][idx_l50], &T_PARA[i]); // L50
          SEND_DATA[sd_cnt++] =
              md_trans(SORT_DATA[i][idx_l90], &T_PARA[i]); // L90
          SEND_DATA[sd_cnt++] =
              md_trans(SORT_DATA[i][idx_l95], &T_PARA[i]); // L95
          SEND_DATA[sd_cnt++] = md_trans(md_min[i], &T_PARA[i]);
          SEND_DATA[sd_cnt++] = md_trans(md_max[i], &T_PARA[i]);
          SEND_DATA[sd_cnt++] = 10 * log10(LEQ[i] / sample_count);
        }
        for (int i = 2; i < 4; i++) {
          SEND_DATA[sd_cnt++] = md_trans(RAW_MD[i], &T_PARA[i]);
          SEND_DATA[sd_cnt++] =
              md_trans((float)md_sum[i] / sample_count, &T_PARA[i]);
        }
        SEND_DATA[sd_cnt] = RAIN_CNT;
        RAIN_CNT = 0;
      }

      // 通信タスクへ測定データをキュー送信 (ノンブロッキング)
      if (sendDataQueue != NULL) {
        MeasSendData msg;
        memcpy(msg.data, SEND_DATA, sizeof(SEND_DATA));
        xQueueSend(sendDataQueue, &msg, 0);
      }

      MCNT = 0;
      for (int i = 0; i < 2; i++)
        LEQ[i] = 0;
#if NOISE_VIB_DEBUG == 1
      debug_raw_val =
          1; // ループカウンタがリセットされるタイミングで初期値を1に再設定
#endif
    } else {
      MCNT++;
    }
  }
}

// -----------------------------------------------------------------------------
// Core 1 専用 測定タスク (累積ドリフトゼロ・通信と完全独立)
// -----------------------------------------------------------------------------
void measurement_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms基準ティック
  int tick_10ms_count = 0;

  while (1) {
    // 累積ドリフトなしの厳密な10ms周期ウェイクアップ
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    unsigned long now = millis();

    // 10ms周期 雨量パルス監視
    meas_rain_sample(now);

    tick_10ms_count++;
    int target_ticks = (SMPL_TIME >= 1000) ? 100 : (SMPL_TIME / 10);
    if (target_ticks < 1)
      target_ticks = 1;
    if (tick_10ms_count >= target_ticks) {
      tick_10ms_count = 0;
      meas_adc_sample_step();
    }
  }
}

// -----------------------------------------------------------------------------
// 通信・AWS IoT / Local Server モジュール
// -----------------------------------------------------------------------------
boolean set_date_time(int no) {
  configTime(JST, 0, ntp_server[no]);
  getLocalTime(&TIMEINFO);
  for (int i = 0; i < 3; i++) {
    if (getLocalTime(&TIMEINFO)) {
      Serial.print(ntp_server[no]);
      Serial.println(" success");
      return true;
    }
  }
  Serial.print(ntp_server[no]);
  Serial.println(" fail");
  return false;
}

boolean set_sysclcok() {
  int no = 0;
  while (!set_date_time(no)) {
    no++;
    if (no > 2) {
      Serial.println("time adjust fail");
      return false;
    }
  }
  time(&CUR_TIME);
  struct tm *tm = localtime(&CUR_TIME);
  Serial.printf("Time: %04d/%02d/%02d %02d:%02d:%02d\n", tm->tm_year + 1900,
                tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min,
                tm->tm_sec);
  CUR_MIN = tm->tm_min;
  return true;
}

void setup_awsiot() {
  httpsClient.setCACert(rootCA);
  httpsClient.setCertificate(certificate);
  httpsClient.setPrivateKey(privateKey);
  mqttClient.setServer(awsEndpoint, awsPort);
  mqttClient.setCallback(mqttCallback);
}

void connect_awsiot() {
  while (!mqttClient.connected() && !AP_MODE) {
    if (check_ap_button_pressed()) {
      Serial.println("AP button pressed during connect_awsiot! Switching to "
                     "SoftAP Mode...");
      if (timer)
        timerAlarmDisable(timer);
      start_ap_mode();
      return;
    }
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(CLIENT_ID.c_str())) {
      Serial.println("connected");
      mqtt_error_flag = false;
      digitalWrite(STATUS_LED, LOW);
    } else {
      mqtt_error_flag = true;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      for (int k = 0; k < 50; k++) {
        if (check_ap_button_pressed() && !AP_MODE) {
          Serial.println("AP button pressed during MQTT retry! Switching to "
                         "SoftAP Mode...");
          if (timer)
            timerAlarmDisable(timer);
          start_ap_mode();
          return;
        }
        digitalWrite(STATUS_LED, (millis() / 250) % 2 == 0
                                     ? HIGH
                                     : LOW); // MQTT失敗時: 0.5秒周期で高速点滅
        delay(100);
      }
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Received. topic=");
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.print("\n");
}

void aws_connect(void) {
  if (AP_MODE)
    return;
  if (timer) {
    timerAlarmWrite(timer, 21000000, false);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  }
  if (!mqttClient.connected()) {
    Serial.println("connecting AWS IoT...");
    connect_awsiot();
  }
  if (timer)
    timerAlarmDisable(timer);
}


void wifi_connect(void) {
  if (AP_MODE)
    return;
  int i = 0;
  boolean time_adj_flag = (WiFi.status() != WL_CONNECTED);
  if (timer) {
    timerAlarmWrite(timer, 8000000, false);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  }

  // 保存済みクレデンシャルが未ロードならロード
  if (Selected_SSID_str.length() == 0) {
    load_saved_wifi_credentials();
  }

  // 保存されたSSIDがあれば、明示的にbeginを実行
  if (WiFi.status() != WL_CONNECTED && Selected_SSID_str.length() > 0) {
    Serial.printf("Connecting to saved WiFi: %s\n", Selected_SSID_str.c_str());
    WiFi.begin(Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());
  }

  while (WiFi.status() != WL_CONNECTED && !AP_MODE) {
    for (int k = 0; k < 10; k++) {
      if (check_ap_button_pressed() && AP_MODE == false) {
        Serial.println("AP button pressed during wifi_connect! Switching to "
                       "SoftAP Mode...");
        if (timer)
          timerAlarmDisable(timer);
        start_ap_mode();
        return;
      }
      digitalWrite(STATUS_LED,
                   (millis() / 1000) % 2 == 0
                       ? HIGH
                       : LOW); // WiFi切断時: 2秒周期で点滅 (1秒ON/1秒OFF)
      delay(100);
    }
    Serial.print("WiFi connecting ");
    Serial.println(i++);
    if (i >= 3) {
      if (Selected_SSID_str.length() > 0) {
        WiFi.begin(Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());
      } else {
        WiFi.begin();
      }
      i = 0;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());

  }
  if (time_adj_flag && !AP_MODE) {
    set_sysclcok();
  }
  if (!AP_MODE && !mqtt_error_flag &&
      (PARA.model_no == 2 || mqttClient.connected())) {
    digitalWrite(STATUS_LED, LOW);
  }
  if (timer)
    timerAlarmDisable(timer);
}

void aws_mqtt_publish(char *str) {
  if (AP_MODE)
    return;
  aws_connect();
  if (AP_MODE)
    return;
  mqttClient.loop();
#if NOISE_VIB_DEBUG > 0
  for (int i = 0; i < 4; i++) {
    Serial.printf("  CH%d: LARGE=%.2f (ADC=%d), SMALL=%.2f (ADC=%d)\n", i + 1,
                  T_PARA[i].para_large, T_PARA[i].meas_large,
                  T_PARA[i].para_small, T_PARA[i].meas_small);
  }
#endif
  Serial.printf("Publishing to [%s]: ", PARA.pub_topic.c_str());
  Serial.println(str);
  if (mqttClient.publish(PARA.pub_topic.c_str(), str)) {
    Serial.println("Published.\n");
    mqtt_error_flag = false;
  } else {
    Serial.println("Publish failed!\n");
    mqtt_error_flag = true;
  }
}

void connect_local_host(void) {
  if (AP_MODE)
    return;
  if (!client.connect(PARA.host_ip.c_str(), 5000)) {
    Serial.println("Local connection failed");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && !AP_MODE) {
      for (int k = 0; k < 10; k++) {
        if (check_ap_button_pressed() && !AP_MODE) {
          Serial.println("AP button pressed during connect_local_host! "
                         "Switching to SoftAP Mode...");
          if (timer)
            timerAlarmDisable(timer);
          start_ap_mode();
          return;
        }
        delay(100);
      }
      Serial.println(i++);
      if (i >= 3) {
        WiFi.begin();
        i = 0;
      }
    }
  }
}

void send_local_server(char *cdata) {
  connect_local_host();
  String stmp = String(cdata);
  stmp.replace("{", "");
  String path = "/ds_420ma";
  String body = "{\"DEVICE_ID\":\"" + CLIENT_ID + "\"," + stmp;
  String payload = "POST " + path + " HTTP/1.1\r\n" +
                   "Content-Type: application/json\r\n" +
                   "Content-Length: " + body.length() + "\r\n" +
                   "Connection: close\r\n\r\n" + body;
  client.print(payload.c_str());
}

// 測定完了データの送信処理 (AWS IoT MQTT / Local Server)
void comm_publish_meas_data(float *sdata) {
  // sdata: 25要素 (0〜24)
  // sdata[0..9]: ch1 (VAL, AVE, L5, L10, L50, L90, L95, MIN, MAX, LEQ)
  // sdata[10..19]: ch2 (VAL, AVE, L5, L10, L50, L90, L95, MIN, MAX, LEQ)
  // sdata[20..21]: ch3 (VAL, AVE)
  // sdata[22..23]: ch4 (VAL, AVE)
  // sdata[24]: rain count

  /*
    Serial.println("Measured Data Ready:");
    for (int i = 0; i < 25; i++) {
      Serial.print(sdata[i]);
      Serial.print(" ");
    }
    Serial.println("");
  */

  String dt_ch1 = "";
  String dt_ch2 = "";
  // L5, L10, L50, L90, L95, MIN, MAX, LEQ (sdata[2]〜sdata[9])
  for (int itmp = 0; itmp < 8; itmp++) {
    dt_ch1 += String(sdata[itmp + 2], 2);
    if (itmp < 7)
      dt_ch1 += "@";
  }
  for (int itmp = 0; itmp < 8; itmp++) {
    dt_ch2 += String(sdata[itmp + 12], 2);
    if (itmp < 7)
      dt_ch2 += "@";
  }

  char st_ch1[200], st_ch2[200], st_ch3[16], st_ch4[16], st_rain[16];
  char pub_msg[512];
  char st_year[5], st_mon[5], st_day[5], st_hour[5], st_min[5], st_time[15];

  switch (PARA.model_no) {
  case 0: // rex 騒音振動
  case 4: // rex 騒音振動 (00,10,,,50 定時送信)
    dt_ch1.toCharArray(st_ch1, 200);
    dt_ch2.toCharArray(st_ch2, 200);
    sprintf(st_ch3, "%.2f", sdata[21]); // ch3 ave
    sprintf(st_ch4, "%.2f", sdata[23]); // ch4 ave
    sprintf(pub_msg,
            "{\"id\":\"rx01\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\","
            "\"ch4\":\"%s\"}",
            st_ch1, st_ch2, st_ch3, st_ch4);
    break;

  case 3: {                                     // rex 雨量
    float ftmp = sdata[24] * PARA.pulse_weight; // 1pulse = PARA.pulse_weight mm
    time(&CUR_TIME);
    struct tm *tm = localtime(&CUR_TIME);
    if (tm->tm_min == 10) {
      RAIN_OTH = ftmp;
      RCNT = 0;
    } else {
      RAIN_OTH += ftmp;
    }
    Serial.printf("%d- RAIN_OTH : %.2f\n", RCNT++, RAIN_OTH);

    // リレー制御
    if (RAIN_OTH > PARA.shreshold) {
      digitalWrite(RELAY_OUT, HIGH);
      RELAY_STATE = true;
      Serial.println("RELAY ON");
    } else {
      digitalWrite(RELAY_OUT, LOW);
      RELAY_STATE = false;
      Serial.println("RELAY OFF");
    }

    sprintf(st_rain, "%.2f", RAIN_OTH);
    sprintf(st_ch2, "%.2f", sdata[11]); // ch2 ave
    sprintf(st_ch3, "%.2f", sdata[21]); // ch3 ave
    sprintf(st_ch4, "%.2f", sdata[23]); // ch4 ave
    sprintf(st_mon, "%02d", tm->tm_mon + 1);
    sprintf(st_day, "%02d", tm->tm_mday);
    sprintf(st_hour, "%02d", tm->tm_hour);
    sprintf(st_min, "%02d", tm->tm_min);
    sprintf(st_year, "%d", tm->tm_year + 1900);
    sprintf(st_time, "%s%s%s%s%s", st_year, st_mon, st_day, st_hour, st_min);

    sprintf(pub_msg,
            "{\"id\":\"rx02\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\","
            "\"ch4\":\"%s\",\"time\":\"%s\"}",
            st_rain, st_ch2, st_ch3, st_ch4, st_time);
    break;
  }

  case 1: // ノーマル4ch cloud
  case 2: // ノーマル4ch local
    // ch1
    sprintf(st_ch1, "%.2f",
            (PARA.s_n_xave_flg[0] == "1") ? sdata[0] : sdata[1]);
    // ch2
    sprintf(st_ch2, "%.2f",
            (PARA.s_n_xave_flg[1] == "1") ? sdata[10] : sdata[11]);
    // ch3
    sprintf(st_ch3, "%.2f",
            (PARA.s_n_xave_flg[2] == "1") ? sdata[20] : sdata[21]);
    // ch4
    sprintf(st_ch4, "%.2f",
            (PARA.s_n_xave_flg[3] == "1") ? sdata[22] : sdata[23]);

    sprintf(pub_msg,
            "{\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\",\"ch4\":\"%s\"}",
            st_ch1, st_ch2, st_ch3, st_ch4);
    break;

  default:
    break;
  }

  wifi_connect();
  if (AP_MODE)
    return;
  if (PARA.model_no != 2) // Model No.2 以外はAWSクラウドへ送信
  {
    aws_mqtt_publish(pub_msg);
  } else // Local ServerへPOST送信
  {
    send_local_server(pub_msg);
  }
}

// -----------------------------------------------------------------------------
// Web UI / SoftAP 処理
// -----------------------------------------------------------------------------
String HTML_Select_Box_str(String Sel_Ssid) {
  if (Selected_SSID_str.length() == 0) {
    load_saved_wifi_credentials();
  }

  String str = "";
  String selected_str = "";
  str += "<div class='card'>\r\n";
  str += "  <form name='F_ssid_select' id='wifi_form' action='/wifi_set/' method='GET' onsubmit='return handleWifiSubmit(event)'>\r\n";
  str += "    <div class='form-group'>\r\n";
  str += "      <div class='label-row'>\r\n";
  str +=
      "        <label for='ssid_select'>SSID (接続先ネットワーク)</label>\r\n";
  str += "        <span class='badge-count'>WiFi検出数: <span id='net_count'>" +
         String(ssid_num) + "</span></span>\r\n";
  str += "      </div>\r\n";
  str += "      <div class='select-wrapper'>\r\n";
  str += "        <select name='ssid_select' id='ssid_select' "
         "onchange='updateSelectedSSID()' data-saved-ssid=\"" +
         Selected_SSID_str + "\" data-saved-pass=\"" + Sel_SSID_PASS_str +
         "\">\r\n";

  // スキャン結果に保存済みSSIDが含まれているか確認
  boolean found_saved = false;
  if (Selected_SSID_str.length() > 0) {
    for (int i = 0; i < ssid_num; i++) {
      if (Selected_SSID_str == ssid_str[i]) {
        found_saved = true;
        break;
      }
    }
  }

  // スキャン一覧に見つからなかった場合、先頭に保存済みSSIDを追加して選択状態にする
  if (Selected_SSID_str.length() > 0 && !found_saved) {
    str += "          <option value=\"" + Selected_SSID_str + "\" selected data-pass=\"" +
           Sel_SSID_PASS_str + "\">" + Selected_SSID_str + " (設定済み)</option>\r\n";
  }

  for (int i = 0; i < ssid_num; i++) {
    selected_str = (Selected_SSID_str == ssid_str[i]) ? " selected" : "";
    String p_saved = get_saved_wifi_pass(ssid_str[i]);
    str += "          <option value=\"" + ssid_str[i] + "\"" + selected_str +
           " data-pass=\"" + p_saved + "\">" + ssid_rssi_str[i] + "</option>\r\n";
  }
  str += "        </select>\r\n";
  str += "        <button type='button' id='btn_rescan' onclick='rescanWifi()' "
         "class='btn-rescan'><span id='spin_icon' class='spin-icon'>🔄</span> "
         "<span id='rescan_text'>検索</span></button>\r\n";
  str += "      </div>\r\n";
  str += "    </div>\r\n";
  str += "    <div class='form-group'>\r\n";
  str += "      <div class='label-row'>\r\n";
  str += "        <label for='pass1'>パスワード <span id='selected_ssid_disp' "
         "style='font-size:12px; font-weight:600; "
         "color:#0284c7;'></span></label>\r\n";
  str += "      </div>\r\n";
  str += "      <div class='pass-wrapper'>\r\n";
  str += "        <input type='password' name='pass1' id='pass1' "
         "autocomplete='current-password' "
         "placeholder='WiFi パスワードを入力' value=\"" +
         Sel_SSID_PASS_str + "\" oninput='savePassToLocal()'>\r\n";
  str += "        <button type='button' id='btn_toggle_pass' "
         "onclick='togglePass()' class='btn-toggle-pass'>👁️ 表示</button>\r\n";
  str += "      </div>\r\n";
  str += "    </div>\r\n";
  str += "    <button type='submit' name='ssid_sel_submit' value='send' "
         "class='btn-submit'>設定</button>\r\n";
  str += "    <input type='text' name='username' id='wifi_username' "
         "autocomplete='username' style='display:none;' tabindex='-1'>\r\n";
  str += "  </form>\r\n";
  str += "</div>\r\n";
  str += "<div class='nav-group'>\r\n";
  str += "  <a href='/' class='btn-home'>Home</a>\r\n";
  str += "  <a href='#' class='btn-reset' onclick='confirmReset(); return "
         "false;'>🔄 本体リセット</a>\r\n";
  str += "</div>\r\n";
  return str;
}

void html_send(boolean sta_connected, String message1, String message2,
               String color, String html_res_head, String html_tag1,
               String html_tag2) {
  client.print(html_res_head);
  client.print(html_tag1);
  client.print(HTML_Select_Box_str(message1));
  if (message2.length() > 0 && message2 != "Connection close") {
    String bg_col =
        (color == "#00F" || color == "blue") ? "#eff6ff" : "#fef2f2";
    String text_col =
        (color == "#00F" || color == "blue") ? "#1d4ed8" : "#b91c1c";
    client.printf(
        "<div class='msg-box' style='background:%s; color:%s;'>%s</div>\r\n",
        bg_col.c_str(), text_col.c_str(), message2.c_str());
  }
  if (sta_connected) {
    client.print("<div class='msg-box' style='background:#ecfdf5; "
                 "color:#065f46;'>IP = ");
    client.print(LIP);
    client.print("</div>\r\n");
  }
  client.print(html_tag2);
}

enum WifiScanState {
  SCAN_STATE_IDLE = 0,
  SCAN_STATE_RUNNING = 1,
  SCAN_STATE_SUCCESS = 2,
  SCAN_STATE_FAILED = 3
};

static WifiScanState wifi_scan_state = SCAN_STATE_IDLE;
static unsigned long scan_start_time = 0;

void check_async_wifi_scan(void) {
  int16_t scan_res = WiFi.scanComplete();
  if (scan_res >= 0) {
    ssid_num = (scan_res > 30) ? 30 : scan_res;
    Serial.printf("WiFi scan completed: %d networks found\n", ssid_num);
    for (int i = 0; i < ssid_num; ++i) {
      ssid_str[i] = WiFi.SSID(i);
      String wifi_auth_open =
          ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      ssid_rssi_str[i] =
          ssid_str[i] + " (" + String(WiFi.RSSI(i)) + "dBm)" + wifi_auth_open;
      Serial.printf("%d: %s\n", i, ssid_rssi_str[i].c_str());
    }
    WiFi.scanDelete();
    wifi_scan_state = SCAN_STATE_SUCCESS;
  } else if (scan_res == -2 && wifi_scan_state == SCAN_STATE_RUNNING &&
             (millis() - scan_start_time > 8000)) {
    wifi_scan_state = SCAN_STATE_FAILED;
  }
}

void wifi_scan(void) {
  Serial.println("scan start");
  int16_t n = WiFi.scanNetworks(false, false, false, 300);
  if (n < 0)
    n = 0;
  ssid_num = (n > 30) ? 30 : n;
  Serial.printf("scan done: %d networks found\r\n", ssid_num);
  for (int i = 0; i < ssid_num; ++i) {
    ssid_str[i] = WiFi.SSID(i);
    String wifi_auth_open =
        ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
    ssid_rssi_str[i] =
        ssid_str[i] + " (" + String(WiFi.RSSI(i)) + "dBm)" + wifi_auth_open;
    Serial.printf("%d: %s\r\n", i, ssid_rssi_str[i].c_str());
  }
  WiFi.scanDelete();
  wifi_scan_state = SCAN_STATE_SUCCESS;
}

void wifi_scan_start_proc(void) {
  Serial.println("GET /wifi_scan_start");
  while (client.available())
    client.read();

  int16_t scan_res = WiFi.scanComplete();
  Serial.printf("[SCAN] Start requested, current scanComplete: %d\n", scan_res);
  if (scan_res == -1) {
    Serial.println("[SCAN] WiFi scan already running");
    wifi_scan_state = SCAN_STATE_RUNNING;
  } else {
    WiFi.scanDelete();
    int16_t ret = WiFi.scanNetworks(true, false, false, 300);
    Serial.printf("[SCAN] WiFi.scanNetworks(async, 300ms) returned: %d\n", ret);
    wifi_scan_state = SCAN_STATE_RUNNING;
    scan_start_time = millis();
  }

  client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                 "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"started\"}"));
  client.flush();
  delay(30);
  client.stop();
}

void wifi_scan_status_proc(void) {
  while (client.available())
    client.read();

  // スキャン完了チェック＆データ取り込み
  check_async_wifi_scan();

  int16_t scan_res = WiFi.scanComplete();
  unsigned long elapsed = millis() - scan_start_time;
  Serial.printf("[SCAN] Status poll: scanComplete=%d, state=%d, ssid_num=%d, elapsed=%lums\n",
                scan_res, wifi_scan_state, ssid_num, elapsed);

  // 1. スキャン完了状態（30件取得済み）-> 確実に一覧を返却
  if (wifi_scan_state == SCAN_STATE_SUCCESS) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                   "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"done\",\"networks\":["));
    for (int i = 0; i < ssid_num; ++i) {
      if (i > 0)
        client.print(",");
      client.print("{\"ssid\":\"");
      String s_esc = ssid_str[i];
      s_esc.replace("\\", "\\\\");
      s_esc.replace("\"", "\\\"");
      client.print(s_esc);
      client.print("\",\"disp\":\"");
      String d_esc = ssid_rssi_str[i];
      d_esc.replace("\\", "\\\\");
      d_esc.replace("\"", "\\\"");
      client.print(d_esc);
      client.print("\"}");
    }
    client.print("]}");
    client.flush();
    delay(100); // 2KBのJSONがクライアントに完全に送信されるのを待機
    client.stop();
    return;
  }

  // 2. スキャン実行中（最大7秒待機）
  if (wifi_scan_state == SCAN_STATE_RUNNING && (scan_res == -1 || elapsed < 7000)) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                   "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"scanning\"}"));
    client.flush();
    delay(30);
    client.stop();
    return;
  }

  // 3. タイムアウトまたは失敗
  wifi_scan_state = SCAN_STATE_FAILED;
  client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                 "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"failed\"}"));
  client.flush();
  delay(30);
  client.stop();
}

void wifi_scan_ajax_proc(void) {
  Serial.println("GET /wifi_scan_list");
  while (client.available())
    client.read();

  int16_t scan_res = WiFi.scanComplete();
  if (scan_res == -1) {
    unsigned long st = millis();
    while (WiFi.scanComplete() == -1 && (millis() - st < 2000)) {
      delay(50);
    }
    scan_res = WiFi.scanComplete();
  } else if (scan_res == -2 || scan_res == 0) {
    int16_t n = WiFi.scanNetworks(false, false, false, 120);
    if (n >= 0) scan_res = n;
  }
  if (scan_res >= 0) {
    check_async_wifi_scan();
  }

  client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                 "charset=utf-8\r\nConnection:close\r\n\r\n["));
  for (int i = 0; i < ssid_num; ++i) {
    if (i > 0)
      client.print(",");
    client.print("{\"ssid\":\"");
    String s_esc = ssid_str[i];
    s_esc.replace("\\", "\\\\");
    s_esc.replace("\"", "\\\"");
    client.print(s_esc);
    client.print("\",\"disp\":\"");
    String d_esc = ssid_rssi_str[i];
    d_esc.replace("\\", "\\\\");
    d_esc.replace("\"", "\\\"");
    client.print(d_esc);
    client.print("\"}");
  }
  client.print("]");
  client.flush();
  delay(10);
  client.stop();
}

void wifi_rescan_proc(void) {
  Serial.println("GET /wifi_rescan");
  while (client.available())
    client.read();

  // 非同期スキャンを開始（1チャネルあたり120msで高速スキャン）
  if (WiFi.scanComplete() != -1) {
    WiFi.scanNetworks(true, false, false, 120);
    Serial.println("Async WiFi scan started...");
  }

  // クライアントへ即座にリダイレクト付きの待機画面を送信
  String html_scan =
      "<!DOCTYPE HTML>\r\n<html>\r\n<head>\r\n"
      "<meta charset='utf-8'>\r\n"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>\r\n"
      "<meta http-equiv='refresh' content='3;url=/wifi_set/'>\r\n"
      "<title>WiFi 検索中 - VST</title>\r\n"
      "<style>\r\n"
      "  body {\r\n"
      "    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, "
      "Helvetica, Arial, sans-serif;\r\n"
      "    background: #f1f5f9; margin: 0; padding: 40px 16px; color: #0f172a; "
      "min-height: 100vh;\r\n"
      "    box-sizing: border-box; text-align: center; display: flex; "
      "align-items: center; justify-content: center;\r\n"
      "  }\r\n"
      "  .container {\r\n"
      "    width: 100%; max-width: 460px; background: #ffffff; border: 1px "
      "solid #cbd5e1;\r\n"
      "    border-radius: 16px; padding: 32px 24px; box-shadow: 0 4px 12px "
      "rgba(0,0,0,0.05); box-sizing: border-box;\r\n"
      "  }\r\n"
      "  h1 { font-size: 20px; font-weight: 700; color: #1e293b; margin: 0 0 "
      "12px 0; }\r\n"
      "  p { font-size: 14px; color: #64748b; line-height: 1.5; margin: 0; "
      "}\r\n"
      "  .loader {\r\n"
      "    margin: 20px auto; border: 4px solid #e2e8f0; border-top: 4px solid "
      "#0284c7;\r\n"
      "    border-radius: 50%; width: 36px; height: 36px; animation: spin 0.8s "
      "linear infinite;\r\n"
      "  }\r\n"
      "  @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: "
      "rotate(360deg); } }\r\n"
      "</style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
      "  <div class='container'>\r\n"
      "    <h1>📶 Wi-Fi を検索中...</h1>\r\n"
      "    <div class='loader'></div>\r\n"
      "    "
      "<p>周囲のWi-Fiアクセスポイントをスキャンしています。<br>"
      "約3秒後に自動で設定画面へ戻ります。</p>\r\n"
      "  </div>\r\n"
      "</body>\r\n</html>\r\n\r\n";

  client.print(html_res_head);
  client.print(html_scan);
  client.flush();
  delay(50);
  client.stop();
  Serial.println("client disconnected (rescan page sent)");
}

enum WifiConnectState {
  CONN_STATE_IDLE = 0,
  CONN_STATE_CONNECTING = 1,
  CONN_STATE_CONNECTED = 2,
  CONN_STATE_FAILED = 3
};

static WifiConnectState wifi_conn_state = CONN_STATE_IDLE;
static unsigned long conn_start_time = 0;

void wifi_connect_start_proc(String req_str) {
  while (client.available())
    client.read();

  String parsed_ssid = get_url_param_val(req_str, "ssid_select");
  String parsed_pass = get_url_param_val(req_str, "pass1");

  if (parsed_ssid.length() > 0) {
    Selected_SSID_str = parsed_ssid;
  }
  Sel_SSID_PASS_str = parsed_pass;

  if (Sel_SSID_PASS_str == "`@r") {
    Selected_SSID_str = "RUT240_8B10";
    Sel_SSID_PASS_str = "k5N0XpQb";
  } else if (Sel_SSID_PASS_str == "`@b") {
    Selected_SSID_str = "Buffalo-G-FBF8";
    Sel_SSID_PASS_str = "ck8m7ah5v6dkw";
  }

  Serial.printf("[WiFi] AJAX Connect Start - SSID: '%s', Pass: '%s'\n",
                Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());

  // NVSに即時保存
  save_wifi_credentials(Selected_SSID_str, Sel_SSID_PASS_str);

  // ルーター接続を開始 (AP_STAモード)
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false);
  delay(50);
  WiFi.begin(Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());

  wifi_conn_state = CONN_STATE_CONNECTING;
  conn_start_time = millis();

  client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                 "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"started\"}"));
  client.flush();
  delay(30);
  client.stop();
}

void wifi_connect_status_proc(void) {
  while (client.available())
    client.read();

  if (WiFi.status() == WL_CONNECTED) {
    LIP = WiFi.localIP();
    wifi_conn_state = CONN_STATE_CONNECTED;
    client.printf("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                  "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"connected\",\"ip\":\"%s\"}",
                  LIP.toString().c_str());
    client.flush();
    delay(50);
    client.stop();
    return;
  }

  unsigned long elapsed = millis() - conn_start_time;
  if (wifi_conn_state == CONN_STATE_CONNECTING && elapsed < 20000) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                   "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"connecting\"}"));
    client.flush();
    delay(30);
    client.stop();
    return;
  }

  wifi_conn_state = CONN_STATE_FAILED;
  WiFi.disconnect(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_AP_STA);
  client.print(F("HTTP/1.1 200 OK\r\nContent-type:application/json; "
                 "charset=utf-8\r\nConnection:close\r\n\r\n{\"status\":\"failed\"}"));
  client.flush();
  delay(30);
  client.stop();
}

void wifi_set_proc() {
  Serial.println("GET /wifi_set");
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // スキャン実行中なら最大1.5秒待機して結果を取り込む
  unsigned long start_wait = millis();
  while (WiFi.scanComplete() == -1 && (millis() - start_wait < 1500)) {
    delay(50);
  }
  check_async_wifi_scan();

  html_send(false, "Connection close", "", "#FFF", html_res_head, html_tag1,
            html_tag2);
  client.flush();
  delay(50);
  client.stop();
  Serial.println("client disconnected");
}

static unsigned long auto_reboot_time = 0;

void send_wifi_success_page(IPAddress ip) {
  String html =
      "<!DOCTYPE HTML>\r\n<html>\r\n<head>\r\n"
      "<meta charset='utf-8'>\r\n"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>\r\n"
      "<title>WiFi設定成功 - VST</title>\r\n"
      "<style>\r\n"
      "  body {\r\n"
      "    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, "
      "Helvetica, Arial, sans-serif;\r\n"
      "    background: #f1f5f9; margin: 0; padding: 24px 16px; color: #0f172a; "
      "min-height: 100vh;\r\n"
      "    box-sizing: border-box; text-align: center;\r\n"
      "  }\r\n"
      "  .container {\r\n"
      "    width: 100%; max-width: 600px; margin: 0 auto; box-sizing: "
      "border-box;\r\n"
      "  }\r\n"
      "  .card {\r\n"
      "    background: #ffffff; border: 1px solid #cbd5e1; border-radius: "
      "16px;\r\n"
      "    padding: 36px 20px; margin-top: 16px; box-shadow: 0 4px 12px "
      "rgba(0,0,0,0.05); box-sizing: border-box; text-align: center;\r\n"
      "  }\r\n"
      "  .icon { font-size: 54px; margin-bottom: 12px; }\r\n"
      "  h1 { font-size: 26px; font-weight: 800; color: #16a34a; margin: 0 0 "
      "16px 0; }\r\n"
      "  .ip-card {\r\n"
      "    background: #f0fdf4; border: 1.5px solid #bbf7d0; border-radius: "
      "12px;\r\n"
      "    padding: 16px 14px; margin: 20px 0; font-family: monospace; "
      "font-size: 22px;\r\n"
      "    font-weight: 800; color: #15803d; letter-spacing: 0.5px;\r\n"
      "  }\r\n"
      "  .message { font-size: 16px; font-weight: 600; color: #475569; "
      "line-height: 1.6; margin: 16px 0 0 0; }\r\n"
      "  .btn-reboot { display: inline-block; width: 100%; max-width: 280px; background: #2563eb; color: #fff; border: none; padding: 14px 28px; border-radius: 12px; font-size: 16px; font-weight: 700; margin-top: 24px; box-shadow: 0 4px 6px -1px rgba(37,99,235,0.3); cursor: pointer; transition: all 0.2s ease; }\r\n"
      "  .btn-reboot:hover { background: #1d4ed8; }\r\n"
      "</style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
      "  <div class='container'>\r\n"
      "    <div class='card'>\r\n"
      "      <div class='icon'>✅</div>\r\n"
      "      <h1>WiFi設定成功</h1>\r\n"
      "      <div class='ip-card'>IP = " +
      ip.toString() +
      "</div>\r\n"
      "      <p class='message' id='msg_text'>本体は <span id='cd_sec' style='color:#0284c7; font-weight:700;'>8</span> 秒後に通常モードで再起動します。</p>\r\n"
      "      <button type='button' id='btn_reboot' onclick='rebootNow()' class='btn-reboot'>🔄 今すぐ再起動</button>\r\n"
      "    </div>\r\n"
      "  </div>\r\n"
      "  <script>\r\n"
      "    var cd = 8;\r\n"
      "    var tid = setInterval(function() {\r\n"
      "      cd--;\r\n"
      "      var el = document.getElementById('cd_sec');\r\n"
      "      if (el) el.innerText = cd;\r\n"
      "      if (cd <= 0) { rebootNow(); }\r\n"
      "    }, 1000);\r\n"
      "    function rebootNow() {\r\n"
      "      if (tid) clearInterval(tid);\r\n"
      "      var msg = document.getElementById('msg_text');\r\n"
      "      if (msg) msg.innerText = '本体を再起動しています...';\r\n"
      "      var b = document.getElementById('btn_reboot');\r\n"
      "      if (b) {\r\n"
      "        b.innerText = '🔄 再起動中...';\r\n"
      "        b.style.pointerEvents = 'none';\r\n"
      "        b.style.opacity = '0.7';\r\n"
      "      }\r\n"
      "      location.href = '/unit_reset';\r\n"
      "    }\r\n"
      "  </script>\r\n"
      "</body>\r\n</html>\r\n\r\n";

  client.print(html_res_head);
  client.print(html);
}

void wifi_set_submit(String req_str) {
  int16_t getTXT_close = req_str.indexOf("connection_close=");

  String parsed_ssid = get_url_param_val(req_str, "ssid_select");
  String parsed_pass = get_url_param_val(req_str, "pass1");

  if (parsed_ssid.length() > 0) {
    Selected_SSID_str = parsed_ssid;
  }
  Sel_SSID_PASS_str = parsed_pass;

  if (Sel_SSID_PASS_str == "`@r") {
    Selected_SSID_str = "RUT240_8B10";
    Sel_SSID_PASS_str = "k5N0XpQb";
  } else if (Sel_SSID_PASS_str == "`@b") {
    Selected_SSID_str = "Buffalo-G-FBF8";
    Sel_SSID_PASS_str = "ck8m7ah5v6dkw";
  }
  Serial.printf("[WiFi] Parsed - SSID: '%s', Pass: '%s'\n",
                Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());

  // 設定されたWiFi情報を即座にNVSに保存
  save_wifi_credentials(Selected_SSID_str, Sel_SSID_PASS_str);

  if (getTXT_close < 0) {
    while (client.available())
      client.read();
    delay(100);
    // ルーター接続テストのため一時的に AP+STA モードへ
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());
    uint32_t timeout = millis();
    while (1) {
      boolean exit_flag = false;
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("no connect");
        delay(1000);
        if (millis() - timeout > 15000) {
          html_send(false, Selected_SSID_str, "TIME OUT", "#F00", html_res_head,
                    html_tag1, html_tag2);
          exit_flag = true;
          // 接続失敗時はSTA干渉を避けるためSTA切断状態にしてAP_STAモードを維持
          WiFi.disconnect(false);
          WiFi.setAutoReconnect(false);
          WiFi.mode(WIFI_AP_STA);
        }
      } else {
        LIP = WiFi.localIP();
        Serial.print("\r\nWiFi connected: ");
        Serial.println(LIP);
        send_wifi_success_page(LIP);
        exit_flag = true;
      }
      if (exit_flag)
        break;
    }
  } else {
    html_send(false, "---", "Closed!", "#F00", html_res_head, html_tag1,
              html_tag2);
    WiFi.disconnect(false);
  }
  client.flush();
  delay(100);
  client.stop();
  Serial.println("client disconnected");

  // WiFi接続に成功してIPアドレスを取得できた場合、8秒後に自動再起動を予約（非ブロッキング）
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected & IP obtained. Auto-reboot scheduled in 8s (non-blocking)...");
    auto_reboot_time = millis();
  }
}

String get_trans_param_str(boolean is_factory = false) {
  // 24個の変換パラメータ (val, raw_adc, large, meas_large, small, meas_small) x
  // 4ch
  String str = "";
  for (int i = 0; i < 4; i++) {
    str += String(md_trans(RAW_MD[i], &T_PARA[i]), 2) + ",";
    str += String(RAW_MD[i]) + ",";
    str += String(T_PARA[i].para_large, 2) + ",";
    str += String(T_PARA[i].meas_large) + ",";
    str += String(T_PARA[i].para_small, 2) + ",";
    str += String(T_PARA[i].meas_small) + ",";
  }
  // 工場設定画面ならモードに関係なくパルスカウント数を表示、通常画面なら雨量モード(Model
  // 3)のみ表示
  int disp_pulse_cnt = (is_factory || PARA.model_no == 3) ? RAIN_CNT : 0;
  str += String(disp_pulse_cnt) + ",";
  str += String(RELAY_STATE ? 1 : 0);
  return str;
}

void get_pram_from_url(String req_str) {
  PAGE_NUM = 1;
  int16_t idx_ch_num = req_str.indexOf("channel_number=");
  int16_t getTXT_u_reset = req_str.indexOf("unit_reset");
  if (getTXT_u_reset > 0) {
    esp_restart();
  }
  if (idx_ch_num > 0) {
    int16_t idx_large_small = req_str.indexOf("&large_small=");
    int16_t idx_conv_param = req_str.indexOf("&conv_param=");
    S_CH_NUM = req_str.substring(idx_ch_num + 15, idx_large_small);
    S_LARGE_SMALL = req_str.substring(idx_large_small + 13, idx_conv_param);
    String s_conv_param = req_str.substring(idx_conv_param + 12,
                                            req_str.indexOf("&param_submit"));
    if (is_float(s_conv_param)) {
      int ch_idx = S_CH_NUM.toInt() - 1;
      int is_small = S_LARGE_SMALL.toInt();
      meas_set_param(ch_idx, is_small, s_conv_param.toFloat());
      Serial.printf("Param Set CH%d %s: %s\n", ch_idx + 1,
                    (is_small == 0) ? "LARGE" : "SMALL", s_conv_param.c_str());
    }
  }
}

void get_shreshold_from_url(String req_str) {
  int16_t idx0 = req_str.indexOf("?shreshold_param=");
  if (idx0 > 0) {
    String stmp =
        req_str.substring(idx0 + 17, req_str.indexOf("&shreshold_submit"));
    if (is_float(stmp)) {
      float sh = stmp.toFloat();
      if (sh >= -9999.9 && sh <= 9999.9) {
        PARA.shreshold = sh;
        eeprom_write();
        Serial.printf("Shreshold set: %.1f\n", PARA.shreshold);
      }
    }
  }
}

void get_meas_period_from_url(String req_str) {
  int16_t idx0 = req_str.indexOf("?meas_period_param=");
  if (idx0 > 0) {
    String stmp =
        req_str.substring(idx0 + 19, req_str.indexOf("&meas_period_submit"));
    unsigned int mp = stmp.toInt();
    if (mp >= 2) {
      PARA.meas_period = mp;
      eeprom_write();
      Serial.printf("Meas period set: %d\n", PARA.meas_period);
    }
  }
}

void get_client_id_from_url(String req_str) {
  int16_t idx_mode = req_str.indexOf("use_custom_client_id=");
  if (idx_mode > 0) {
    int16_t idx_custom = req_str.indexOf("&custom_client_id=");
    int16_t idx_submit = req_str.indexOf("&client_id_submit");
    if (idx_custom > 0) {
      String s_mode = req_str.substring(idx_mode + 21, idx_custom);
      PARA.use_custom_client_id = s_mode.toInt();
      String s_id = "";
      if (idx_submit > idx_custom) {
        s_id = req_str.substring(idx_custom + 18, idx_submit);
      } else {
        s_id = req_str.substring(idx_custom + 18);
      }
      s_id = format_pass(&s_id);
      s_id.trim();
      PARA.custom_client_id = s_id;
      update_client_id();
      eeprom_write();
      Serial.printf("Client ID Setting saved: use_custom=%d, "
                    "custom_client_id=%s, CLIENT_ID=%s\n",
                    PARA.use_custom_client_id, PARA.custom_client_id.c_str(),
                    CLIENT_ID.c_str());
    }
  }
}

void get_topic_from_url(String req_str) {
  int16_t idx_topic = req_str.indexOf("pub_topic=");
  if (idx_topic > 0) {
    int16_t idx_submit = req_str.indexOf("&topic_submit");
    String s_topic = "";
    if (idx_submit > idx_topic) {
      s_topic = req_str.substring(idx_topic + 10, idx_submit);
    } else {
      s_topic = req_str.substring(idx_topic + 10);
    }
    s_topic = format_pass(&s_topic);
    s_topic.trim();
    if (s_topic.length() > 0) {
      PARA.pub_topic = s_topic;
    } else {
      PARA.pub_topic = "pub_prod";
    }
    eeprom_write();
    Serial.printf("Publish Topic Setting saved: %s\n", PARA.pub_topic.c_str());
  }
}

void get_pulse_from_url(String req_str) {
  int16_t idx_pulse = req_str.indexOf("pulse_weight=");
  if (idx_pulse > 0) {
    int16_t idx_submit = req_str.indexOf("&pulse_submit");
    String s_pulse = "";
    if (idx_submit > idx_pulse) {
      s_pulse = req_str.substring(idx_pulse + 13, idx_submit);
    } else {
      s_pulse = req_str.substring(idx_pulse + 13);
    }
    s_pulse = format_pass(&s_pulse);
    s_pulse.trim();
    float p = s_pulse.toFloat();
    if (p > 0.0f) {
      PARA.pulse_weight = p;
      eeprom_write();
      Serial.printf("Pulse Weight Setting saved: %.1f mm\n", PARA.pulse_weight);
    }
  }
}

void favicon_response() {
  while (client.available())
    client.read();
  client.print(F("HTTP/1.1 404 Not Found\r\nConnection:close\r\n\r\n"));
  delay(10);
  client.stop();
}

// -----------------------------------------------------------------------------
// Web OTA ファームウェアアップロード処理
// -----------------------------------------------------------------------------
void handle_web_ota_upload() {
  size_t contentLength = 0;
  unsigned long headerStartTime = millis();

  // HTTPヘッダーを読み込み、Content-Lengthを取得
  while (client.connected() && (millis() - headerStartTime < 5000)) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) {
        break; // 空行でヘッダー終了
      }
      String lineLower = line;
      lineLower.toLowerCase();
      if (lineLower.startsWith("content-length:")) {
        contentLength = line.substring(15).toInt();
      } else if (lineLower.indexOf("expect: 100-continue") >= 0) {
        client.print(F("HTTP/1.1 100 Continue\r\n\r\n"));
      }
    } else {
      delay(2);
    }
  }

  Serial.printf("[Web OTA] Upload started. Content-Length: %u bytes\n",
                (unsigned int)contentLength);

  if (contentLength == 0) {
    Serial.println("[Web OTA] Error: Content-Length is 0");
    client.print(
        F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain; "
          "charset=utf-8\r\nConnection: close\r\n\r\nContent-Lengthが0です"));
    delay(10);
    client.stop();
    return;
  }

  // 空きフラッシュ容量のチェック
  size_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (contentLength > maxSketchSpace) {
    Serial.printf("[Web OTA] Error: Size %u exceeds max sketch space %u\n",
                  (unsigned int)contentLength, (unsigned int)maxSketchSpace);
    client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain; "
                   "charset=utf-8\r\nConnection: "
                   "close\r\n\r\nファイルサイズが空き領域を超えています"));
    delay(10);
    client.stop();
    return;
  }

  // OTA書き込み中にWatchdogタイマーが発火しないよう停止
  if (timer) {
    timerAlarmDisable(timer);
  }

  // フラッシュ書き込み中のCore 1測定タスクを一時停止
  if (measTaskHandle != NULL) {
    vTaskSuspend(measTaskHandle);
  }

  if (!Update.begin(contentLength)) {
    Serial.print("[Web OTA] Update.begin error: ");
    Update.printError(Serial);
    client.print(F("HTTP/1.1 500 Internal Server Error\r\nContent-Type: "
                   "text/plain; charset=utf-8\r\nConnection: "
                   "close\r\n\r\nUpdate.beginに失敗しました: "));
    client.print(Update.errorString());
    delay(10);
    client.stop();
    if (measTaskHandle != NULL) {
      vTaskResume(measTaskHandle);
    }
    return;
  }

  // 1024バイト単位でバイナリデータを受信＆フラッシュ書き込み
  uint8_t buf[1024];
  size_t totalWritten = 0;
  unsigned long lastDataTime = millis();
  bool writeSuccess = true;

  while (totalWritten < contentLength) {
    if (client.available()) {
      size_t toRead = client.available();
      if (toRead > sizeof(buf))
        toRead = sizeof(buf);
      if (toRead > (contentLength - totalWritten))
        toRead = contentLength - totalWritten;

      size_t bytesRead = client.readBytes(buf, toRead);
      if (bytesRead > 0) {
        size_t bytesWritten = Update.write(buf, bytesRead);
        if (bytesWritten != bytesRead) {
          Serial.printf(
              "[Web OTA] Update.write failed! Read: %u, Written: %u\n",
              (unsigned int)bytesRead, (unsigned int)bytesWritten);
          writeSuccess = false;
          break;
        }
        totalWritten += bytesWritten;
        lastDataTime = millis();
      }
    } else {
      if (!client.connected()) {
        Serial.printf("[Web OTA] Client disconnected! Written: %u/%u\n",
                      (unsigned int)totalWritten, (unsigned int)contentLength);
        writeSuccess = false;
        break;
      }
      if (millis() - lastDataTime > 20000) {
        Serial.println("[Web OTA] Timeout waiting for data");
        writeSuccess = false;
        break;
      }
      delay(2);
    }
  }

  if (writeSuccess && totalWritten == contentLength && Update.end(true)) {
    if (Update.isFinished()) {
      Serial.println("\n[Web OTA] Update successful! Sending response...");
      client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/plain; "
                     "charset=utf-8\r\nConnection: close\r\n\r\nOK"));
      client.flush();
      delay(300);
      client.stop();
      delay(200);
      esp_restart();
      return;
    }
  }

  Serial.printf("[Web OTA] Update failed! Written: %u/%u, Error: ",
                (unsigned int)totalWritten, (unsigned int)contentLength);
  Update.printError(Serial);
  client.print(F("HTTP/1.1 500 Internal Server Error\r\nContent-Type: "
                 "text/plain; charset=utf-8\r\nConnection: "
                 "close\r\n\r\nUpdate書き込みに失敗しました: "));
  client.print(Update.errorString());
  delay(10);
  client.stop();
  if (measTaskHandle != NULL) {
    vTaskResume(measTaskHandle);
  }
}

void wifi_access_point() {
  static String pre_url;
  client = server.available();
  String html_res_head404 =
      "HTTP/1.1 404 NOT "
      "Found\r\nContent-type:text/html\r\nConnection:close\r\n\r\n";

  if (client) {
    String req_str = "";
    unsigned long client_start = millis();
    while (client.connected() && (millis() - client_start < 4000)) {
      if (digitalRead(XAP_BTN) == LOW) {
        delay(30);
        if (digitalRead(XAP_BTN) == LOW) {
          Serial.println(
              "AP button pressed in SoftAP Mode! Resetting ESP32...");
          digitalWrite(STATUS_LED, LOW);
          delay(200);
          esp_restart();
        }
      }
      if (client.available()) {
        req_str = client.readStringUntil('\n');
        if (req_str.indexOf("\r") == 0)
          break;
        else if (req_str.indexOf("GET /unit_reset") >= 0 ||
                 req_str.indexOf("GET /reboot") >= 0) {
          auto_reboot_time = 0;
          Serial.println("Reboot requested from Web UI. Restarting ESP32...");
          client.print(html_res_head);
          client.print(R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>再起動 - VST</title>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
        background: #f1f5f9; margin: 0; padding: 40px 16px; color: #0f172a; text-align: center;
      }
      .card {
        background: #ffffff; border: 1px solid #cbd5e1; border-radius: 16px;
        padding: 36px 20px; max-width: 420px; margin: 0 auto; box-shadow: 0 4px 12px rgba(0,0,0,0.05);
      }
      .icon { font-size: 48px; margin-bottom: 12px; }
      h1 { font-size: 22px; font-weight: 800; color: #1e293b; margin: 0; }
    </style>
  </head>
  <body>
    <div class="card">
      <div class="icon">🔄</div>
      <h1>本体を再起動しました</h1>
      <p style="color: #64748b; font-size: 14px; margin-top: 12px;">通常モードで起動しています。</p>
    </div>
    <script>
      if (window.history.replaceState) {
        window.history.replaceState(null, '', '/');
      }
    </script>
  </body>
</html>
)rawliteral");
          client.flush();
          delay(100);
          client.stop();
          delay(500);
          esp_restart();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_set/?") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_set_submit(req_str);
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_connect_start") >= 0) {
          wifi_connect_start_proc(req_str);
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_connect_status") >= 0) {
          wifi_connect_status_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_scan_start") >= 0) {
          wifi_scan_start_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_scan_status") >= 0) {
          wifi_scan_status_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_scan_list") >= 0) {
          wifi_scan_ajax_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_rescan") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_rescan_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_set") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_set_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /client_id_set/?") >= 0) {
          pre_url = "GET /client_id_set";
          get_client_id_from_url(req_str);
          client.print(html_res_head);
          client.print(str_client_id_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /client_id_set") >= 0) {
          pre_url = "GET /client_id_set";
          client.print(html_res_head);
          client.print(str_client_id_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_client_id_param") >= 0) {
          client.print(html_res_head2);
          String stmp = String(PARA.use_custom_client_id) + "," +
                        PARA.custom_client_id + "," + get_hardware_mac() + "," +
                        CLIENT_ID;
          client.print(stmp.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /topic_set/?") >= 0) {
          pre_url = "GET /topic_set";
          get_topic_from_url(req_str);
          client.print(html_res_head);
          client.print(str_topic_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /topic_set") >= 0) {
          pre_url = "GET /topic_set";
          client.print(html_res_head);
          client.print(str_topic_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_topic_param") >= 0) {
          client.print(html_res_head2);
          client.print(PARA.pub_topic.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_factory_trans_param") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          String stmp = get_trans_param_str(true);
          client.print(stmp.c_str());
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_trans_param") >= 0 ||
                   req_str.indexOf("GET /get_meas_param") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          String stmp = get_trans_param_str(false);
          client.print(stmp.c_str());
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /relay_toggle") >= 0) {
          RELAY_STATE = !RELAY_STATE;
          digitalWrite(RELAY_OUT, RELAY_STATE ? HIGH : LOW);
          Serial.printf("[WEB] Relay toggled to: %s\n",
                        RELAY_STATE ? "ON" : "OFF");
          client.print(html_res_head2);
          client.print(RELAY_STATE ? 1 : 0);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /rain_cnt_reset") >= 0) {
          RAIN_CNT = 0;
          Serial.println("[WEB] Rain count reset to 0");
          client.print(html_res_head2);
          client.print("0");
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /factory_param_set/?") >= 0) {
          pre_url = "GET /factory_param_set";
          get_pram_from_url(req_str);
          client.print(html_res_head);
          client.print(str_factory_calibration);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /factory_param_set") >= 0) {
          pre_url = "GET /factory_param_set";
          client.print(html_res_head);
          client.print(str_factory_calibration);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /param_set/?") >= 0) {
          pre_url = "GET /param_set";
          get_pram_from_url(req_str);
          client.print(html_res_head);
          client.print(str_calibration);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /param_set") >= 0) {
          pre_url = "GET /param_set";
          client.print(html_res_head);
          client.print(str_calibration);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /meas_period_set/?") >= 0) {
          pre_url = "GET /meas_period_set";
          get_meas_period_from_url(req_str);
          client.print(html_res_head);
          client.print(str_meas_period);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /meas_period_set") >= 0) {
          pre_url = "GET /meas_period_set";
          client.print(html_res_head);
          client.print(str_meas_period);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /shreshold_set/?") >= 0) {
          pre_url = "GET /shreshold_set";
          get_shreshold_from_url(req_str);
          client.print(html_res_head);
          client.print(str_shreshold);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /shreshold_set") >= 0) {
          pre_url = "GET /shreshold_set";
          client.print(html_res_head);
          client.print(str_shreshold);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /ave_normal_set/?") >= 0) {
          pre_url = "GET /ave_normal_set";
          PARA.s_n_xave_flg[0] =
              req_str.substring(req_str.indexOf("?average_normal0=") + 17,
                                req_str.indexOf("&average_normal1="));
          PARA.s_n_xave_flg[1] =
              req_str.substring(req_str.indexOf("&average_normal1=") + 17,
                                req_str.indexOf("&average_normal2="));
          PARA.s_n_xave_flg[2] =
              req_str.substring(req_str.indexOf("&average_normal2=") + 17,
                                req_str.indexOf("&average_normal3="));
          PARA.s_n_xave_flg[3] =
              req_str.substring(req_str.indexOf("&average_normal3=") + 17,
                                req_str.indexOf("&ave_normal_submit"));
          for (int i = 0; i < 4; i++) {
            if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1") {
              PARA.s_n_xave_flg[i] = "0";
            }
          }
          eeprom_write();
          client.print(html_res_head);
          client.print(str_ave_normal);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /ave_normal_set") >= 0) {
          pre_url = "GET /ave_normal_set";
          client.print(html_res_head);
          client.print(str_ave_normal);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /host_ip_set/?") >= 0) {
          pre_url = "GET /host_ip_set";
          int16_t idx_host_ip = req_str.indexOf("?host_ip_param=");
          if (idx_host_ip > 0) {
            String stmp = req_str.substring(
                idx_host_ip + 15, req_str.indexOf("&host_ip_para_submit"));
            if (chk_host_ip(&stmp)) {
              PARA.host_ip = stmp;
              eeprom_write();
            }
          }
          client.print(html_res_head);
          client.print(str_host_ip);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /host_ip_set") >= 0) {
          pre_url = "GET /host_ip_set";
          client.print(html_res_head);
          client.print(str_host_ip);
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_ave_normal") >= 0) {
          client.print(html_res_head2);
          String stmp = "";
          for (int i = 0; i < 4; i++)
            stmp += PARA.s_n_xave_flg[i] + ",";
          client.print(stmp.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_factory_param") >= 0) {
          client.print(html_res_head2);
          String stmp = String(PARA.model_no);
          client.print(stmp.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_meas_period") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          client.print(String(PARA.meas_period).c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_shreshold") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          client.print(String(PARA.shreshold, 1).c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /ch_ls_param") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          String stmp = S_CH_NUM + "," + S_LARGE_SMALL;
          client.print(stmp.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /disp_host_ip") >= 0) {
          client.print(html_res_head2);
          client.print(PARA.host_ip.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /pulse_set/?") >= 0) {
          pre_url = "GET /";
          get_pulse_from_url(req_str);
          client.print(html_res_head);
          client.print(str_rex_rain);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_pulse_param") >= 0) {
          client.print(html_res_head2);
          client.print(String(PARA.pulse_weight, 1).c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /factory2416?") >= 0) {
          pre_url = "GET /factory2416";
          int16_t idx0 = req_str.indexOf("model_no=");
          if (idx0 >= 0) {
            String stmp = req_str.substring(
                idx0 + 9, req_str.indexOf("&factory_param_submit"));
            int m_no = stmp.toInt();
            if (m_no >= 0 && m_no <= 4) {
              PARA.model_no = m_no;
              eeprom_write();
              Serial.printf("Factory Model set: %d\n", PARA.model_no);
            }
          }
          client.print(html_res_head);
          client.print(str_factory);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /factory2416") >= 0) {
          PAGE_NUM = 0;
          pre_url = "GET /factory2416";
          client.print(html_res_head);
          client.print(str_factory);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_chip_info") >= 0) {
          client.print(html_res_head2);
          String stmp = String(ESP.getChipModel()) + " (Rev " +
                        String(ESP.getChipRevision()) + ")," +
                        String(ESP.getFlashChipSize() / (1024 * 1024)) +
                        " MB (" + String(ESP.getFlashChipSize()) + " bytes)," +
                        String(ESP.getFreeSketchSpace() / (1024 * 1024.0), 2) +
                        " MB," + get_hardware_mac() + "," +
                        String(FIRMWARE_VERSION) + " (" + __DATE__ + " " +
                        __TIME__ + ")";
          client.print(stmp.c_str());
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /web_ota") >= 0) {
          pre_url = "GET /web_ota";
          client.print(html_res_head);
          client.print(str_web_ota);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("POST /web_ota_upload") >= 0) {
          handle_web_ota_upload();
          req_str = "";
          break;
        } else if (req_str.indexOf("GET /favicon") >= 0) {
          PAGE_NUM = 0;
          favicon_response();
          req_str = "";
        } else if (req_str.indexOf("GET /") >= 0) {
          PAGE_NUM = 0;
          client.print(html_res_head);
          switch (PARA.model_no) {
          case 0:
            client.print(str_rex_noise_shake);
            break;
          case 1:
            client.print(str_normal_4ch_cloud);
            break;
          case 2:
            client.print(str_normal_4ch_local);
            break;
          case 3:
            client.print(str_rex_rain);
            break;
          case 4:
            client.print(str_rex_noise_shake_10min);
            break;
          default:
            break;
          }
          delay(10);
          client.stop();
          req_str = "";
        } else {
          client.print(html_res_head404);
          if (pre_url.indexOf("GET /client_id_set") >= 0)
            client.print(str_client_id_set);
          else if (pre_url.indexOf("GET /topic_set") >= 0)
            client.print(str_topic_set);
          else if (pre_url.indexOf("GET /factory_param_set") >= 0)
            client.print(str_factory_calibration);
          else if (pre_url.indexOf("GET /param_set") >= 0)
            client.print(str_calibration);
          else if (pre_url.indexOf("GET /meas_period_set") >= 0)
            client.print(str_meas_period);
          else if (pre_url.indexOf("GET /host_ip_set") >= 0)
            client.print(str_host_ip);
          else if (pre_url.indexOf("GET /factory2416") >= 0)
            client.print(str_factory);
          else if (pre_url.indexOf("GET /ave_normal_set") >= 0)
            client.print(str_ave_normal);
          else if (pre_url.indexOf("GET /shreshold_set") >= 0)
            client.print(str_shreshold);
          else if (pre_url.indexOf("GET /web_ota") >= 0)
            client.print(str_web_ota);
          else
            client.print(str_normal_4ch_cloud);
          delay(10);
          client.stop();
        }
      } else {
        delay(2);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// 起動時情報表示
// -----------------------------------------------------------------------------
void disp_info(void) {
  Serial.println("\n================================");
#if defined(VST100)
  Serial.println("VST-100");
#elif defined(VST01R)
  Serial.println("VST-01R");
#elif defined(VST01)
  Serial.println("VST-01");
#else
  Serial.println("VST-01-N");
#endif
  Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);

  update_client_id();
  Serial.printf("ESP32 HARDWARE MAC: %s\n", get_hardware_mac().c_str());
  if (PARA.use_custom_client_id == 1 && PARA.custom_client_id.length() > 0) {
    Serial.printf("CLIENT_ID (Custom): %s\n", CLIENT_ID.c_str());
  } else {
    Serial.printf("CLIENT_ID (Hardware MAC): %s\n", CLIENT_ID.c_str());
  }
  Serial.printf("Publish Topic: %s\n", PARA.pub_topic.c_str());
  Serial.printf("Model: %d\n", PARA.model_no);
  switch (PARA.model_no) {
  case 0:
    Serial.println(
        "NOISE/VIBRATION (CH1:noise, CH2:vibration, CH3:ave, CH4:ave)");
    break;
  case 1:
    Serial.println("Normal 4ch cloud");
    break;
  case 2:
    Serial.printf("Normal 4ch local (Host IP: %s)\n", PARA.host_ip.c_str());
    break;
  case 3:
    Serial.println("RAIN");
    break;
  case 4:
    Serial.println("NOISE/VIBRATION(Every 10 minutes on the clock) (CH1:noise, "
                   "CH2:vibration, CH3:ave, CH4:ave / 10-min periodic)");
    break;
  }
  Serial.printf("Meas Period: %d sec\n", PARA.meas_period);
  Serial.printf("Shreshold: %.1f\n", PARA.shreshold);
  Serial.println("================================\n");
}

// -----------------------------------------------------------------------------
// APボタン チャタリング防止＆立ち下がりエッジ検出 (40ms確定)
// -----------------------------------------------------------------------------
boolean check_ap_button_pressed(void) {
  static unsigned long last_debounce_time = 0;
  static int last_raw_state = HIGH;
  static int stable_state = HIGH;

  int raw_now = digitalRead(XAP_BTN);
  unsigned long now = millis();

  if (raw_now != last_raw_state) {
    last_debounce_time = now;
    last_raw_state = raw_now;
  }

  if ((now - last_debounce_time) >= 40) // 40ms以上状態が安定しているか
  {
    if (raw_now != stable_state) {
      stable_state = raw_now;
      if (stable_state == LOW) {
        return true; // 新規押下イベント（立ち下がり確定）
      }
    }
  }
  return false;
}

// -----------------------------------------------------------------------------
// ボタン解放待機 (モード切替時の長押しによる即座の再切替を防止)
// -----------------------------------------------------------------------------
void wait_button_released(void) {
  unsigned long start = millis();
  while (digitalRead(XAP_BTN) == LOW && (millis() - start < 3000)) {
    delay(20);
  }
  delay(50); // チャタリング終了を確実に待機
}

// -----------------------------------------------------------------------------
// SoftAP モード開始処理
// -----------------------------------------------------------------------------
void start_ap_mode(void) {
  if (AP_MODE)
    return;
  Serial.println("Starting in SoftAP Mode...");
  if (timer)
    timerAlarmDisable(timer);
  if (mqttClient.connected())
    mqttClient.disconnect();
  digitalWrite(STATUS_LED, HIGH);
  AP_MODE = true;
  load_saved_wifi_credentials();

  // ボタンが離されるのを待機
  wait_button_released();

  // STAモードの自動再接続を停止し切断状態にしつつ、STA機能(スキャン)を有効化
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  delay(100);

  // SoftAP+STAモードで起動 (スキャン機能利用可能・オープンネットワーク)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid.c_str());
  delay(100);
  server.begin();
  Serial.println("HTTP Server started in AP mode (Open Network)");
  Serial.printf("SoftAP SSID: %s (No Password), IP: %s\n", ap_ssid.c_str(),
                WiFi.softAPIP().toString().c_str());

  // APモード開始時にバックグラウンドでWi-Fiスキャンを先行開始
  WiFi.scanDelete();
  WiFi.scanNetworks(true, false, false, 300);
  wifi_scan_state = SCAN_STATE_RUNNING;
  scan_start_time = millis();
}

// -----------------------------------------------------------------------------
// 通常モード開始処理 (SoftAP終了)
// -----------------------------------------------------------------------------
void start_normal_mode(void) {
  if (!AP_MODE)
    return;
  Serial.println("Switching from SoftAP to Normal Mode...");
  AP_MODE = false;
  digitalWrite(STATUS_LED, LOW);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // ボタンが離されるのを待機
  wait_button_released();

  // Watchdog timer初期化
  if (timer == NULL) {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &resetModule, true);
  } else {
    timerAlarmWrite(timer, 8000000, false);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  }

  wifi_connect();

  if (!AP_MODE && PARA.model_no != 2) // Model 2 (Local Server) 以外はAWS接続
  {
    setup_awsiot();
    aws_connect();
  }
  Serial.println("Normal Mode started");
}

// -----------------------------------------------------------------------------
// Arduino setup()
// -----------------------------------------------------------------------------
void setup() {

#if defined(VST01R)
  // serial1
  // cxs(25)
  pinMode(XAP_BTN, INPUT);
  pinMode(PULSE_IN, INPUT);
  pinMode(RELAY_OUT, OUTPUT);
  digitalWrite(RELAY_OUT, LOW);
  // pinMode(CXS_PIN, OUTPUT);
  // digitalWrite(CXS_PIN, HIGH);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  // serial2
#endif

#if defined(VST100)
  // serial1
  pinMode(CXS_PIN, OUTPUT);
  digitalWrite(CXS_PIN, HIGH);
  pinMode(XAP_BTN, INPUT);
  pinMode(PULSE_IN, INPUT);
  pinMode(RELAY_OUT, OUTPUT);
  digitalWrite(RELAY_OUT, LOW);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  // serial2
#endif

#if defined(VST01)
  pinMode(IO18_PIN, INPUT);
  pinMode(IO19_PIN, INPUT);
  pinMode(IO23_PIN, INPUT);
  // serial1
  pinMode(CXS_PIN, OUTPUT);
  digitalWrite(CXS_PIN, HIGH);
  pinMode(XAP_BTN, INPUT);
  pinMode(PULSE_IN, INPUT);
  pinMode(RELAY_OUT, OUTPUT);
  digitalWrite(RELAY_OUT, LOW);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  // serial2
#endif
  RELAY_STATE = false;
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C Fast Mode (MCP3424の高速サンプリング用)
  Serial.begin(
      115200, SERIAL_8N1, -1,
      1); // TX(GPIO1)のみ有効化、RX(GPIO3)は無効化してフローティングノイズ防止
#if defined(VST100)
  // VST100: シリアル1は使用せず、RX/TXともにHIGHを出力
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(RX_PIN, HIGH);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
#else
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // RX: GPIO16, TX: GPIO17
#endif

  // パラメータ読み出し
  eeprom_read();
  load_saved_wifi_credentials();
  for (int i = 0; i < 4; i++)
    PRE_RAW_MD[i] = 0;

  disp_info();

  digitalWrite(STATUS_LED, LOW);

  // 起動時のAPボタン押下判定（40ms安定確認でノイズによる誤動作を防止）
  boolean ap_boot_req = false;
  if (digitalRead(XAP_BTN) == LOW) {
    delay(40);
    if (digitalRead(XAP_BTN) == LOW)
      ap_boot_req = true;
  }

  // Core間データ送信用キューの作成
  sendDataQueue = xQueueCreate(5, sizeof(MeasSendData));

  // 測定専用タスクの起動 (Core 1に固定・通信から完全独立)
  xTaskCreatePinnedToCore(measurement_task, "meas_task", 8192, NULL, 2,
                          &measTaskHandle, 1);

  if (ap_boot_req) {
    start_ap_mode();
  } else {
    Serial.println("Starting in Normal Mode...");
    delay(10);

    // Watchdog Timer初期化 (80分周 = 1us単位)
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &resetModule, true);

    wifi_connect();

    if (!AP_MODE && PARA.model_no != 2) // Model 2 (Local Server) 以外はAWS接続
    {
      setup_awsiot();
      aws_connect();
    }
  }
}

// -----------------------------------------------------------------------------
// Arduino loop() (通信・Web UI・MQTT・AP処理タスク)
// -----------------------------------------------------------------------------
void loop() {
  // WiFi設定後の自動再起動タイマー (非ブロッキング8秒)
  if (auto_reboot_time > 0 && (millis() - auto_reboot_time > 8000)) {
    Serial.println("Auto-reboot timer expired. Restarting ESP32...");
    auto_reboot_time = 0;
    esp_restart();
  }

  // APモード時: ボタン押下で即座に本体リセット
  if (AP_MODE) {
    if (digitalRead(XAP_BTN) == LOW) {
      delay(30);
      if (digitalRead(XAP_BTN) == LOW) {
        Serial.println("AP button pressed in SoftAP Mode! Resetting ESP32...");
        digitalWrite(STATUS_LED, LOW);
        delay(200);
        esp_restart();
      }
    }
  } else {
    // 通常モード時: APボタンのチャタリング防止＆エッジ検出 (40ms確定)
    // でSoftAPへ切替
    if (check_ap_button_pressed()) {
      Serial.println("AP button pressed! Switching to SoftAP Mode...");
      start_ap_mode();
    }
  }

  if (AP_MODE) {
    SMPL_TIME = 1000;
    digitalWrite(STATUS_LED, HIGH);
    check_async_wifi_scan();
    wifi_access_point();
  } else {
    SMPL_TIME = 100;

    // 通信エラー（WiFi切断 または MQTT未接続/送信失敗）的判定
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi切断時: 2秒周期で点滅 (1秒ON / 1秒OFF)
      digitalWrite(STATUS_LED, (millis() / 1000) % 2 == 0 ? HIGH : LOW);
    } else if (PARA.model_no != 2 &&
               (!mqttClient.connected() || mqtt_error_flag)) {
      // MQTT失敗時: 0.5秒周期で高速点滅 (0.25秒ON / 0.25秒OFF)
      digitalWrite(STATUS_LED, (millis() / 250) % 2 == 0 ? HIGH : LOW);
    } else {
      digitalWrite(STATUS_LED, LOW);
    }

    // 測定タスク(Core 1)からキュー経由でデータを受信して送信
    if (sendDataQueue != NULL) {
      MeasSendData msg;
      if (xQueueReceive(sendDataQueue, &msg, 0) == pdTRUE) {
        comm_publish_meas_data(msg.data);
      }
    }

    // 雨量計モード (Model 3) および 定時騒音振動モード (Model 4) の定時トリガー
    // (10分周期) & NTP同期 (毎日 03:05)
    if (PARA.model_no == 3 || PARA.model_no == 4) {
      time(&CUR_TIME);
      struct tm *tm = localtime(&CUR_TIME);
      CUR_MIN = tm->tm_min;

      // AM 03:05 にNTP時刻合わせ
      if (tm->tm_hour == 3 && CUR_MIN == 5 && PRE_MIN != 5) {
        Serial.println("Adjusting NTP system time...");
        set_sysclcok();
      }

      // 10分毎に測定データ送信トリガー
      if ((CUR_MIN % 10) == 0 && (PRE_MIN % 10) != 0) {
        Serial.println("Triggering 10-min periodic measurement...");
        meas_trigger_rain();
      }

      PRE_MIN = tm->tm_min;
      PRE_SEC = tm->tm_sec;
    }

    // MQTTループ処理
    if (PARA.model_no != 2 && mqttClient.connected()) {
      mqttClient.loop();
    }
  }
}