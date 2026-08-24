// =============================================================================
// VST-01 / VST-100 統合ファームウェア (1-Chip / 1-CPU版)
// 測定(MCP3424 / 雨量計 / 統計計算) と 通信(WiFi / SoftAP / AWS IoT MQTT)
// の統合
// =============================================================================

#define VST100        // VST-100なら定義、VST-01ならコメントアウト
#define CLOUD_DEBUG 0 // クラウドデバッグ用 通常動作時は0をセット
// 0:通常動作
// 1:RFT-01クラウドデバッグ用  ch1のデータをCH4で代用
// 2:VST-01 騒音振動番チェック すべてのデータをCH4で代用
// 3:ノーマル4chチェック       すべてのデータをCH4で代用
// 4:VST-100 雨量計版チェック  すべてのデータをCH4で代用

#include "aws.h" // AWS証明書
#include "esp_sntp.h"
#include "esp_system.h"
#include "time.h"
#include <EEPROM.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
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

#define JST (3600 * 9)

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
  int use_custom_mac;       // 0: ESP32 Hardware MAC, 1: Custom Specified MAC
  String custom_mac;        // 指定したMACアドレス
  String pub_topic;         // MQTT Publish Topic ("pub_prod", "pub01" 等)
};

// 統合EEPROM保存用構造体 (固定長バイナリ)
struct UnifiedEepromSettings {
  char magic[8]; // "VST_U03"
  int model_no;
  char s_n_xave_flg[4][4];
  char host_ip[32];
  float shreshold;
  unsigned int meas_period;
  trans_para t_para[4];
  int use_custom_mac;
  char custom_mac[32];
  char pub_topic[32];
};

// -----------------------------------------------------------------------------
// グローバル変数
// -----------------------------------------------------------------------------
para_d PARA;
trans_para T_PARA[4];

// 通信・Web関連変数
const char *pubTopic = "pub_prod"; // デフォルト製品版 ("pub01" はクラウドデバッグ用)
const char ntp_server[][30] = {"ntp.nict.jp", "pool.ntp.org",
                               "ntp.jst.mfeed.ad.jp"};
long CUR_TIME;
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

String S_CH_NUM = "1";      // Calibration画面用
String S_LARGE_SMALL = "0"; // 0:large 1:small

// AWS IoT
const char *awsEndpoint = "a24t2172v8g5ia-ats.iot.ap-northeast-1.amazonaws.com";
const int awsPort = 8883;
WiFiClientSecure httpsClient;
PubSubClient mqttClient(httpsClient);

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
unsigned int md_max[4], md_min[4];
unsigned long md_sum[4];
unsigned int SORT_DATA[2][6000]; // 騒音・振動パーセンタイル計算用ソートバッファ
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
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Calibration</h1>
    <p style='color:brown; font-weight: bold'>CONVERTED DATA / PARAMETER(LARGE/SMALL)</p>
    <p><table>
      <tr><th>CHANNEL</th><th>DATA</th><th>LARGE</th><th>SMALL</th></tr>
      <tr><td>CH1</td><td><span id="val_ch1" class="value"></span></td><td><span id="pl_ch1" class="value"></span></td><td><span id="ps_ch1" class="value"></span></td></tr>
      <tr><td>CH2</td><td><span id="val_ch2" class="value"></span></td><td><span id="pl_ch2" class="value"></span></td><td><span id="ps_ch2" class="value"></span></td></tr>
      <tr><td>CH3</td><td><span id="val_ch3" class="value"></span></td><td><span id="pl_ch3" class="value"></span></td><td><span id="ps_ch3" class="value"></span></td></tr>
      <tr><td>CH4</td><td><span id="val_ch4" class="value"></span></td><td><span id="pl_ch4" class="value"></span></td><td><span id="ps_ch4" class="value"></span></td></tr>
    </table></p>
    <p style='color:brown; font-weight: bold'>Scaling Parameter Set</p>
    <form name='paremeter_set'>
      <p><table>
        <tr><th style='width: 30px'>CH</th><th>LARGE/SMALL</th><th>PARAMETER</th><th style='border-top-style:none'></th></tr>
        <tr><td style='width: 30px'>
          <select name="channel_number">
          <option value="1">CH1</option>
          <option value="2">CH2</option>
          <option value="3">CH3</option>
          <option value="4">CH4</option>
          </select>
        </td>
        <td style='width: 30px'>
          <select name="large_small">
          <option value="0">LARGE</option>
          <option value="1">SMALL</option>
          </select>
        </td>
        <td><input type='text' name='conv_param'></td><td><button type='submit' name='param_submit' id='button1' value='send' style='background-color:#AFA;'>Set</button></td></tr>
      </table></p>
    </form>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
  </body>
  <script>
    var disp_trans_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let val = this.responseText.split(',');
          document.getElementById("val_ch1").innerHTML = val[0];
          document.getElementById("pl_ch1").innerHTML = val[1];
          document.getElementById("ps_ch1").innerHTML = val[2];
          document.getElementById("val_ch2").innerHTML = val[3];
          document.getElementById("pl_ch2").innerHTML = val[4];
          document.getElementById("ps_ch2").innerHTML = val[5];
          document.getElementById("val_ch3").innerHTML = val[6];
          document.getElementById("pl_ch3").innerHTML = val[7];
          document.getElementById("ps_ch3").innerHTML = val[8];
          document.getElementById("val_ch4").innerHTML = val[9];
          document.getElementById("pl_ch4").innerHTML = val[10];
          document.getElementById("ps_ch4").innerHTML = val[11];
        }
      };
      xhr.open("GET", "/disp_trans_param", true);
      xhr.send(null);
    }
    var ch_ls_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            let cmd = this.responseText.split(',');
            let elements = document.getElementsByName('channel_number');
            elements[0].options[Number(cmd[0])-1].selected = true;
            elements = document.getElementsByName('large_small');
            elements[0].options[Number(cmd[1])].selected = true;
          }
      };
      xhr.open("GET", "/ch_ls_param", true);
      xhr.send(null);
    }
    setInterval(disp_trans_param, 1000);
    window.onload = ch_ls_param;
  </script>
</html>)rawliteral";

const char *str_mac_set = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
      input[type=text] { font-size: 16px; padding: 6px; }
      button { font-size: 16px; padding: 8px 24px; cursor: pointer; }
    </style>
  </head>
  <body>
    <h1>MAC Address Setting</h1>
    <p><table>
      <tr><th>Current CLIENT_ID</th><th>ESP32 Hardware MAC</th></tr>
      <tr><td><span id="current_client_id" class="value"></span></td><td><span id="hw_mac" class="value"></span></td></tr>
    </table></p>
    <form action='/mac_set/' method='GET'>
      <p style='margin: 15px 0; font-size: 16px;'>
        <label><input type="radio" name="use_custom_mac" value="0" id="mac_opt_hw"> ESP32 MACアドレスを使用 (Auto)</label><br><br>
        <label><input type="radio" name="use_custom_mac" value="1" id="mac_opt_custom"> 指定したMACアドレスを使用 (Custom)</label>
      </p>
      <p>
        <label>Custom MAC: </label>
        <input type='text' name='custom_mac' id='custom_mac_input' value='' placeholder='e.g. 24-0a-c4-xx-xx-xx'>
      </p>
      <button type='submit' name='mac_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br><br>
    <a href='/f1c9t' style='color:navy; font-size:20px;'>Factory Home</a>
  </body>
  <script>
    var disp_mac_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let val = this.responseText.split(',');
          if (val[0] === "1") {
            document.getElementById("mac_opt_custom").checked = true;
          } else {
            document.getElementById("mac_opt_hw").checked = true;
          }
          document.getElementById("custom_mac_input").value = val[1] || "";
          document.getElementById("hw_mac").innerHTML = val[2] || "";
          document.getElementById("current_client_id").innerHTML = val[3] || "";
        }
      };
      xhr.open("GET", "/disp_mac_param", true);
      xhr.send(null);
    }
    window.onload = disp_mac_param;
  </script>
</html>)rawliteral";

const char *str_topic_set = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
      input[type=text] { font-size: 16px; padding: 6px; }
      button { font-size: 16px; padding: 8px 24px; cursor: pointer; }
    </style>
  </head>
  <body>
    <h1>Publish Topic Setting</h1>
    <p><table>
      <tr><th>Current Publish Topic</th></tr>
      <tr><td><span id="current_topic" class="value"></span></td></tr>
    </table></p>
    <form action='/topic_set/' method='GET'>
      <p style='margin: 15px 0; font-size: 16px; text-align: left; display: inline-block;'>
        <label><input type="radio" name="topic_preset" value="pub_prod" id="topic_opt_prod" onclick="document.getElementById('custom_topic_input').value='pub_prod'"> 製品版 (pub_prod) [デフォルト]</label><br><br>
        <label><input type="radio" name="topic_preset" value="pub01" id="topic_opt_debug" onclick="document.getElementById('custom_topic_input').value='pub01'"> クラウドデバッグ用 (pub01)</label><br><br>
        <label><input type="radio" name="topic_preset" value="custom" id="topic_opt_custom"> カスタム指定</label>
      </p>
      <p>
        <label>Topic: </label>
        <input type='text' name='pub_topic' id='custom_topic_input' value='' placeholder='e.g. pub_prod'>
      </p>
      <button type='submit' name='topic_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br><br>
    <a href='/f1c9t' style='color:navy; font-size:20px;'>Factory Home</a>
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
  </script>
</html>)rawliteral";

const char *str_factory = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Factory</h1>
    <form>
      <label for="model_no">Select Model</label>
      <select name="model_no">
        <option value="0">NOISE/VIBRATION</option>
        <option value="1">Normal 4ch cloud</option>
        <option value="2">Normal 4ch local</option>
        <option value="3">RAIN</option>
        <option value="4">NOISE/VIBRATION(Every 10 minutes on the clock)</option>
      </select>
      <button type='submit' name='factory_param_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br><br><br><br>
    <a href='/mac_set/' style='color:navy; font-size:20px;'>MAC Address Setting</a><br><br>
    <a href='/topic_set/' style='color:navy; font-size:20px;'>Publish Topic Setting</a><br><br>
    <a href='/meas_period_set/' style='color:navy; font-size:20px;'>Measurement Period</a><br><br>
    <a href='/ave_normal_set/' style='color:navy; font-size:20px;'>Average / Normal Setting</a><br><br>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a><br><br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
  </body>
  <script>
    var factory_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let no = this.responseText;
          let elements = document.getElementsByName('model_no');
          elements[0].options[Number(no)].selected = true;
        }
      }
      xhr.open("GET", "/disp_factory_param", true);
      xhr.send(null);
    }
    window.onload = factory_param;
  </script>
</html>)rawliteral";

const char *str_rex_noise_shake = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>ch1:noise ch2:vibration</h1>
    <h1>ch3:average ch4:average</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
  </body>
</html>)rawliteral";

const char *str_rex_noise_shake_10min = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>正時基準10分周期</h1>
    <h1>ch1:noise ch2:vibration</h1>
    <h1>ch3:average ch4:average</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
  </body>
</html>)rawliteral";

const char *str_rex_rain = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>ch1:rain ch2-4:average</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a><br><br>
    <a href='/shreshold_set/' style='color:navy; font-size:20px;'>Shreshold</a>
  </body>
</html>)rawliteral";

const char *str_normal_4ch_cloud = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>4CH NORMAL CLOUD</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a><br><br>
    <a href='/ave_normal_set/' style='color:navy; font-size:20px;'>Average / Normal Setting</a>
  </body>
</html>)rawliteral";

const char *str_normal_4ch_local = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>4CH NORMAL LOCAL</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a><br><br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a><br><br>
    <a href='/meas_period_set/' style='color:navy; font-size:20px;'>Measurement Period Setting</a><br><br>
    <a href='/host_ip_set/' style='color:navy; font-size:20px;'>Server IP</a>
  </body>
</html>)rawliteral";

const char *str_host_ip = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Server IP Setting</h1>
    <form>
      <label>Server IP</label>
      <input type='text' name='host_ip_param' id='host_ip_param1' value="">
      <br><br>
      <button type='submit' name='host_ip_para_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br><br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
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
  </script>
</html>)rawliteral";

const char *str_meas_period = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Measurement Period Setting</h1>
    <form>
      <p><table>
        <tr><th>Measurement Period(sec)</th></tr>
        <tr><td><span id="meas_period_val" class="value"></span></td></tr>
      </table></p>
      <label>input:</label><input type='text' name='meas_period_param' value=""><label>(60 - 3600)</label>
      <br><br>
      <button type='submit' name='meas_period_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br>
    <a href='/f1c9t' style='color:navy; font-size:20px;'>Factory Home</a>
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
  </script>
</html>)rawliteral";

const char *str_shreshold = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Shreshold Setting</h1>
    <form>
      <p><table>
        <tr><th>Shreshold</th></tr>
        <tr><td><span id="shreshold_val" class="value"></span></td></tr>
      </table></p>
      <label>input:</label><input type='text' name='shreshold_param' value=""><label>(0 - 9999.9)</label>
      <br><br>
      <button type='submit' name='shreshold_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
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
  </script>
</html>)rawliteral";

const char *str_ave_normal = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto;text-align: center;} 
      h1 {font-size:28px;}
      body {text-align: center;} 
      table { border-collapse: collapse; margin-left:auto; margin-right:auto;}
      th { padding: 12px; background-color: #0000cd; color: white; border: solid 2px #c0c0c0;}
      tr { border: solid 2px #c0c0c0; padding: 12px;}
      td { border: solid 2px #c0c0c0; padding: 12px;}
      .value { color:blue; font-weight: bold; padding: 1px;}
    </style>
  </head>
  <body>
    <h1>Average / Normal Setting</h1>
    <p style='color:brown; font-weight: bold'>Measurement Period</p>
    <form>
      <p><table>
        <tr><th>CH</th><th>AVERAGE / NORMAL</th></tr>
        <tr><td>1</td><td><input type="radio" name="average_normal0" value="0">Average<input type="radio" name="average_normal0" value="1">Normal</td></tr>
        <tr><td>2</td><td><input type="radio" name="average_normal1" value="0">Average<input type="radio" name="average_normal1" value="1">Normal</td></tr>
        <tr><td>3</td><td><input type="radio" name="average_normal2" value="0">Average<input type="radio" name="average_normal2" value="1">Normal</td></tr>
        <tr><td>4</td><td><input type="radio" name="average_normal3" value="0">Average<input type="radio" name="average_normal3" value="1">Normal</td></tr>
      </table></p>
      <button type='submit' name='ave_normal_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br>
    <a href='/f1c9t' style='color:navy; font-size:20px;'>Factory Home</a>
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
            elements[Number(cmd[i])].checked = true;
          }
        }
      };
      xhr.open("GET", "/disp_ave_normal", true);
      xhr.send(null);
    }
    window.onload = disp_ave_normal;
  </script>
</html>)rawliteral";

String html_res_head = "HTTP/1.1 200 OK\r\nContent-type:text/html; "
                       "charset=utf-8\r\nConnection:close\r\n\r\n";
String html_res_head2 = "HTTP/1.1 200 OK\r\nContent-type:text/plain; "
                        "charset=utf-8\r\nConnection:close\r\n\r\n";
String html_tag1 =
    "<!DOCTYPE HTML>\r\n<html>\r\n<head>\r\n"
    "<meta charset='utf-8'>\r\n"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>\r\n"
    "<style>\r\n"
    "  html { font-family: Helvetica, Arial, sans-serif; display: "
    "inline-block; margin: 0px auto; text-align: center; }\r\n"
    "  h1 { font-size: 24px; margin-bottom: 20px; }\r\n"
    "  body { text-align: center; margin: 20px auto; max-width: 480px; }\r\n"
    "  select, input[type=password], input[type=text] { font-size: 16px; "
    "padding: 8px; margin: 6px 0; width: 90%; max-width: 320px; box-sizing: "
    "border-box; }\r\n"
    "  button { font-size: 16px; padding: 8px 24px; margin: 10px; border: 1px "
    "solid #aaa; border-radius: 4px; cursor: pointer; }\r\n"
    "  a { color: navy; text-decoration: none; font-size: 18px; }\r\n"
    "</style>\r\n"
    "</head>\r\n"
    "<body>\r\n"
    "<h1>WiFi Setting</h1>\r\n";
String html_tag2 = "\r\n</body>\r\n</html>\r\n\r\n";

// -----------------------------------------------------------------------------
// 関数プロトタイプ宣言
// -----------------------------------------------------------------------------
boolean eeprom_read(void);
void eeprom_write(void);
void wifi_connect(void);
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
void favicon_response(void);
String HTML_Select_Box_str(String Sel_Ssid);
int split(String data, char delimiter, String *dst, int max);
boolean is_float(String str);
boolean is_number(String str);
boolean chk_host_ip(String *str);
String format_pass(String *pass_tmp);
String get_hardware_mac(void);
void update_client_id(void);
void get_mac_from_url(String req_str);
void get_topic_from_url(String req_str);
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
    stmp = pass_tmp->charAt(i);
    if (stmp == "%") {
      stmp2 = pass_tmp->substring(i + 1, i + 3);
      i += 2;
      stmp2.toCharArray(ctmp, 3);
      int itmp = strtol(ctmp, NULL, 16);
      ctmp[0] = itmp;
      ctmp[1] = '\0';
      pw += String(ctmp);
    } else {
      pw += stmp;
    }
  }
  return pw;
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
  if (PARA.use_custom_mac == 1 && PARA.custom_mac.length() > 0) {
    CLIENT_ID = PARA.custom_mac;
  } else {
    CLIENT_ID = hw_mac;
  }
}

void IRAM_ATTR resetModule() { esp_restart(); }

// -----------------------------------------------------------------------------
// EEPROM 読み書き (統合版)
// -----------------------------------------------------------------------------
void eeprom_write(void) {
  UnifiedEepromSettings cfg;
  memset(&cfg, 0, sizeof(cfg));
  strcpy(cfg.magic, "VST_U03");
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

  cfg.use_custom_mac = PARA.use_custom_mac;
  strncpy(cfg.custom_mac, PARA.custom_mac.c_str(), sizeof(cfg.custom_mac) - 1);
  strncpy(cfg.pub_topic, PARA.pub_topic.c_str(), sizeof(cfg.pub_topic) - 1);

  EEPROM.put(0, cfg);
  EEPROM.commit();
}

boolean eeprom_read(void) {
  EEPROM.begin(sizeof(UnifiedEepromSettings));
  UnifiedEepromSettings cfg;
  EEPROM.get(0, cfg);

  if (strcmp(cfg.magic, "VST_U03") == 0) {
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

    PARA.use_custom_mac = cfg.use_custom_mac;
    PARA.custom_mac = String(cfg.custom_mac);
    PARA.custom_mac.trim();

    PARA.pub_topic = String(cfg.pub_topic);
    PARA.pub_topic.trim();
    if (PARA.pub_topic.length() == 0) {
      PARA.pub_topic = "pub_prod";
    }

    update_client_id();
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

    PARA.use_custom_mac = cfg.use_custom_mac;
    PARA.custom_mac = String(cfg.custom_mac);
    PARA.custom_mac.trim();
    PARA.pub_topic = "pub_prod";

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

    PARA.use_custom_mac = 0;
    PARA.custom_mac = "";
    PARA.pub_topic = "pub_prod";

    update_client_id();
    eeprom_write();
    return true;
  } else {
    // 初期値設定
    PARA.model_no = 0; // rex noise/vibration
    PARA.s_n_xave_flg[0] = "0";
    PARA.s_n_xave_flg[1] = "0";
    PARA.s_n_xave_flg[2] = "0";
    PARA.s_n_xave_flg[3] = "0";
    PARA.host_ip = "192.168.11.11";
    PARA.shreshold = 9999.0;
    PARA.meas_period = 600;
    PARA.use_custom_mac = 0;
    PARA.custom_mac = "";
    PARA.pub_topic = "pub_prod";

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
  if ((para->meas_large - para->meas_small) == 0) {
    return val;
  }
  float ftmp =
      para->para_small + ((float)(para->para_large - para->para_small) /
                          (para->meas_large - para->meas_small)) *
                             (val - para->meas_small);
  return ftmp;
}

void read_mcp3424(void) {
  int ch_num, val[3];
  static int adc_conv_time = 6;
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
        delay(5);
        break;
      } else if (millis() - spl_start >= adc_conv_time) {
        break;
      }
    }
  }
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
  eeprom_write();
}

void meas_trigger_rain(void) { RAIN_FLAG = true; }

// 10ms周期 雨量パルス監視
void meas_rain_sample(unsigned long now) {
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
      } else {
        int sample_count = MCNT + 1;
        if (sample_count > 6000)
          sample_count = 6000;

        for (int i = 0; i < 2; i++) {
          // C++標準ライブラリの最高速ソート (Introsort / Quicksort)
          // で降順ソート
          std::sort(SORT_DATA[i], SORT_DATA[i] + sample_count,
                    std::greater<unsigned int>());

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
    int target_ticks = (SMPL_TIME >= 1000)
                           ? 100
                           : 10; // 通常時: 10回(100ms), AP時: 100回(1000ms)
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
  Serial.printf("Time: %02d:%02d\n", tm->tm_hour, tm->tm_min);
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
    } else {
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
      delay(100);
    }
    Serial.print("WiFi connecting ");
    Serial.println(i++);
    if (i >= 3) {
      WiFi.begin();
      i = 0;
    }
  }
  if (time_adj_flag && !AP_MODE) {
    set_sysclcok();
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
  Serial.printf("Publishing to [%s]: ", PARA.pub_topic.c_str());
  Serial.println(str);
  mqttClient.publish(PARA.pub_topic.c_str(), str);
  Serial.println("Published.\n");
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

  Serial.println("Measured Data Ready:");
  for (int i = 0; i < 25; i++) {
    Serial.print(sdata[i]);
    Serial.print(" ");
  }
  Serial.println("");

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

  case 3: // rex 雨量
  {
    float ftmp = sdata[24] * 0.5; // 1pulse = 0.5mm
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
      Serial.println("RELAY ON");
    } else {
      digitalWrite(RELAY_OUT, LOW);
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

#if CLOUD_DEBUG == 1
    sprintf(pub_msg, "{\"ch1\": \"%s\"}", st_ch4);
#elif CLOUD_DEBUG == 2
    sprintf(st_ch1, "%s@%s@%s@%s@%s@%s@%s@%s", st_ch4, st_ch4, st_ch4, st_ch4,
            st_ch4, st_ch4, st_ch4, st_ch4);
    sprintf(pub_msg,
            "{\"id\":\"rx01\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\","
            "\"ch4\":\"%s\"}",
            st_ch1, st_ch1, st_ch4, st_ch4);
#elif CLOUD_DEBUG == 3
    sprintf(pub_msg,
            "{\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\",\"ch4\":\"%s\"}",
            st_ch4, st_ch4, st_ch4, st_ch4);
#elif CLOUD_DEBUG == 4
    sprintf(pub_msg,
            "{\"id\":\"rx02\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\","
            "\"ch4\":\"%s\",\"time\":\"%s\"}",
            st_ch4, st_ch4, st_ch4, st_ch4, st_time);
#else
    sprintf(pub_msg,
            "{\"id\":\"rx02\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\","
            "\"ch4\":\"%s\",\"time\":\"%s\"}",
            st_rain, st_ch2, st_ch3, st_ch4, st_time);
#endif
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
  String str = "";
  String selected_str = "";
  str += "<form name='F_ssid_select' action='/wifi_set/' method='GET'>\r\n";
  str += "  <label for='ssid_select'><b>SSID:</b></label><br>\r\n";
  str += "  <select name='ssid_select' id='ssid_select'>\r\n";
  for (int i = 0; i < ssid_num; i++) {
    selected_str = (Selected_SSID_str == ssid_str[i]) ? " selected" : "";
    str += "    <option value=\"" + ssid_str[i] + "\"" + selected_str + ">" +
           ssid_rssi_str[i] + "</option>\r\n";
  }
  str += "  </select><br>\r\n";
  str += "  <a href='/wifi_rescan' style='display:inline-block; padding:4px "
         "10px; margin:4px 0 12px 0; background:#e0e0e0; border-radius:4px; "
         "font-size:13px; color:#333;'>再検索</a><br>\r\n";
  str += "  <label for='pass1'><b>Password:</b></label><br>\r\n";
  str += "  <input type='password' name='pass1' id='pass1'><br>\r\n";
  str += "  <button type='submit' name='ssid_sel_submit' value='send' "
         "style='background-color:#AFA; font-weight:bold;'>SET</button>\r\n";
  str += "</form><br><br>\r\n";
  str += "<a href='/' style='color:navy;'>Home</a>\r\n";
  return str;
}

void html_send(boolean sta_connected, String message1, String message2,
               String color, String html_res_head, String html_tag1,
               String html_tag2) {
  client.print(html_res_head);
  client.print(html_tag1);
  client.print(HTML_Select_Box_str(message1));
  if (message2.length() > 0 && message2 != "Connection close") {
    client.printf(
        "<p style='color:%s; font-size:100%%; font-weight:bold;'>%s</p>\r\n",
        color.c_str(), message2.c_str());
  }
  if (sta_connected) {
    client.print("<span style='font-size:100%; color:blue;'>IP = ");
    client.print(LIP);
    client.print("<br></span>\r\n");
  }
  client.print(html_tag2);
}

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
  }
}

void wifi_scan(void) {
  Serial.println("scan start");
  int16_t n = WiFi.scanNetworks(false, false, false, 120);
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
      "<style>\r\n"
      "  html { font-family: Helvetica, Arial, sans-serif; display: "
      "inline-block; margin: 0px auto; text-align: center; }\r\n"
      "  body { margin-top: 50px; }\r\n"
      "  h1 { font-size: 22px; color: #333; }\r\n"
      "  p { font-size: 15px; color: #666; }\r\n"
      "  .loader { margin: 24px auto; border: 5px solid #f3f3f3; border-top: "
      "5px solid #2196F3; border-radius: 50%; width: 40px; height: 40px; "
      "animation: spin 1s linear infinite; }\r\n"
      "  @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: "
      "rotate(360deg); } }\r\n"
      "  a { color: navy; text-decoration: none; font-size: 15px; }\r\n"
      "</style>\r\n"
      "</head>\r\n"
      "<body>\r\n"
      "<h1>Wi-Fiを再検索中...</h1>\r\n"
      "<div class='loader'></div>\r\n"
      "<p>周囲のWi-Fiアクセスポイントをスキャンしています。<br>"
      "約3秒後に自動で設定画面へ戻ります。</p>\r\n"
      "</body>\r\n</html>\r\n\r\n";

  client.print(html_res_head);
  client.print(html_scan);
  client.flush();
  delay(50);
  client.stop();
  Serial.println("client disconnected (rescan page sent)");
}

void wifi_set_proc() {
  Serial.println("GET /wifi_set");
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // スキャン実行中なら最大1秒待機
  unsigned long start_wait = millis();
  while (WiFi.scanComplete() == -1 && (millis() - start_wait < 1000)) {
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

void wifi_set_submit(String req_str) {
  String pass_tmp;
  int16_t getTXT_select = req_str.indexOf("?ssid_select=");
  int16_t getTXT_close = req_str.indexOf("connection_close=");
  if (getTXT_select > 0) {
    Selected_SSID_str =
        req_str.substring(getTXT_select + 13, req_str.indexOf("&pass1"));
    pass_tmp = req_str.substring(req_str.indexOf("&pass1=") + 7,
                                 req_str.indexOf("&ssid_sel_submit"));
  }
  Sel_SSID_PASS_str = format_pass(&pass_tmp);

  if (Sel_SSID_PASS_str == "`@r") {
    Selected_SSID_str = "RUT240_8B10";
    Sel_SSID_PASS_str = "k5N0XpQb";
  } else if (Sel_SSID_PASS_str == "`@b") {
    Selected_SSID_str = "Buffalo-G-FBF8";
    Sel_SSID_PASS_str = "ck8m7ah5v6dkw";
  }
  Serial.println(Selected_SSID_str);
  Serial.println(Sel_SSID_PASS_str);

  if (getTXT_close < 0) {
    while (client.available())
      client.read();
    delay(100);
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
        }
      } else {
        LIP = WiFi.localIP();
        Serial.print("\r\nWiFi connected: ");
        Serial.println(LIP);
        html_send(true, Selected_SSID_str, "本体を再起動しました<br>", "#00F",
                  html_res_head, html_tag1, html_tag2);
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

  // WiFi接続に成功してIPアドレスを取得できた場合、再起動する
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected & IP obtained. Rebooting ESP32...");
    delay(1000); // レスポンスがブラウザに確実に届くよう待機
    esp_restart();
  }
}

String get_trans_param_str() {
  // 12個の変換パラメータ (val, large, small) x 4ch + 測定周期 + モデルNo
  String str = "";
  for (int i = 0; i < 4; i++) {
    str += String(md_trans(RAW_MD[i], &T_PARA[i]), 2) + ",";
    str += String(T_PARA[i].para_large, 2) + ",";
    str += String(T_PARA[i].para_small, 2) + ",";
  }
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

void get_mac_from_url(String req_str) {
  int16_t idx_mode = req_str.indexOf("use_custom_mac=");
  if (idx_mode > 0) {
    int16_t idx_custom_mac = req_str.indexOf("&custom_mac=");
    int16_t idx_submit = req_str.indexOf("&mac_submit");
    if (idx_custom_mac > 0) {
      String s_mode = req_str.substring(idx_mode + 15, idx_custom_mac);
      PARA.use_custom_mac = s_mode.toInt();
      String s_mac = "";
      if (idx_submit > idx_custom_mac) {
        s_mac = req_str.substring(idx_custom_mac + 12, idx_submit);
      } else {
        s_mac = req_str.substring(idx_custom_mac + 12);
      }
      s_mac = format_pass(&s_mac);
      s_mac.trim();
      PARA.custom_mac = s_mac;
      update_client_id();
      eeprom_write();
      Serial.printf("MAC Setting saved: use_custom=%d, custom_mac=%s, CLIENT_ID=%s\n",
                    PARA.use_custom_mac, PARA.custom_mac.c_str(), CLIENT_ID.c_str());
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

void favicon_response() {
  while (client.available())
    client.read();
  client.print(F("HTTP/1.1 404 Not Found\r\nConnection:close\r\n\r\n"));
  delay(10);
  client.stop();
}

void wifi_access_point() {
  static String pre_url;
  client = server.available();
  String html_res_head404 =
      "HTTP/1.1 404 NOT "
      "Found\r\nContent-type:text/html\r\nConnection:close\r\n\r\n";

  if (client) {
    String req_str = "";
    while (client.connected()) {
      while (client.available()) {
        req_str = client.readStringUntil('\n');
        if (req_str.indexOf("\r") == 0)
          break;
        else if (req_str.indexOf("GET /wifi_set/?") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_set_submit(req_str);
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_rescan") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_rescan_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /wifi_set") >= 0) {
          pre_url = "GET /wifi_set";
          wifi_set_proc();
          req_str = "";
        } else if (req_str.indexOf("GET /mac_set/?") >= 0) {
          pre_url = "GET /mac_set";
          get_mac_from_url(req_str);
          client.print(html_res_head);
          client.print(str_mac_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /mac_set") >= 0) {
          pre_url = "GET /mac_set";
          client.print(html_res_head);
          client.print(str_mac_set);
          delay(10);
          client.stop();
          req_str = "";
        } else if (req_str.indexOf("GET /disp_mac_param") >= 0) {
          client.print(html_res_head2);
          String stmp = String(PARA.use_custom_mac) + "," + PARA.custom_mac +
                        "," + get_hardware_mac() + "," + CLIENT_ID;
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
        } else if (req_str.indexOf("GET /disp_trans_param") >= 0 ||
                   req_str.indexOf("GET /get_meas_param") >= 0) {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          String stmp = get_trans_param_str();
          client.print(stmp.c_str());
          delay(10);
          client.stop();
        } else if (req_str.indexOf("GET /param_set/?") >= 0) {
          pre_url = "GET /param_set";
          get_pram_from_url(req_str);
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
        } else if (req_str.indexOf("GET /f1c9t?") >= 0) {
          pre_url = "GET /f1c9t";
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
        } else if (req_str.indexOf("GET /f1c9t") >= 0) {
          PAGE_NUM = 0;
          pre_url = "GET /f1c9t";
          client.print(html_res_head);
          client.print(str_factory);
          delay(10);
          client.stop();
          req_str = "";
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
          if (pre_url.indexOf("GET /mac_set") >= 0)
            client.print(str_mac_set);
          else if (pre_url.indexOf("GET /topic_set") >= 0)
            client.print(str_topic_set);
          else if (pre_url.indexOf("GET /param_set") >= 0)
            client.print(str_calibration);
          else if (pre_url.indexOf("GET /meas_period_set") >= 0)
            client.print(str_meas_period);
          else if (pre_url.indexOf("GET /host_ip_set") >= 0)
            client.print(str_host_ip);
          else if (pre_url.indexOf("GET /f1c9t") >= 0)
            client.print(str_factory);
          else if (pre_url.indexOf("GET /ave_normal_set") >= 0)
            client.print(str_ave_normal);
          else if (pre_url.indexOf("GET /shreshold_set") >= 0)
            client.print(str_shreshold);
          else
            client.print(str_normal_4ch_cloud);
          delay(10);
          client.stop();
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
// 起動時情報表示
// -----------------------------------------------------------------------------
void disp_info(void) {
  Serial.println("\n================================");
#ifdef VST100
  Serial.println("VST-100 (Unified 1-Chip 1-CPU)");
#else
  Serial.println("VST-01 (Unified 1-Chip 1-CPU)");
#endif

  update_client_id();
  Serial.printf("ESP32 HARDWARE MAC: %s\n", get_hardware_mac().c_str());
  if (PARA.use_custom_mac == 1 && PARA.custom_mac.length() > 0) {
    Serial.printf("CLIENT_ID (Custom MAC): %s\n", CLIENT_ID.c_str());
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

  // ボタンが離されるのを待機
  wait_button_released();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());
  delay(100);
  server.begin();
  Serial.println("HTTP Server started in AP mode");
  wifi_scan(); // 起動時に事前スキャンを実行（クライアント接続中のチャネル切替による切断を防止）
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
  pinMode(XAP_BTN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PULSE_IN, INPUT);
  pinMode(RELAY_OUT, OUTPUT);
  digitalWrite(RELAY_OUT, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX: GPIO16, TX: GPIO17 (9600bps)

  // パラメータ読み出し
  eeprom_read();
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
  // Serial2から入力した文字をそのままSerial2に出力 (エコーバック)
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial2.write(c);
  }

  // APボタンのチャタリング防止＆エッジ検出 (40ms確定)
  if (check_ap_button_pressed()) {
    if (AP_MODE == false) {
      Serial.println("AP button pressed! Switching to SoftAP Mode...");
      start_ap_mode();
    } else {
      Serial.println("AP button pressed! Switching to Normal Mode...");
      start_normal_mode();
    }
  }

  if (AP_MODE) {
    SMPL_TIME = 1000;
    digitalWrite(STATUS_LED, HIGH);
    check_async_wifi_scan();
    wifi_access_point();
  } else {
    SMPL_TIME = 100;
    digitalWrite(STATUS_LED, LOW);

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