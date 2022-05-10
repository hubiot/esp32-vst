// ORG 2021/08/04 adachi
// 2021/12/15
// rex 騒音振動用 APボタンにチャタリング対策
//パスワードに `@0 の3文字でローカルに切り替え ,``` パラメータ表示, `@r 実験用RUT240WiFi設定, `@b 実験用BaffaloルータWiFi設定
//プロジェクトフォルダ(dataではない)にaws.hに認証データを入れておく
// aws証明書は、ソースの中にいれた
// esp32のflashを暗号化機能を使えば、独自に暗号化するより安全と判断
//証明書を変更するには再コンパイルが必要
// #define VST100 1
#include "esp_system.h"
#include <WiFi.h>
// #include "FS.h"
// #include "SPIFFS.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
// #include "TimeLib.h" //Use Arduino time library ver1.5
#include <HTTPClient.h>
// #include <ArduinoJson.h>
#include <EEPROM.h> //host ip 保管用
#include "aws.h"    //aws証明書
// #include "WireSlave.h"
// #define I2C_SLAVE_ADDR 0x08
char SC_BUF[200]; // serial char buff
int SCB_CNT = 0;  // serial char cnt
String SERIAL_BUF, PRE_SERIAL_BUF;
int PAGE_NUM = 0; // 0:wifi set 1:parameter set

String S_CH_NUM = "1";      // ch1 = 1
String S_LARGE_SMALL = "0"; // 0:large 1:small

struct eeprom_struct // EEPROMで利用する型を宣言
{
  char setting_para[128];
};

struct para_d //動作を規定するパラメータ
{
  int model_no;           // Model No. 0:rex noise/shake 1:4ch normal
  String s_n_xave_flg[4]; //演算 0:ave 1:normal ch1,2,3,4のそれぞれにセット
  // unsigned int meas_period; //測定周期(=通信周期)
  String host_ip; // host ip
};
para_d PARA;
// const char *pubTopic = "pub01"; //クラウドデバッグ環境用
const char *pubTopic = "pub_prod"; //クラウド製品版

#define SDA_PIN 21
#define SCL_PIN 22

// String rootCA_file = "/AmazonRootCA1.pem";        // rootCA
// String certificate_file = "/certificate.pem.crt"; // certificate
// String privateKey_file = "/private.pem.key";      // private
String ap_ssid = "TIC-AP";   // ESP32 softAP SSID
String ap_pass = "12345678"; // ESP32 softAP password
// const String MSC = "/msc.txt";
IPAddress LIP; // Local IP address
WiFiServer server(80);
WiFiClient client;
uint8_t ssid_num;
String ssid_rssi_str[30];
String ssid_str[30];
String Selected_SSID_str;
String Sel_SSID_PASS_str;
String CLIENT_ID; // mac addressをユニークなIDとして使用
uint32_t scanLastTime = 0;
boolean First_Scan_Set = true;
boolean CMD_RECEIVE_FLAG = false;
#define XAP_BTN 35 // io番号で指定する(pin no.ではない)
#define STATUS_LED 32
#define CXS 25           // 1:通常 0:セッティング
#define BZ_ON 5          // 1:ブザーオン 0:オフ
boolean AP_MODE = false; // true:アクセスポイントモード false:通常モード

int CHATTERING_AP[3] = {1, 1, 1}; //チャタリング対策
int CHATTERING_CNT = 0;           //チャタリング対策

// boolean tsf = false; //本体起動から10秒経ったらtrueにする

/* HTMLページ */
const char *strHtml = R"rawliteral(
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
      <tr><td>CH1</td><td><span id="val_ch1" class="value">%CH1%</span></td><td><span id="pl_ch1" class="value">%PL_CH1%</span></td><td><span id="ps_ch1" class="value">%PS_CH1%</span></td></tr>
      <tr><td>CH2</td><td><span id="val_ch2" class="value">%CH2%</span></td><td><span id="pl_ch2" class="value">%PL_CH2%</span></td><td><span id="ps_ch2" class="value">%PS_CH2%</span></td></tr>
      <tr><td>CH3</td><td><span id="val_ch3" class="value">%CH3%</span></td><td><span id="pl_ch3" class="value">%PL_CH3%</span></td><td><span id="ps_ch3" class="value">%PS_CH3%</span></td></tr>
      <tr><td>CH4</td><td><span id="val_ch4" class="value">%CH4%</span></td><td><span id="pl_ch4" class="value">%PL_CH4%</span></td><td><span id="ps_ch4" class="value">%PS_CH4%</span></td></tr>
    </table></p>
    <p style='color:brown; font-weight: bold'>Scaling Parameter Set</p>
    <form name='paremeter_set'>
      <p><table>
        <tr><th sytle='width: 30px'>CH</th><th>LARGE/SMALL</th><th>PARAMETER</th><th style='border-top-style:none'></th></tr>
        <tr><td sytle='width: 30px'>
          <select name="channel_number">
          <option value="1">CH1</option>
          <option value="2">CH2</option>
          <option value="3">CH3</option>
          <option value="4">CH4</option>
          </select>
        </td>
        <td sytle='width: 30px'>
          <select name="large_small">
          <option value="large">LARGE</option>
          <option value="small">SMALL</option>
          </select>
        </td>
        <td><input type='text' name='conv_param'></td><td><button type='submit' name='param_submit' value='send' style='background-color:#AFA;'>Set</button></td></tr>
      </table></p>
    </form>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>WiFi SET</a>
  </body>
  <script>
    var get_meas_param = function () {
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
      xhr.open("GET", "/get_meas_param", true);
      xhr.send(null);
    }
    setInterval(get_meas_param, 1000);
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
      <label for = model_no>Select Model</label>
      <select name="model_no">
        <option value="0">REX (NOISE/SHAKE)</option>
        <option value="1">Normal 4ch cloud</option>
        <option value="2">Normal 4ch local</option>
      </select>
      <button type='submit' name='factory_param_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    </form>
    <br>
    <br>
    <br>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
    <br>
    <br>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a>
    <br>
    <br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
    <br>
    <br>
    <a href='/meas_period_set/' style='color:navy; font-size:20px;'>Measurement Period</a>
    <br>
    <br>
    <a href='/ave_normal_set/' style='color:navy; font-size:20px;'>Average / Normal Setting</a>
  </body>
  <script>
    var factory_param = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let no = this.responseText;
          console.log('no');
          console.log(no);
          let elements = document.getElementsByName('model_no');
          elements[0].options[Number(no)].selected = true;
        }
      }
      xhr.open("GET", "/disp_factory_param", true);
      xhr.send(null);
    }
    window.onload = factory_param;    //ページ読み込み後実行
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
    <h1>ch1:noise ch2:shake</h1>
    <h1>ch3:average ch4:average</h1>
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a>
    <br>
    <br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
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
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a>
    <br>
    <br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
    <br>
    <br>
    <a href='/meas_period_set/' style='color:navy; font-size:20px;'>Measurement Period</a>
    <br>
    <br>
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
    <a href='/wifi_set/' style='color:navy; font-size:20px;'>WiFi Setting</a>
    <br>
    <br>
    <a href='/param_set/' style='color:navy; font-size:20px;'>Calibration</a>
    <br>
    <br>
    <a href='/meas_period_set/' style='color:navy; font-size:20px;'>Measurement Period Setting</a>
    <br>
    <br>
    <a href='/host_ip_set/' style='color:navy; font-size:20px;'>Server IP</a>
  </body>
</html>)rawliteral";

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
        <tr><th sytle='width: 30px'>CH</th><th>LARGE/SMALL</th><th>PARAMETER</th><th style='border-top-style:none'></th></tr>
        <tr><td sytle='width: 30px'>
          <select name="channel_number">
          <option value="1">CH1</option>
          <option value="2">CH2</option>
          <option value="3">CH3</option>
          <option value="4">CH4</option>
          </select>
        </td>
        <td sytle='width: 30px'>
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
    window.onload = ch_ls_param;    //ページ読み込み後実行
  </script>
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
      <br>
      <br>
      <button type='submit' name='host_ip_para_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
  </body>
  <script>
    var disp_host_ip = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText;
          console.log(cmd);
          let element =document.getElementById("host_ip_param1");
          element.value = cmd;
        }
      };
      xhr.open("GET", "/disp_host_ip", true);
      xhr.send(null);
    }
    window.onload = disp_host_ip;    //ページ読み込み後実行
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
      <label>input:</label><input type='text' name='ave_normal_param' value=""><label>(60 - 3600)</label>
      <br>
      <br>
      <button type='submit' name='meas_period_submit' value='send' style='background-color:#AFA;'>Set</button>
    </form>
    <br>
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
  </body>
  <script>
    var disp_meas_period = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText;
          console.log(cmd);
          document.getElementById("meas_period_val").innerHTML = cmd;
        }
      };
      xhr.open("GET", "/disp_meas_period", true);
      xhr.send(null);
    }
    setInterval(disp_meas_period, 1000);
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
    <a href='/' style='color:navy; font-size:20px;'>Home</a>
  </body>
  <script>
    var disp_ave_normal = function () {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          let cmd = this.responseText.split(',');
          console.log(cmd);
          for(let i=0;i<4;i++){
            let stmp = "average_normal" + i;
            let elements = document.getElementsByName(stmp);
            console.log(elements);
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

// AWS IoT Setting
const char *awsEndpoint = "a24t2172v8g5ia-ats.iot.ap-northeast-1.amazonaws.com";
const int awsPort = 8883;
WiFiClientSecure httpsClient;
PubSubClient mqttClient(httpsClient);

boolean first_flag = true; //最初のループであることを示すフラグ
unsigned long msum = 0;    // 10分間の合計
unsigned int mcnt = 0;     // 10分間に何回ループしたか

hw_timer_t *timer = NULL;    // watchdog timer用
const int wdtTimeout = 6000; // time in ms to trigger the watchdog

void eeprom_write(void)
{
  eeprom_struct ebuf; //メモリ上に実体を作成
  String sbuf = "";
  // model no: 0
  sbuf = String(PARA.model_no); // model No. 0:rex noise/shake,1:normal 4ch
  sbuf += ",";
  // ave normal flag : 1
  for (int i = 0; i < 4; i++)
  {
    sbuf += PARA.s_n_xave_flg[i]; // 1:normal 0:average ch1,ch2,ch3,ch4と個別設定
  }
  sbuf += ",";
  // host ip : 2
  sbuf += PARA.host_ip;
  sbuf.toCharArray(ebuf.setting_para, 128); // Stringをcharに
  EEPROM.put<eeprom_struct>(0, ebuf);       // 変数に値を書き込み
  EEPROM.commit();                          // EEPROMに書き込み
}

// eepromから変換用パラメータを読み込む
// 3個に区切れなければfalseデフォルト値を設定しtrueをリターン
boolean eeprom_read(void)
{
  int itmp;
  EEPROM.begin(128);                  // EEPROM開始（サイズ指定）
  eeprom_struct cbuf;                 //メモリ上に実体を作成
  EEPROM.get<eeprom_struct>(0, cbuf); //実態変数にEEPROMの値を代入
  String stmp = cbuf.setting_para;    // charをstringに変換
  String dst[16];                     //分割数　max 16まで対応
  itmp = split(stmp, ',', dst, 4);
  if (itmp != 3) // eepromにパラメータが入っていなければ
  {
    PARA.model_no = 0;              // rex:0
    PARA.s_n_xave_flg[0] = "0";     // ch1:ave:1
    PARA.s_n_xave_flg[1] = "0";     // ch2:ave
    PARA.s_n_xave_flg[2] = "0";     // ch3:ave
    PARA.s_n_xave_flg[3] = "0";     // ch4:ave
    PARA.host_ip = "192.168.11.11"; //  host_ip:2
    eeprom_write();
    return false;
  }
  else
  {
    PARA.model_no = dst[0].toInt();              // model no. :0
    if (PARA.model_no < 0 || PARA.model_no >= 3) //想定外だとrex noise/shake版にする
    {
      PARA.model_no = 0;
    }
    PARA.s_n_xave_flg[0] = dst[1].substring(0, 1); // normal / xave :1
    PARA.s_n_xave_flg[1] = dst[1].substring(1, 2); // normal / xave
    PARA.s_n_xave_flg[2] = dst[1].substring(2, 3); // normal / xave
    PARA.s_n_xave_flg[3] = dst[1].substring(3);    // normal / xave
    PARA.host_ip = dst[2];                         // host IP :2
    return true;
  }
}
//文字列がfloatかチェック(0-9 & "."が一つ)
boolean is_float(String str)
{
  int val;
  int cnt = 0;
  for (int i = 0; i < str.length(); i++)
  {
    val = str.charAt(i);
    if (val == 0x2e)
    {
      cnt++;
    }
    else if (val < 0x30 || val > 0x39)
    {
      return false;
    }
  }
  if (cnt <= 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//文字列が0-9かチェック
boolean is_number(String str)
{
  int val, i;
  for (i = 0; i < str.length(); i++)
  {
    val = str.charAt(i) - 0x30;
    if (val < 0 || val > 9)
    {
      return false;
    }
  }
  return true;
}
// 分割数 = 分割処理(文字列, 区切り文字, 配列)
// 返り値:分割数 エラーの場合は-1
int split(String data, char delimiter, String *dst, int max)
{
  int index = 0;
  int datalength = data.length();
  // int snum = sizeof(dst);
  for (int i = 0; i < datalength; i++)
  {
    char tmp = data.charAt(i);
    if (tmp == delimiter)
    {
      if (++index >= max) //渡された配列数より分割数が多くなったら
      {
        return -1;
      }
    }
    else
      dst[index] += tmp;
  }
  return (index + 1);
}
//正常なipアドレスかチェックする
boolean chk_host_ip(String *str)
{
  int i, tmp;
  String dst[4];
  i = split(*str, '.', dst, 4);
  if (i != 4) //.で4つに区切れていなければ
  {
    return false;
  }
  for (i = 0; i < 4; i++)
  {
    if (!is_number(dst[i])) // 0～9の文字でなければ
    {
      return false;
    }
  }
  for (i = 0; i < 4; i++)
  {
    tmp = dst[i].toInt();
    if (tmp > 255 || tmp < 0) // 0-255の範囲に入っていなければ
    {
      return false;
    }
  }
  return true; //正常なIPアドレスだとtrue
}
//正常なアドレスかチェックする
// mで4つに区切れていて、それぞれが数値ならtrue,そうでなければfalse
boolean chk_para(void)
{
  int i, div;
  String dst[4];
  Serial.print(PARA.host_ip);
  div = split(PARA.host_ip, 'm', dst, 4);
  if (div == 4) // mで4つに区切れていれば
  {
    for (i = 0; i < 4; i++)
    {
      if (!is_number(dst[i])) // 0～9の文字でなければ
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

// host ip の項目に入っている文字列がm区切りで、4つの数値が入っていれば、その値をPARAに設定する
//フォーマット
//通信周期mサンプリング周期mアベレージフラグ
// host ipならeepromへ保存
// void set_para_ip(void)
// {
//   int i, div;
//   String dst[4];
//   if (chk_para()) //パラメータとして設定可能なら
//   {
//     div = split(PARA.host_ip, 'm', dst, 4);
//     PARA.com_period = dst[0].toInt();
//     PARA.sampling_period = dst[1].toInt();
//     PARA.ave_flag = dst[2].toInt();
//     PARA.cxl = dst[3].toInt();
//     write_para_file();
//     Serial.println("Parameter set");
//   }
//   else if (chk_host_ip()) // host ipとして正常なら
//   {
//     eeprom_struct buf;                         //メモリ上に実体を作成
//     PARA.host_ip.toCharArray(buf.host_ip, 17); // Stringをcharに
//     EEPROM.put<eeprom_struct>(0, buf);         //変数に値を書き込み
//     EEPROM.commit();                           // EEPROMに書き込み
//     Serial.println("host ip set");
//   }
// }

void IRAM_ATTR resetModule()
{
  esp_restart();
}

void setup_awsiot()
{
  httpsClient.setCACert(rootCA);
  httpsClient.setCertificate(certificate);
  httpsClient.setPrivateKey(privateKey);
  mqttClient.setServer(awsEndpoint, awsPort);
  mqttClient.setCallback(mqttCallback);
}

void connect_awsiot()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    // CLIENT_ID += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying＿
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received. topic=");
  Serial.println(topic);
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.print("\n");
}

// PARAをファイルに書き出す
// boolean write_para_file(void)
// {
//   File fp = SPIFFS.open(MSC.c_str(), "w");
//   if (!fp || fp.isDirectory())
//   {
//     Serial.println("SPIFFS Failed to open file for writing");
//     return false;
//   }
//   else
//   {
//     fp.println(PARA.cxl);
//     fp.println(PARA.com_period);
//     fp.println(PARA.sampling_period);
//     fp.println(PARA.ave_flag);
//     fp.close();
//   }
// }

// getで受け取ったpasswordに記号が含まれていた場合、ascii codeを元の記号に戻す
String format_pass(String *pass_tmp)
{
  String stmp, stmp2, pw;
  char ctmp[3];
  int itmp;
  pw = "";
  for (int i = 0; i < pass_tmp->length(); i++)
  {
    stmp = pass_tmp->charAt(i);
    if (stmp == "%")
    {                                            //パスワードに%が含まれていたら
      stmp2 = pass_tmp->substring(i + 1, i + 3); //% 以降の 2文字切り取り
      i += 2;
      stmp2.toCharArray(ctmp, 3);    // char型の配列へ変換
      itmp = strtol(ctmp, NULL, 16); // 16進文字列を数値へ変換
      ctmp[0] = itmp;
      ctmp[1] = '\0';
      stmp2 = ctmp;        // charの文字列をstringへ変換
      pw += String(stmp2); // 16進数のasciiコードをstringに変換した文字をパスワード文字列に追加
    }
    else
    {
      pw += stmp;
    }
  }
  return pw;
}

/* 計測処理ロジック */
String get_meas_param()
{
  String str = "";
  String dst[14];       // split()を呼ぶ前に初期化しなければならない
  if (CMD_RECEIVE_FLAG) //コマンドを受け取っていたら
  {
    CMD_RECEIVE_FLAG = false;
    int itmp = split(SERIAL_BUF, ',', dst, 14);
    if (itmp != 14) // 14に分割されなければコマンドではないと判断
    {
      return "";
    }
    for (itmp = 1; itmp <= 12; itmp++) //先頭の","を除いてajaxにわたす文字列作成
    {
      str += dst[itmp];
      str += ","; //最後に","があってもajaxで無視される
    }
    return str;
  }
  else //コマンドを受け取っていなければ
  {
    return PRE_SERIAL_BUF; //前回のコマンドを返す
  }
}

String html_res_head = "HTTP/1.1 200 OK\r\nContent-type:text/html\r\nConnection:close\r\n\r\n";
String html_res_head2 = "HTTP/1.1 200 OK\r\nContent-type:text/plain\r\nConnection:close\r\n\r\n";
String html_tag1 = "<meta name='viewport' content='initial-scale=1.5'>\r\n</head>\r\n\r\n<body style='background:#fff; color:#000; font-size:100%;'>\r\nWiFi SET<br>\r\nSSID";
String html_tag2 = "\r\n</body>\r\n</html>\r\n\r\n";

void html_send(boolean sta_connected, String message1, String message2, String color, String html_res_head, String html_tag1, String html_tag2)
{
  client.print(html_res_head);
  client.print(html_tag1);
  // client.print(HTML_Select_Box_str("!xxxx", message1));
  client.print(HTML_Select_Box_str(message1));
  client.printf("<p style='color:%s; font-size:80%%'>%s</p>\r\n", color.c_str(), message2.c_str()); //%%と重ねなければならない
  if (sta_connected == true)
  {
    client.print("<span style='font-size:80%'>IP = ");
    client.print(LIP);
    client.print("<br>");
    client.print("</span>\r\n");
  }
  client.print(html_tag2);
  Serial.print(html_res_head);
  Serial.print(html_tag1);
  // Serial.print(HTML_Select_Box_str("!xxxx", message1));
  Serial.print(HTML_Select_Box_str(message1));
  Serial.printf("<p style='color:%s; font-size:80%%'>%s</p>\r\n", color.c_str(), message2.c_str());
  if (sta_connected == true)
  {
    Serial.print("<span style='font-size:80%'>IP = ");
    Serial.print(LIP);
    Serial.print("</span>\r\n");
  }
  Serial.print(html_tag2);
}

String HTML_Select_Box_str(String Sel_Ssid)
{
  // String HTML_Select_Box_str(String button_id, String Sel_Ssid){
  String str = "";
  String selected_str = "";
  str += "<form name='F_ssid_select'>\r\n";
  str += "  <select name='ssid_select'>\r\n";
  for (int i = 0; i < ssid_num; i++)
  {
    if (Selected_SSID_str == ssid_str[i])
    {
      selected_str = " selected";
    }
    else
    {
      selected_str = "";
    }
    str += "    <option value=" + ssid_str[i] + selected_str + ">" + ssid_rssi_str[i] + "</option>\r\n";
  }
  str += "</select><br>\r\n";
  str += "Password<br><input type='password' name='pass1'>\r\n";
  // str += "Password<br><input type='text' name='pass1' value='diikr7csk5cxf'>\r\n"; //debug用初期値
  // str += "Password<br><input type='text' name='pass1' value='ck8m7ah5v6dkw'>\r\n"; // debug用初期値
  // str += "<br><button type='submit' name='ssid_sel_submit' value='send' style='background-color:#AFA;' onclick='document.getElementById(\"ssid_sel_txt\").innerHTML=document.F_ssid_select.ssid_select.value;'>Start connection</button>\r\n";
  str += "<br><button type='submit' name='ssid_sel_submit' value='send' style='background-color:#AFA;'>SET</button>\r\n";
  str += "<br>";
  // str += "<br><button type='submit' name='unit_reset' value='send' style='background-color:#FAF;'>RESET</button>\r\n";
  str += "</form><br>\r\n";
  str += "<br>";
  str += "<a href=\"/\" style=\"color:navy\">Home</a>";
  // str += "<form name='F_connection_close'>\r\n";
  // str += "  <button type='submit' name='connection_close' value='send' style='background-color:#FAA;' onclick='document.getElementById(\"ssid_sel_txt\").innerHTML=\"Connection close\";'>Connection Close</button>\r\n";
  // str += "</form>\r\n";
  // str += "<br>  Selected SSID<br><span id='ssid_sel_txt'  style='font-size:80%;'>";
  // str += "<br><span id='ssid_sel_txt'  style='font-size:80%;'>";

  // str += Sel_Ssid;
  // str += "Selected SSID" + Sel_Ssid;
  str += "</span>\r\n";
  return str;
}

void wifi_set_proc()
{
  Serial.println("GET /wifi_set");
  while (client.available())
  {
    char c = client.read();
    Serial.write(c);
  }
  html_send(false, "Connection close", "Connection close", "#FFF", html_res_head, html_tag1, html_tag2);

  delay(10);
  client.stop();
  Serial.println("client disonnected");
  delay(10);
}

void wifi_set_submit(String req_str)
{
  String pass_tmp;
  int16_t getTXT_select = req_str.indexOf("?ssid_select=");
  int16_t getTXT_close = req_str.indexOf("connection_close=");
  // int16_t getTXT_u_reset = req_str.indexOf("unit_reset");
  if (getTXT_select > 0)
  {
    Selected_SSID_str = req_str.substring(getTXT_select + 13, req_str.indexOf("&pass1"));
    pass_tmp = req_str.substring(req_str.indexOf("&pass1=") + 7, req_str.indexOf("&ssid_sel_submit")); // ssid取得
  }
  Sel_SSID_PASS_str = format_pass(&pass_tmp); //アスキーコードを記号に戻す
  //コマンド抽出
  if (Sel_SSID_PASS_str == "`@r") //`@r なら動作確認用RUT240セット
  {
    Selected_SSID_str = "RUT240_8B10";
    Sel_SSID_PASS_str = "k5N0XpQb";
  }
  else if (Sel_SSID_PASS_str == "`@b") //`@b なら動作確認用wifiセット
  {
    Selected_SSID_str = "Buffalo-G-FBF8";
    Sel_SSID_PASS_str = "ck8m7ah5v6dkw";
  }
  Serial.println(Selected_SSID_str);
  Serial.println(Sel_SSID_PASS_str);
  if (getTXT_close < 0)
  {
    Serial.printf("Selected_SSID_str = %s\r\n", Selected_SSID_str.c_str());
    Serial.printf("Sel_SSID_PASS_str = %s\r\n", Sel_SSID_PASS_str.c_str());

    while (client.available())
    {
      char c = client.read();
      Serial.write(c);
    }
    delay(500); // Important! This delay is necessary to connect to the Access Point.

    WiFi.begin(Selected_SSID_str.c_str(), Sel_SSID_PASS_str.c_str());
    uint32_t timeout = millis();
    while (1)
    {
      boolean exit_flag = false;
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("no connect");
        delay(1000);
        if (millis() - timeout > 15000)
        {
          html_send(false, Selected_SSID_str, "TIME OUT", "#F00", html_res_head, html_tag1, html_tag2);
          exit_flag = true;
        }
      }
      else
      {
        Serial.println("connect");
        LIP = WiFi.localIP();
        Serial.println("\r\nWiFi connected");
        Serial.print("Local IP address: ");
        Serial.println(LIP);
        Serial.printf("\r\n-----------%s Connected!\r\n", Selected_SSID_str.c_str());
        html_send(true, Selected_SSID_str, "Set OK! Push RESET<br>", "#00F", html_res_head, html_tag1, html_tag2);
        exit_flag = true;
      }
      if (exit_flag)
      {
        break;
      }
    }
  }
  else
  {
    html_send(false, "---", "Closed!", "#F00", html_res_head, html_tag1, html_tag2);
    WiFi.disconnect(false); // false=WiFi_ON , true=WiFi_OFF
  }

  delay(10);
  client.stop();
  Serial.println("client disonnected");
  delay(10);
}

// APモード時のコマンドを受け取っていたら、先頭の","を除いて返す ajax
//(変換値 + large + small)x4ch + 測定周期 = 13個のパラメータ
//文字列の最初と最後に","があるので注意
String get_trans_param_func()
{
  String str = "";
  String dst[15]; // split()を呼ぶ前に初期化しなければならない
  int itmp = split(SERIAL_BUF, ',', dst, 15);
  if (itmp != 15) // 15に分割されなければコマンドではないと判断
  {
    return "";
  }
  for (itmp = 1; itmp <= 12; itmp++) //先頭の","を除いてajaxにわたす文字列作成
  {
    str += dst[itmp];
    str += ","; //最後に","があってもajaxで無視される
  }
  return str;
}

void param_set_submit(String req_str)
{
  Serial.println("param_set");
  PAGE_NUM = 1;
  int16_t idx_ch_num = req_str.indexOf("channel_number=");
  int16_t getTXT_u_reset = req_str.indexOf("unit_reset");
  if (getTXT_u_reset > 0)
  {
    esp_restart();
  }
  if (idx_ch_num > 0)
  {
    int16_t idx_large_small = req_str.indexOf("&large_small=");
    int16_t idx_conv_param = req_str.indexOf("&conv_param=");
    S_CH_NUM = req_str.substring(idx_ch_num + 15, idx_large_small);                                 // CH No.(1,2,3,4)
    S_LARGE_SMALL = req_str.substring(idx_large_small + 13, idx_conv_param);                        // 0:large small:1
    String s_conv_param = req_str.substring(idx_conv_param + 12, req_str.indexOf("&param_submit")); // 変換パラメータ
    if (is_float(s_conv_param))                                                                     // float変換できるなら,measへコマンド送信
    {
      // format: CH No., sxl , conv para
      Serial.print("PARAM_SET@");
      Serial.print(S_CH_NUM);
      Serial.print(",");
      Serial.print(S_LARGE_SMALL);
      Serial.print(",");
      Serial.println(s_conv_param);
      Serial2.print("PARAM_SET@");
      Serial2.print(S_CH_NUM);
      Serial2.print(",");
      Serial2.print(S_LARGE_SMALL);
      Serial2.print(",");
      Serial2.println(s_conv_param);
      // Serial1.print("PARAM_SET@");
      // Serial1.print(S_CH_NUM);
      // Serial1.print(",");
      // Serial1.print(S_LARGE_SMALL);
      // Serial1.print(",");
      // Serial1.println(s_conv_param);
    }
    else
    {
      Serial.println("param error");
    }
  }
}
void ave_normal_submit(String req_str)
{
  int16_t idx0 = req_str.indexOf("?ave_normal_param=");
  String stmp;
  if (idx0 > 0)
  {
    stmp = req_str.substring(idx0 + 18, req_str.indexOf("&meas_period_submit"));
    Serial.println(stmp);
  }
  unsigned int meas_period;
  meas_period = stmp.toInt(); // intに変換できなければ0
  if (meas_period >= 2 && meas_period <= 3600) //営業サンプルはmeasは60以下も受け付ける、製品版は60未満なら受け付けない
  // if (meas_period >= 2)
  {
    eeprom_write();                  // ave normalはcommで保存
    Serial2.print("OPE_PARAM_SET@"); // measへコマンド転送
    Serial2.println(meas_period);    // 測定周期転送
    Serial.print("OPE_PARAM_SET@");  // measへコマンド転送 debug用
    Serial.println(meas_period);     // 測定周期転送
  }
}
// APモード時のコマンドを受け取っていたら、測定周期＋ave/normalを返す ajax
//(変換値 + large + small)x4ch + 測定周期 = 13個のパラメータ
//文字列の最初と最後に","があるので注意
String get_meas_period_func()
{
  String dst[15]; // split()を呼ぶ前に初期化しなければならない
  int itmp = split(SERIAL_BUF, ',', dst, 15);
  Serial.print("SERIAL_BUF:");
  Serial.println(SERIAL_BUF);
  // Serial.println(itmp);
  if (itmp != 15) // 15に分割されなければコマンドではないと判断
  {
    return "";
  }
  String stmp = dst[13]; //測定周期取得
  return stmp;
}

void wifi_access_point()
{
  static String pre_url; // req_strにurl以外が入る場合がある。その場合は、前のurlを表示する
  // String html_res_head = "HTTP/1.1 200 OK\r\n";
  // html_res_head += "Content-type:text/html\r\n";
  // html_res_head += "Connection:close\r\n\r\n";
  // String html_tag1 = "<!DOCTYPE html>\r\n<html>\r\n<head>\r\n";
  // html_tag1 += "<meta name='viewport' content='initial-scale=1.5'>\r\n";
  // html_tag1 += "</head>\r\n\r\n";
  // html_tag1 += "<body style='background:#fff; color:#000; font-size:100%;'>\r\n";
  // html_tag1 += "WiFi SET<br>\r\n";
  // html_tag1 += "SSID";
  // String html_tag2 = "\r\n</body>\r\n</html>\r\n\r\n";

  // String html_res_head2 = "HTTP/1.1 200 OK\r\n";
  // html_res_head2 += "Content-type:text/plain\r\n";
  // html_res_head2 += "Connection:close\r\n\r\n";
  client = server.available();
  String html_res_head404 = "HTTP/1.1 404 NOT Found\r\n";
  html_res_head404 += "Content-type:text/html\r\n";
  html_res_head404 += "Connection:close\r\n\r\n";

  if (client)
  {
    Serial.println("new client");
    String req_str = "";
    String pass_tmp; // password temporary
    while (client.connected())
    {
      while (client.available()) // decode
      {
        req_str = client.readStringUntil('\n');
        if (req_str.indexOf("\r") == 0)
          break;
        else if (req_str.indexOf("GET /wifi_set/?") >= 0)
        {
          pre_url = "GET /wifi_set";
          Serial.println("GET /wifi_set/?");
          wifi_set_submit(req_str);
          req_str = "";
        }
        else if (req_str.indexOf("GET /wifi_set") >= 0)
        {
          wifi_scan();
          pre_url = "GET /wifi_set";
          wifi_set_proc();
          req_str = "";
        }
        else if (req_str.indexOf("GET /disp_trans_param") >= 0) // ajax
        {
          client.print(html_res_head2); // plain text
          String stmp = get_trans_param_func();
          client.print(stmp.c_str()); // ajax 返り値
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /param_set/?") >= 0)
        {
          pre_url = "GET /param_set";
          param_set_submit(req_str);
          req_str = "";
        }
        else if (req_str.indexOf("GET /param_set") >= 0)
        {
          pre_url = "GET /param_set";
          client.print(html_res_head);
          client.print(str_calibration);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /get_meas_param") >= 0) // ajax
        {
          PAGE_NUM = 1;
          client.print(html_res_head2);
          String stmp = get_meas_param();
          client.print(stmp.c_str());
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /meas_period_set/?") >= 0) // GET /meas_period_setより先に"?"付きを検出
        {
          Serial.println("GET /meas_period_set/?");
          pre_url = "GET /meas_period_set";
          ave_normal_submit(req_str);
          req_str = "";
        }
        else if (req_str.indexOf("GET /meas_period_set") >= 0)
        {
          Serial.println("GET /meas_period_set");
          pre_url = "GET /meas_period_set";
          client.print(html_res_head);
          client.print(str_meas_period);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /ave_normal_set/?") >= 0)
        {
          pre_url = "GET /ave_normal_set";
          PARA.s_n_xave_flg[0] = req_str.substring(req_str.indexOf("?average_normal0=") + 17, req_str.indexOf("&average_normal1="));
          PARA.s_n_xave_flg[1] = req_str.substring(req_str.indexOf("&average_normal1=") + 17, req_str.indexOf("&average_normal2="));
          PARA.s_n_xave_flg[2] = req_str.substring(req_str.indexOf("&average_normal2=") + 17, req_str.indexOf("&average_normal3="));
          PARA.s_n_xave_flg[3] = req_str.substring(req_str.indexOf("&average_normal3=") + 17, req_str.indexOf("&ave_normal_submit"));
          for (int i = 0; i < 4; i++) //"0" or "1" 以外がセットされたら"0"にする
          {
            if (PARA.s_n_xave_flg[i] != "0" && PARA.s_n_xave_flg[i] != "1")
            {
              PARA.s_n_xave_flg[i] = "0";
            }
          }
          eeprom_write();
        }
        else if (req_str.indexOf("GET /ave_normal_set") >= 0)
        {
          pre_url = "GET /ave_normal_set";
          client.print(html_res_head);
          client.print(str_ave_normal);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /host_ip_set/?") >= 0)
        {
          pre_url = "GET /host_ip_set";
          int16_t idx_host_ip = req_str.indexOf("?host_ip_param=");
          String stmp;
          if (idx_host_ip > 0)
          {
            stmp = req_str.substring(idx_host_ip + 15, req_str.indexOf("&host_ip_para_submit"));
          }
          Serial.println(req_str);
          Serial.println(stmp);
          if (chk_host_ip(&stmp)) // host ipとして正しいか？
          {
            PARA.host_ip = stmp;
          }
          Serial.println(PARA.host_ip);
          eeprom_write();
        }
        else if (req_str.indexOf("GET /host_ip_set") >= 0)
        {
          pre_url = "GET /host_ip_set";
          client.print(html_res_head);
          client.print(str_host_ip);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /disp_ave_normal") >= 0) // ajax
        {
          client.print(html_res_head2); // plain text
          String stmp;
          for (int i = 0; i < 4; i++)
          {
            stmp = stmp + PARA.s_n_xave_flg[i] + ","; // ave normal flg追加
          }
          client.print(stmp.c_str()); // ajax 返り値
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
          eeprom_write();
        }
        else if (req_str.indexOf("GET /disp_factory_param") >= 0) // ajax
        {
          client.print(html_res_head2); // plain text
          String stmp = String(PARA.model_no);

          Serial.println("GET /disp_factory_param");
          Serial.println(stmp);
          client.print(stmp.c_str()); // ajax 返り値
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /disp_trans_param") >= 0) // ajax
        {
          PAGE_NUM = 1;
          client.print(html_res_head2); // plain text
          String stmp = get_trans_param_func();
          client.print(stmp.c_str()); // ajax 返り値
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /disp_meas_period") >= 0) // ajax
        {
          String stmp;
          stmp = get_meas_period_func();
          PAGE_NUM = 1;
          client.print(html_res_head2); // plain text
          client.print(stmp.c_str());   // ajax 返り値
          Serial.print(stmp);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /ch_ls_param") >= 0) // ajax
        {
          String stmp;
          stmp = S_CH_NUM + "," + S_LARGE_SMALL;
          PAGE_NUM = 1;
          client.print(html_res_head2); // plain text
          client.print(stmp.c_str());   // ajax 返り値
          Serial.print(stmp);
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /disp_host_ip") >= 0) // ajax
        {
          client.print(html_res_head2); // plain text
          String stmp = PARA.host_ip;
          client.print(stmp.c_str()); // ajax 返り値
          Serial.print("ajax:");
          Serial.print(stmp.c_str());
          delay(10);
          client.stop();
        }
        else if (req_str.indexOf("GET /f1c9t?") >= 0) // factory
        {
          pre_url = "GET /f1c9t";
          Serial.println(req_str);
          String stmp = req_str.substring(req_str.indexOf("GET /?model_no=") + 21, req_str.indexOf("&factory_param_submit"));
          Serial.println(stmp);
          PARA.model_no = stmp.toInt();
          Serial.print("model no:");
          Serial.println(PARA.model_no);
          eeprom_write();
          client.print(html_res_head);
          client.print(str_factory);
          delay(10);
          client.stop();
          req_str = "";
        }
        else if (req_str.indexOf("GET /f1c9t") >= 0)
        {
          PAGE_NUM = 0;
          pre_url = "GET /f1c9t";
          client.print(html_res_head);
          client.print(str_factory);
          delay(10);
          client.stop();
          req_str = "";
        }
        else if (req_str.indexOf("GET /favicon") >= 0)
        {
          PAGE_NUM = 0;
          favicon_response();
          req_str = "";
        }
        else if (req_str.indexOf("GET /") >= 0)
        {
          PAGE_NUM = 0;
          client.print(html_res_head);
          if (PARA.model_no == 0)
          {
            client.print(str_rex_noise_shake);
          }
          else if (PARA.model_no == 1)
          {
            client.print(str_normal_4ch_cloud);
          }
          else if (PARA.model_no == 2)
          {
            client.print(str_normal_4ch_local);
          }
          delay(10);
          client.stop();
          req_str = "";
        }
        else
        {
          // pre_url = "GET /param_set/?";
          Serial.print("req_str:");
          Serial.print(req_str);
          client.print(html_res_head404);
          if (pre_url.indexOf("GET /param_set") >= 0) // reloadすると"new clientst: 192.168.4.1"がreq_strに入るため、その前のURLを表示
          {
            client.print(str_calibration);
          }
          else if (pre_url.indexOf("GET /meas_period_set") >= 0)
          {
            client.print(str_meas_period);
          }
          else if (pre_url.indexOf("GET /host_ip_set") >= 0)
          {
            client.print(str_host_ip);
          }
          else if (pre_url.indexOf("GET /f1c9t") >= 0) // factory
          {
            client.print(str_factory);
          }
          else if (pre_url.indexOf("GET /ave_normal_set") >= 0) // ave / normal
          {
            client.print(str_ave_normal);
          }
          else
          {
            client.print(str_normal_4ch_cloud);
          }
          delay(10);
          client.stop();
        }
      }
    }
  }
}

void wifi_scan(void)
{
  // static boolean FIRST_SCAN_FLAG = true;
  // Serial.println("wifi scan");
  // if ((FIRST_SCAN_FLAG == true) || ((millis() - scanLastTime) > scan_interval))
  // if ((millis() - scanLastTime) > scan_interval)
  // {
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  ssid_num = WiFi.scanNetworks();
  if (ssid_num > 30)
    ssid_num = 30;
  Serial.println("scan done\r\n");
  if (ssid_num == 0)
  {
    Serial.println("no networks found\r\n");
  }
  else
  {
    Serial.printf("%d networks found\r\n\r\n", ssid_num);
    for (int i = 0; i < ssid_num; ++i)
    {
      ssid_str[i] = WiFi.SSID(i);
      String wifi_auth_open = ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      ssid_rssi_str[i] = ssid_str[i] + " (" + WiFi.RSSI(i) + "dBm)" + wifi_auth_open;
      Serial.printf("%d: %s\r\n", i, ssid_rssi_str[i].c_str());
      delay(10);
    }
  }
  Serial.println("");
  // scanLastTime = millis();
  // FIRST_SCAN_FLAG = false;
  // }
}
//*******************************************
void favicon_response()
{
  Serial.println(F("-----------------------Favicon GET Request Received"));
  while (client.available())
  {
    Serial.write(client.read());
  }

  client.print(F("HTTP/1.1 404 Not Found\r\n"));
  client.print(F("Connection:close\r\n\r\n"));

  delay(10);
  client.stop();
  delay(10);

  Serial.println(F("-----------------Client.stop (by Favicon Request)"));
}

//設定用ファイルからパラメータを読み取る
// boolean read_para_file(void)
// {
//   //動作モード読み込み
//   File fp = SPIFFS.open(MSC.c_str(), "r");
//   if (!fp || fp.isDirectory())
//   {
//     Serial.println("SPIFFS Failed to open file for reading");
//     return false;
//   }
//   else
//   {
//     String readstr;
//     int cnt = 0;
//     while (1)
//     {
//       readstr = fp.readStringUntil('\n'); //改行まで１行読み出し
//       readstr.trim();
//       if (readstr == "")
//       {
//         break;
//       }
//       else
//       {
//         switch (cnt++)
//         {
//         case 0:
//           PARA.cxl = readstr.toInt(); //クラウド or ローカル
//           Serial.println(PARA.cxl);
//           break;
//         case 1:
//           PARA.com_period = readstr.toInt(); //通信周期
//           break;
//         case 2:
//           PARA.sampling_period = readstr.toInt(); //サンプリング周期
//           break;
//         case 3:
//           PARA.ave_flag = readstr.toInt(); // 1:通信周期での平均 0:測定値をそのまま出力
//         default:
//           break;
//         }
//       }
//     }
//     fp.close();
//     return true;
//   }
// }

void disp_ave_normal(void)
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(":");

    if (PARA.s_n_xave_flg[i] == "0")
    {
      Serial.println("average");
    }
    else
    {
      Serial.println("nomal");
    }
  }
}
//パラメータをシリアル表示
void disp_info(void)
{
  Serial.println("");
  Serial.println("================================");
  Serial.print("DATE:");
  Serial.println("2022-04-22");
  uint8_t mac0[6];
  esp_efuse_mac_get_default(mac0); // macアドレス読み取り
  String stmp;
  for (int i = 0; i < 6; i++)
  {
    stmp = String(mac0[i], HEX);
    if (stmp.length() < 2)
      stmp = "0" + stmp;
    CLIENT_ID += stmp;
    if (i < 5)
    {
      CLIENT_ID += "-";
    }
  }
  Serial.print("MAC ADDRESS:");
  Serial.println(CLIENT_ID);
  //動作モードを表示
  Serial.print("Model:");
  if (PARA.model_no == 0)
  {
    Serial.println("REX (NOISE/SHAKE)");
    Serial.println("CH1:noise");
    Serial.println("CH2:shake");
    Serial.println("CH3:average");
    Serial.println("CH4:average");
  }
  else if (PARA.model_no == 1)
  {
    Serial.println("Normal 4ch cloud");
    disp_ave_normal();
  }
  else if (PARA.model_no == 2)
  {
    Serial.println("Normal 4ch local");
    disp_ave_normal();
    Serial.print("Host IP:");
    Serial.println(PARA.host_ip);
  }
  Serial.println("================================");
}

void connect_local_host(void) //つながらなかった時リセットがかかるコードをいれること 2021/10/18
{
  if (!client.connect(PARA.host_ip.c_str(), 5000))
  {
    Serial.println("connection failed");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.println(i++);
      if (i == 3)
      {
        WiFi.begin();
        i = 0;
      }
      else
        delay(1000); //ウェイト無しで早くWiFi.status()を見に行くと全然接続状態にならない
    }
    return;
  }
}

void send_local_server(char *cdata)
{
  String path;
  String body;
  String dt;
  String payload;
  connect_local_host();
  // Serial.println(SC_BUF);
  String stmp = String(cdata);
  stmp.replace("{", "");
  path = "/ds_420ma";
  body = "{\"DEVICE_ID\":\"" + CLIENT_ID + "\"," + stmp;

  // Serial.print("send to local server: ");
  // Serial.println(body);
  payload = "POST " + path + " HTTP/1.1\r\n" +
            "Content-Type: application/json\r\n" +
            "Content-Length: " + body.length() + "\r\n" +
            "Connection: close\r\n\r\n" + body;
  client.print(payload.c_str());
}

void aws_mqtt_publish(char *str)
// void aws_mqtt_publish(String pub_msg)
{
  aws_connect();
  mqttClient.loop();
  // sprintf(pubMessage, "{\"ch1\": \"%s\"}", str);
  Serial.print("Publishing:");
  // Serial.println(pubTopic);
  // Serial.println(str);
  // strcpy(str, "{\"id\":\"rx01\",\"ch1\":\"5@10@5.0@9.0@9.5@1@21@15.5\",\"ch2\":\"5.5@1.1@8.5@L9.9@19.5@3@23@18.0\",\"ch3\":\"3.3\",\"ch4\": \"4.4\"}");
  Serial.println(str);
  mqttClient.publish(pubTopic, str);
  Serial.println("Published.");
}
// WiFiに接続しにいって、8秒間接続できなければ本体リセット
void wifi_connect(void)
{
  int i = 0;
  timerAlarmWrite(timer, 8000000, false); //ウォッチドッグタイマ8秒セット
  timerWrite(timer, 0);                   // reset timer (feed watchdog)
  timerAlarmEnable(timer);                //ウォッチドッグタイマ有効化
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED)
  { // wifiが切れていたらつなぎに行く
    Serial.println("WiFi connecting");
    Serial.println(i++);
    if (i == 3) // 3回WiFiに接続できなければ
    {
      if (CHATTERING_AP[0] == 0 && CHATTERING_AP[1] == 0 && CHATTERING_AP[2] == 0 && AP_MODE == false) // APボタンが押されている かつAPモードでなければ
      {
        esp_restart(); // reset
      }
      WiFi.begin(); //再接続(flashからssid,passwordを読みに行くのでパラメータは必要なし)
      i = 0;
    }
    else
      delay(1000); //ウェイト無しで早くWiFi.status()を見に行くと全然接続状態にならない
  }
  timerAlarmDisable(timer); //ウォッチドッグタイマ無効化
}

// awsに接続しにいって、21秒間接続できなければ本体リセット
void aws_connect(void)
{
  timerAlarmWrite(timer, 21000000, false); //ウォッチドッグタイマ21秒セット
  timerWrite(timer, 0);                    // reset timer (feed watchdog)
  timerAlarmEnable(timer);                 //ウォッチドッグタイマ有効化
  if (!mqttClient.connected())
  { // awsが切れていたら
    // Serial.println("aws connecting");
    Serial.println("connecting");
    connect_awsiot(); //接続する
  }
  timerAlarmDisable(timer); //ウォッチドッグタイマ無効化
}

void setup()
{
#ifdef VST100
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
#else
  Wire.begin(); //使用しないが接続されているの
#endif
  Serial.begin(115200);
  Serial2.begin(115200);
  eeprom_read();
  disp_info();

  // if (!SPIFFS.begin())
  // {
  //   Serial.println("SPIFFS failed, or not present");
  //   return;
  // }
  // if (!read_para_file()) // msc.txtよりパラメータを読み取りできなければ
  // {
  pinMode(XAP_BTN, INPUT);     // ap button
  pinMode(STATUS_LED, OUTPUT); // status led
  pinMode(CXS, OUTPUT);        // 1:通常 0:セッティングモード
  pinMode(BZ_ON, OUTPUT);      // 1:ON 0:OFF
  digitalWrite(CXS, HIGH);
  digitalWrite(BZ_ON, LOW);
  digitalWrite(STATUS_LED, LOW); // status led off

  if (digitalRead(XAP_BTN) == 0) // APボタンが押されていたらアクセスポイントモードで起動
  {
    digitalWrite(STATUS_LED, HIGH);
    AP_MODE = true;
    // if (!SPIFFS.begin())
    // {
    //   Serial.println("SPIFFS failed, or not present");
    //   return;
    // }
    WiFi.mode(WIFI_AP_STA); // AP mode and STA mode
    WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());
    delay(100);
    Serial.println("Setup done");

    // server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send_P(200, "text/plain", get_meas_param().c_str()); });
    // server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send_P(200, "text/plain", getHumidity().c_str()); });
    server.begin();
    Serial.println(F("Server started"));
  }
  else //通常モードで起動
  {
    delay(10); //ウェイト無しで早くWiFi.status()を見に行くと全然接続状態にならない
    // SPIFFS.begin(); // SPIFFS開始
    //ウォッチドッグタイマ設定
    timer = timerBegin(0, 80, true);                 // timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true); // attach callback
    wifi_connect();                                  // wifi接続
    if (PARA.model_no != 2)                          //クラウドならaws接続
    {
      setup_awsiot();
      aws_connect();
    }
    // readADC();  //最初のデータ取得のためad変換をスタートする
    // delay(100); //変換のため時間確保
    Serial2.println("dummy"); //一発目はなぜかちゃんと送信できないようなので、正常なコマンドを送信する前にダミー送信
  }
}

void loop()
{
  CHATTERING_AP[CHATTERING_CNT++] = digitalRead(XAP_BTN);
  if (CHATTERING_CNT >= 3)
  {
    CHATTERING_CNT = 0;
  }
  if (CHATTERING_AP[0] == 0 && CHATTERING_AP[1] == 0 && CHATTERING_AP[2] == 0 && AP_MODE == false) // APボタンが押されている かつ APモードでなければ
  {
    esp_restart(); // reset
  }
  // Serial.println(digitalRead(XAP_BTN));
  if (Serial2.available()) // 受信データがあるか？
  {
    char key = Serial2.read(); // 1文字読み込み
    if (key == '\n')
    {
      if (SCB_CNT > 0)
      {
        SC_BUF[--SCB_CNT] = '\0'; //終端
        SERIAL_BUF = String(SC_BUF);
        SCB_CNT = 0;
        if (!CMD_RECEIVE_FLAG)
        {
          CMD_RECEIVE_FLAG = true;
          Serial.println(SERIAL_BUF);
          PRE_SERIAL_BUF = SERIAL_BUF;
        }
      }
    }
    else
    {
      SC_BUF[SCB_CNT++] = key;
    }
  }
  if (AP_MODE) //アクセスポイントモードなら
  {
    wifi_access_point();
    digitalWrite(CXS, LOW);         //セッティングモード
    digitalWrite(STATUS_LED, HIGH); // status led on
    // Serial.println(PAGE_NUM);
    // if (PAGE_NUM == 0)
    // {
    //   wifi_scan(30000);
    // }
  }
  else if (CMD_RECEIVE_FLAG) //コマンドを受け取っていたら通信スタート
  {
    CMD_RECEIVE_FLAG = false;
    // digitalWrite(CXS, HIGH);
    String dst[26]; // split()を呼ぶ前に初期化しなければならない
    int itmp = split(SC_BUF, ',', dst, 26);
    Serial.println(itmp);
    String pub_msg;
    if (itmp == 26) //(VAL,AVE,L5,L10,L50,L90,L95,MAX,MIN,LEQ)x2 ,(VAL,AVE)x2 + 2(先頭と最後に","があるため)
    {
      for (itmp = 1; itmp <= 24; itmp++) //データが正常ならdst[1]からdst[24]に測定データが入っている
      {
        Serial.print(dst[itmp]);
        Serial.print(" ");
      }
      Serial.println("");
      String dt_ch1 = "";
      String dt_ch2 = "";
      for (itmp = 0; itmp < 8; itmp++)
      {
        dt_ch1 += dst[itmp + 3];
        if (itmp < 7)
        {
          dt_ch1 += "@";
        }
      }
      for (itmp = 0; itmp < 8; itmp++)
      {
        dt_ch2 += dst[itmp + 13];
        if (itmp < 7)
        {
          dt_ch2 += "@";
        }
      }
      SCB_CNT = 0;
      // sbuf = "";

      char st_ch1[200], st_ch2[200], st_ch3[10], st_ch4[10]; // mqtt送信用バッファ
      char pub_msg[500];
      // Serial.print("Model No.");  //debug
      // Serial.println(PARA.model_no);
      if (PARA.model_no == 0) // rex 騒音振動
      {
        dt_ch1.toCharArray(st_ch1, 200);
        dt_ch2.toCharArray(st_ch2, 200);
        dst[22].toCharArray(st_ch3, 10);
        dst[24].toCharArray(st_ch4, 10);
        sprintf(pub_msg, "{\"id\":\"rx01\",\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\",\"ch4\":\"%s\"}", st_ch1, st_ch2, st_ch3, st_ch4);
      }
      else if (PARA.model_no == 1 || PARA.model_no == 2) //ノーマル4ch
      {
        // ch1
        if (PARA.s_n_xave_flg[0] == "1")
        {
          dst[1].toCharArray(st_ch1, 200); // val
        }
        else
        {
          dst[2].toCharArray(st_ch1, 200); // ave
        }
        // ch2
        if (PARA.s_n_xave_flg[1] == "1")
        {
          dst[11].toCharArray(st_ch2, 200); // val
        }
        else
        {
          dst[12].toCharArray(st_ch2, 200); // ave
        }
        // ch3
        if (PARA.s_n_xave_flg[2] == "1")
        {
          dst[21].toCharArray(st_ch3, 200); // val
        }
        else
        {
          dst[22].toCharArray(st_ch3, 200); // ave
        }
        // ch4
        if (PARA.s_n_xave_flg[3] == "1")
        {
          dst[23].toCharArray(st_ch4, 200);
        }
        else
        {
          dst[24].toCharArray(st_ch4, 200);
        }
        sprintf(pub_msg, "{\"ch1\":\"%s\",\"ch2\":\"%s\",\"ch3\":\"%s\",\"ch4\":\"%s\"}", st_ch1, st_ch2, st_ch3, st_ch4);
      }
      wifi_connect();         // wifiの接続がなければ接続しに行く
      if (PARA.model_no != 2) //クラウドなら(Model No.2 4ch normal local以外はクラウドへ転送)
      {
        aws_mqtt_publish(pub_msg);   // awsへ送信 送信できなければリセットがかかる
        Serial2.println("ACK_COMM"); //クラウドへ転送後、measへackを返す
      }
      else //ローカルなら
      {
        send_local_server(pub_msg);  //ホストへ転送
        Serial2.println("ACK_COMM"); //クラウドへ転送後、measへackを返す 送信できなかった場合、リセットがかかっているのでここは実行されない
      }
    }
  }
}
