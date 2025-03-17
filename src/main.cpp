#include <Arduino.h>
#include <HardwareSerial.h>

//-----------------------------------------
// ピン定義
//-----------------------------------------
const int auxPin = 19; // LoRaモジュール AUX（動作状態確認用）
const int m0Pin = 48;  // LoRaモジュール M0
const int m1Pin = 47;  // LoRaモジュール M1
const int ledPin = 14; // デバッグLED

// 使用するUARTインスタンス（LoRa用）
// mySerial: RX=21, TX=20
HardwareSerial mySerial(1);

//-----------------------------------------
// 関数プロトタイプ
//-----------------------------------------
void enterConfigurationMode();
void exitConfigurationMode();
uint8_t calculateChecksum(const uint8_t *data, size_t length);
void sendCommand(const uint8_t *command, size_t len);
void configureLoRaModule();
void Addaddressandchannel();
void printHexData(const uint8_t *data, size_t length);
void decodeGPSData(const char *gpsString);

//-----------------------------------------
// LoRaモジュール設定用関数群
//-----------------------------------------
void enterConfigurationMode()
{
  digitalWrite(m0Pin, HIGH);
  digitalWrite(m1Pin, HIGH);
  delay(1000);
}

void exitConfigurationMode()
{
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  delay(1000);
}

uint8_t calculateChecksum(const uint8_t *data, size_t length)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++)
  {
    sum += data[i];
  }
  return sum & 0xFF;
}

void sendCommand(const uint8_t *command, size_t len)
{
  mySerial.write(command, len);
  delay(10);
  Serial.print("Send Command: ");
  for (size_t i = 0; i < len; i++)
  {
    Serial.printf("%02X ", command[i]);
  }
  Serial.println();
}

void configureLoRaModule()
{
  Serial.println("Configuring LoRa module...");
  // 設定モードに入る
  enterConfigurationMode();

  // 設定コマンド作成
  uint8_t command[] = {
      0xC0, // 書き込みコマンドヘッダ
      0x00, // 開始レジスタアドレス
      0x06, // 書き込み数（6バイト）
      0x00, // アドレス上位バイト:00
      0x03, // アドレス下位バイト:03
      0x62, // UART Serial Port Rate 9600, Air Data Rate 62500 bps
      0xC1, // ペイロード長 32 bytes, RSSI環境ノイズ無効化, 送信出力 13 dBm
      0x00, // チャンネル設定 0
      0xC0  // RSSI有効化, 固定送信モード
  };
  command[9] = calculateChecksum(command, sizeof(command) - 1); // チェックサム追加

  // コマンド送信
  sendCommand(command, sizeof(command));
  Serial.println("Configuration command sent.");

  // 応答確認
  delay(100);
  uint8_t response[10] = {0}; // 応答バッファ
  size_t responseIndex = 0;

  while (mySerial.available() && responseIndex < sizeof(response))
  {
    response[responseIndex++] = mySerial.read();
  }

  // 応答を出力
  Serial.print("Response: ");
  printHexData(response, responseIndex);

  // 動作モードに戻る
  exitConfigurationMode();

  Serial.println("LoRa module setup finished! Start Operation Mode.");
}

// 16進数でデータを表示するヘルパー関数
void printHexData(const uint8_t *data, size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
}

void Addaddressandchannel()
{
  uint8_t addressHigh = 0x00;
  uint8_t addressLow = 0x03;
  uint8_t channel = 0x00;
  mySerial.write(addressHigh);
  mySerial.write(addressLow);
  mySerial.write(channel);
}

// ASCII形式のGPSデータを解析する関数
void decodeGPSData(const char *gpsString)
{
  // GPS文字列から緯度と経度を抽出（形式: "latitude,longitude"）
  double latitude = 0.0, longitude = 0.0;
  int parsed = sscanf(gpsString, "%lf,%lf", &latitude, &longitude);

  if (parsed == 2)
  {
    // 正常にパースできた場合

    // デコードした値を標準形式で表示
    Serial.print("GPS Data: Lat=");
    Serial.print(latitude, 6); // 小数点以下6桁まで表示
    Serial.print(", Lon=");
    Serial.println(longitude, 6);

    // 度分秒形式でも表示
    int latDeg = (int)latitude;
    double latMin = (latitude - latDeg) * 60.0;
    int latMinInt = (int)latMin;
    double latSec = (latMin - latMinInt) * 60.0;

    int lonDeg = (int)longitude;
    double lonMin = (longitude - lonDeg) * 60.0;
    int lonMinInt = (int)lonMin;
    double lonSec = (lonMin - lonMinInt) * 60.0;

    Serial.printf("Latitude: %d° %d' %.2f\", Longitude: %d° %d' %.2f\"\n",
                  latDeg, latMinInt, latSec, lonDeg, lonMinInt, lonSec);
  }
  else
  {
    Serial.println("Error decoding GPS data. Invalid format.");
  }
}

void setup()
{
  Serial.begin(115200); // 高速なシリアル通信
  Serial.println("ASCII GPS Receiver Station Starting...");

  pinMode(auxPin, INPUT);
  pinMode(m0Pin, OUTPUT);
  pinMode(m1Pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  digitalWrite(ledPin, LOW);

  // LoRa用UART初期化（RX=20, TX=21）
  mySerial.begin(9600, SERIAL_8N1, 20, 21);

  configureLoRaModule();

  Serial.println("ASCII GPS Receiver Ready - Waiting for data...");
}

void loop()
{
  // コンソールからのコマンド送信処理
  if (Serial.available() > 0)
  {
    String cmdStr = Serial.readStringUntil('\n');
    cmdStr.trim();
    if (cmdStr.length() > 0)
    {
      Serial.print("Sending command: ");
      Serial.println(cmdStr);

      Addaddressandchannel();
      mySerial.write(cmdStr.c_str(), cmdStr.length());
      while (digitalRead(auxPin) == LOW)
      {
        delay(2);
      }
      mySerial.flush();
      digitalWrite(ledPin, HIGH);
      delay(50);
      digitalWrite(ledPin, LOW);
    }
  }

  // LoRaモジュールからのデータ受信処理
  if (mySerial.available() > 0)
  {
    // アドレスとチャンネル情報（3バイト）読み取り
    uint8_t addressHigh, addressLow, channel;
    if (mySerial.available() >= 3)
    {
      addressHigh = mySerial.read();
      addressLow = mySerial.read();
      channel = mySerial.read();

      Serial.printf("Address: %02X%02X, Channel: %02X\n", addressHigh, addressLow, channel);
    }
    else
    {
      Serial.println("Incomplete header received");
      return;
    }

    // ASCII形式のGPSデータを受信
    char buffer[64] = {0}; // 受信バッファ
    int index = 0;

    // タイムアウト付きでASCII文字列を読み取り
    unsigned long startTime = millis();
    while (mySerial.available() > 0 && index < sizeof(buffer) - 1 && (millis() - startTime < 100))
    {
      char c = mySerial.read();
      buffer[index++] = c;
    }
    buffer[index] = '\0'; // 文字列終端

    // 最後の1バイトはRSSI
    uint8_t rssi = 0;
    if (index > 0)
    {
      rssi = buffer[index - 1];
      buffer[index - 1] = '\0'; // RSSIを除外
    }

    // タイムスタンプと受信データ長を表示
    unsigned long currentTime = millis();
    Serial.printf("[%lu] Received %d bytes | RSSI: %d\n", currentTime, index, rssi);

    // 受信したASCII文字列を表示
    Serial.print("Raw Data: ");
    Serial.println(buffer);

    // GPSデータをデコード
    if (index > 5)
    { // 最低でも妥当なGPSデータとなる長さか確認
      decodeGPSData(buffer);
    }

    // LED点滅でデータ受信を視覚的に表示
    digitalWrite(ledPin, HIGH);
    delay(10);
    digitalWrite(ledPin, LOW);
  }
}