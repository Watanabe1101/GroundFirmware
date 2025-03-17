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
void sendLoRaCommand(const uint8_t *command, size_t len);
void configureLoRaModule();

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
    Serial.printf("0x%02X ", command[i]);
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
  for (size_t i = 0; i < responseIndex; i++)
  {
    Serial.printf("0x%02X ", response[i]);
  }
  Serial.println();

  // 動作モードに戻る
  exitConfigurationMode();

  Serial.println("LoRa module setup finished! Start Operation Mode.");
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

void setup()
{
  Serial.begin(9600);
  Serial.println("Ground Station Starting...");

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

  Serial.println("Enter command via Serial Monitor (例: s, p, l, c, n, m, q, r, etc.):");
}

void loop()
{
  if (Serial.available() > 0)
  {
    String cmdStr = Serial.readStringUntil('\n');
    cmdStr.trim();
    if (cmdStr.length() > 0)
    {
      Serial.print("Sending command: ");
      Addaddressandchannel();
      mySerial.write(cmdStr.c_str(), cmdStr.length());
      while (digitalRead(auxPin) == LOW)
      {
        delay(2);
      }
      Serial.println(cmdStr);
      mySerial.flush();
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
    }
  }

  if (mySerial.available() > 0)
  {
    uint8_t buffer[33]; // 最大32バイト + 1バイトRSSIの受信バッファ
    int index = 0;

    // 受信バッファからデータを取得
    while (mySerial.available() > 0 && index < sizeof(buffer))
    {
      buffer[index++] = mySerial.read();
    }

    if (index < 2)
    { // 最小データ長が2未満なら無視
      Serial.println("[DEBUG] Received incomplete data");
      return;
    }

    uint8_t rssi = buffer[index - 1]; // 最後の1バイトはRSSI
    int dataLength = index - 1;       // RSSIを除いたデータ長

    // 「GPS data not available」の受信処理
    if (dataLength == 23 && memcmp(buffer, "GPS data not available", 23) == 0)
    {
      Serial.print("[DEBUG] Received special message: GPS data not available | RSSI: ");
      Serial.println(rssi);
      return;
    }

    // データの解析
    if (dataLength == 16)
    { // GPSデータ単体
      double latitude, longitude;
      memcpy(&latitude, buffer, sizeof(latitude));
      memcpy(&longitude, buffer + sizeof(latitude), sizeof(longitude));

      Serial.print("[DEBUG] Received GPS (Only): N:");
      Serial.print(latitude, 6);
      Serial.print(", E:");
      Serial.print(longitude, 6);
      Serial.print(" | RSSI: ");
      Serial.println(rssi);
    }
    else if (dataLength == 29)
    { // GPS + クォータニオンデータ
      double latitude, longitude;
      float altitude;
      memcpy(&latitude, buffer, sizeof(latitude));
      memcpy(&longitude, buffer + sizeof(latitude), sizeof(longitude));
      memcpy(&altitude, buffer + sizeof(latitude) + sizeof(longitude), sizeof(altitude));

      // クォータニオンデータ（int16_t の4つの要素を float に変換）
      int16_t rawQuat[4];
      memcpy(rawQuat, buffer + 20, sizeof(rawQuat));

      float quaternion[4];
      for (int i = 0; i < 4; i++)
      {
        quaternion[i] = rawQuat[i] / 32768.0f; // -1.0 ~ 1.0 に正規化
      }

      Serial.print("[DEBUG] Received GPS: N:");
      Serial.print(latitude, 6);
      Serial.print(", E:");
      Serial.print(longitude, 6);
      Serial.print(", Alt:");
      Serial.print(altitude, 2);
      Serial.print(" | Quaternion: [");
      Serial.print(quaternion[0], 3);
      Serial.print(", ");
      Serial.print(quaternion[1], 3);
      Serial.print(", ");
      Serial.print(quaternion[2], 3);
      Serial.print(", ");
      Serial.print(quaternion[3], 3);
      Serial.print("] | RSSI: ");
      Serial.println(rssi);
    }
    else if (dataLength == 8)
    { // 電圧データ（float 2つ）
      float voltages[2];
      memcpy(voltages, buffer, sizeof(voltages));
      Serial.print("[DEBUG] Received Voltage1: ");
      Serial.print(voltages[0], 2);
      Serial.print(" V, Voltage2: ");
      Serial.print(voltages[1], 2);
      Serial.print(" V | RSSI: ");
      Serial.println(rssi);
    }
    else if (dataLength == 5)
    { // 各基板のステータス
      Serial.print("[DEBUG] Received Board Statuses: ");
      Serial.print("COM=");
      Serial.print(static_cast<char>(buffer[0]));
      Serial.print(" ");
      Serial.print("PARA=");
      Serial.print(static_cast<char>(buffer[1]));
      Serial.print(" ");
      Serial.print("POWER=");
      Serial.print(static_cast<char>(buffer[2]));
      Serial.print(" ");
      Serial.print("GIMBAL=");
      Serial.print(static_cast<char>(buffer[3]));
      Serial.print(" ");
      Serial.print("CAMERA=");
      Serial.print(static_cast<char>(buffer[4]));

      Serial.print(" | RSSI: ");
      Serial.println(rssi);
    }
    else
    {
      Serial.print("[DEBUG] Unknown data format (Length: ");
      Serial.print(dataLength);
      Serial.print(" bytes) | RSSI: ");
      Serial.println(rssi);

      // データの中身を16進数で表示
      Serial.print("Data: ");
      for (int i = 0; i < dataLength; i++)
      {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  delay(1);
}