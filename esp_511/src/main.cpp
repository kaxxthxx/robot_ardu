#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "app_lab";
const char *password = "12345678";

WebServer server(80);

void handleRoot() {
  server.send(200, "text/plain", "Hello from ESP32!");
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}