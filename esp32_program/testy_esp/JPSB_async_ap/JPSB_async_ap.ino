


#include <WiFi.h>
#include <ESPAsyncWebServer.h>


#include <Arduino.h>
#include <HardwareSerial.h>

const char *ssid = "MyESP32AP";
const char *password = "testpassword";
 
AsyncWebServer server(80);

int i=0;
HardwareSerial MySerial(0);


static const uint8_t TXa = 21;
static const uint8_t RXa = 20;

 const char index_html[] PROGMEM = R"rawliteral( "<!DOCTYPE HTML><html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <style>
    body{
        margin: auto;
        width: 80%;
        height: 80%;
        display: flex;
        
        justify-content: center;
        
    }
    .footer{
        position: fixed;
        left: 0;
        bottom: 0;
        width: 100%;
        text-align: center;
        background-color: rgb(163, 193, 222);
    }
    .panel{
        margin-top: 200px;
        padding: 20px;
        display: flex;
        justify-content: space-between;
        width: 500px;
        height: 50px;
        border-radius: 10px;
        background-color: rgb(209, 209, 209);
        box-shadow: 10px 10px 5px #aaaaaa;
    }
    .data_input{
        font-size:24px;
        width: 200px;

    }
    label{
width: 400px ;
        font-size: 20px;
        margin-top: 12px;
        padding-right: 10px;
    }
    input[type=number]::-webkit-inner-spin-button,
input[type=number]::-webkit-outer-spin-button {
   opacity: 1;
}
  </style>
</head>
<body>

    <div class="panel">
  <label for="name">Prędkość silnika (obr/min):  </label>

<input type="number" placeholder="P" class="data_input" id="data_P" />
<input type="number" placeholder="I" class="data_input" id="data_I" />
<input type="number" placeholder="D" class="data_input" id="data_D" />
<input class="zatwierdz"
       type="button"
       value="Zatwierdź"
       onclick="sendPID()">
    </div>
<div class="footer">
    Aleksander Płocha
</div>
</body>
<script>
var sendPID = function(){
  var P = document.getElementById("data_P").value
  var I = document.getElementById("data_I").value
  var D = document.getElementById("data_D").value
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
   

  };
 xhttp.open("GET", "/getPID?PIDParam="+"P="+P+";I="+I+";D="+D+";", true); 
 xhttp.send();
  

}

var data="";
var getData = function(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      data = this.responseText;
        
    console.log(data);
    
    }

  };
  xhttp.open("GET", "/getdata", true);
  xhttp.send();
  
  }


setInterval(function ( ) {
    getData();
  
}, 100) ;

</script>
</html>)rawliteral";

String str ="start";
void setup(){
  Serial.begin(115200);


  MySerial.begin(115200, SERIAL_8N1, RXa, TXa);
  
 Serial.print("dupa");


  WiFi.softAP(ssid, password);
 
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });
  server.on("/getdata", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", str);
  });

  server.on("/getPID", HTTP_GET, [](AsyncWebServerRequest *request){
    
    if (request->hasParam("PIDParam")){
       String PID = request->getParam("PIDParam")->value().substring(0,24); 
       Serial.println(PID);
       MySerial.print(PID+"\r\n");
    }

    request->send(200, "text/plain", str);
  });
 
  server.begin();
}
 
void loop(){

if (MySerial.available()) {
    str  = MySerial.readStringUntil('\r');
    Serial.println(str);
  }
  //Serial.println("wysylam");
  //MySerial.print("P=125.5;I=0.323;D=2.212;\r\n");
  delay(1000);

  //MySerial.print("60\r\n");
  
  
  
  

}