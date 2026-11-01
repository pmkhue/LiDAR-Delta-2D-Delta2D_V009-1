/*
  Delta2D Radar V7.1 - ESP8266 D1 mini - SDK protocol 0x11
  Works with the observed Delta2D_V009-1 data stream using LDS_DELTA_2C_115200.
  Requires Arduino library: https://github.com/kaiaai/LDS (recent main)

  Wiring (power OFF whenever changing connections):
    LiDAR pin 5 TX -> ESP8266 D7 / GPIO13 (must be <= 3.3V logic)
    LiDAR pin 4 GND -> ESP8266 GND
    LiDAR pin 3 VCC -> separately REGULATED verified 5V sensor supply
    LiDAR pin 1/2 motor +/- -> external regulated 3V motor supply (~40mA observed)
  Motor power and logic supply must have an appropriate common ground reference.
  NEVER put 5V directly into D7; use a level shifter if the TX idle level is 5V.
  This is a TEST tool, NOT a collision avoidance / safety system.
*/
#include <Arduino.h>
#include "SectorResult.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <lds_all_models.h>
#include <math.h>
#include <string.h>

static const char AP_SSID[] = "Delta2D-Radar";
static const char AP_PASS[] = "12345678";
static const uint32_t BAUD_RATE = 115200;
static const uint16_t MIN_MM = 100;   // engineering display filter only
static const uint16_t MAX_MM = 4000;  // observed module saturation near 4 m

ESP8266WebServer server(80);
LDS_DELTA_2C_115200 driver;
LDS* lidar = &driver;

// Only the last COMPLETED revolution is shown. No data from older revolutions.
static uint16_t currentMm[360] = {0};
static uint8_t currentQuality[360] = {0};
static uint16_t completedMm[360] = {0};
static uint8_t completedQuality[360] = {0};

static uint32_t receivedBytes = 0;
static uint32_t packets = 0;
static uint32_t totalRaw = 0;
static uint32_t totalValid = 0;
static uint32_t totalZeros = 0;
static uint32_t completedScans = 0;
static uint32_t errors = 0;
static uint32_t checksumErrors = 0;
static int lastError = 0;
static uint16_t lastPacketLen = 0;
static int sdkStart = -999;

static uint32_t currentRaw = 0;
static uint32_t currentValid = 0;
static uint32_t currentZero = 0;
static uint32_t currentQualityZero = 0;
static uint32_t lastRaw = 0;
static uint32_t lastValid = 0;
static uint32_t lastZero = 0;
static uint32_t lastQualityZero = 0;
static uint32_t lastScanAt = 0;
static uint32_t prevScanAt = 0;
static uint32_t lastScanPeriod = 0;
static bool haveScan = false;

static uint32_t lastStatsAt = 0;
static uint32_t prevBytes = 0;
static uint32_t prevRaw = 0;
static uint32_t prevValid = 0;
static uint32_t bytesPerSec = 0;
static uint32_t rawPerSec = 0;
static uint32_t validPerSec = 0;

static char jsonStatus[1250];
static char jsonScan[3000];
static char jsonSector[380];

void motorPinCallback(float value, LDS::lds_pin_t pin) {
  (void)value;
  (void)pin; // External current-limited motor supply; NO GPIO motor control
}
int serialReadCallback() {
  int value = Serial.read();
  if (value >= 0) ++receivedBytes;
  return value;
}
size_t serialWriteCallback(const uint8_t* data, size_t length) {
  (void)data;
  return length; // No outbound UART connected to LiDAR
}
void packetCallback(uint8_t* data, uint16_t length, bool scanDone) {
  (void)data;
  (void)scanDone;
  ++packets;
  lastPacketLen = length;
}
void infoCallback(LDS::info_t code, String info) {
  (void)code;
  (void)info;
}
void errorCallback(LDS::result_t result, String info) {
  (void)info;
  ++errors;
  lastError = (int)result;
  if (result == LDS::ERROR_CHECKSUM) ++checksumErrors;
}

// In this SDK, scanDone=true appears on the FIRST sample of the NEW revolution.
// Close the previous revolution BEFORE storing that sample.
static void closeRevolution() {
  if (currentRaw) {
    memcpy(completedMm, currentMm, sizeof(completedMm));
    memcpy(completedQuality, currentQuality, sizeof(completedQuality));
    lastRaw = currentRaw;
    lastValid = currentValid;
    lastZero = currentZero;
    lastQualityZero = currentQualityZero;
    prevScanAt = lastScanAt;
    lastScanAt = millis();
    lastScanPeriod = prevScanAt ? lastScanAt - prevScanAt : 0;
    haveScan = true;
    ++completedScans;
  }
  memset(currentMm, 0, sizeof(currentMm));
  memset(currentQuality, 0, sizeof(currentQuality));
  currentRaw = 0;
  currentValid = 0;
  currentZero = 0;
  currentQualityZero = 0;
}

void scanPointCallback(float angle, float distance, float quality, bool scanDone) {
  if (scanDone) closeRevolution();

  ++totalRaw;
  ++currentRaw;
  if (quality <= 0.0f) ++currentQualityZero;

  if (!(distance > 0.0f) || !isfinite(distance)) {
    ++totalZeros;
    ++currentZero;
    return;
  }
  if (!isfinite(angle) || angle < 0.0f || angle >= 360.0f) return;
  if (distance < MIN_MM || distance > MAX_MM) return;

  ++totalValid;
  ++currentValid;

  const int bin = ((int)(angle + 0.5f)) % 360;
  uint8_t q = (uint8_t)(quality < 0 ? 0 : (quality > 255 ? 255 : quality));
  // Prefer the best-quality return when multiple samples land in one degree.
  if (!currentMm[bin] || q >= currentQuality[bin]) {
    currentMm[bin] = (uint16_t)(distance + 0.5f);
    currentQuality[bin] = q;
  }
}


static SectorResult calculateSector(int center, int halfWidth) {
  SectorResult r = {0, 0, 0, 0, 0, 0};
  if (!haveScan) return r;
  uint16_t sorted[61]; // halfWidth <= 30, at most 61 angular bins
  uint16_t qsum = 0;
  for (int delta = -halfWidth; delta <= halfWidth; ++delta) {
    const int i = (center + delta + 360) % 360;
    ++r.cells;
    uint16_t mm = completedMm[i];
    if (!mm) continue;
    qsum += completedQuality[i];
    // Insertion sort, tiny fixed-size input, no dynamic allocation.
    uint8_t p = r.hits;
    while (p > 0 && sorted[p - 1] > mm) {
      sorted[p] = sorted[p - 1];
      --p;
    }
    sorted[p] = mm;
    ++r.hits;
  }
  if (r.hits) {
    r.minimum = sorted[0];
    r.maximum = sorted[r.hits - 1];
    r.median = sorted[r.hits / 2];
    if (!(r.hits & 1)) {
      r.median = (uint16_t)(((uint32_t)sorted[r.hits / 2 - 1] + sorted[r.hits / 2]) / 2);
    }
    r.meanQuality = (uint8_t)(qsum / r.hits);
  }
  return r;
}

static void handleStatus() {
  const uint32_t now = millis();
  float zeroPct = lastRaw ? 100.0f * (float)lastZero / (float)lastRaw : 0.0f;
  int n = snprintf(jsonStatus, sizeof(jsonStatus),
    "{\"bytes\":%lu,\"bps\":%lu,\"heap\":%lu,\"raw_pps\":%lu,\"valid_pps\":%lu,"
    "\"raw_total\":%lu,\"valid_total\":%lu,\"zero_total\":%lu,\"packets\":%lu,"
    "\"scans\":%lu,\"last_raw\":%lu,\"last_valid\":%lu,\"last_zero\":%lu,"
    "\"last_quality_zero\":%lu,\"last_zero_pct\":%.2f,\"last_scan_age_ms\":%lu,"
    "\"last_scan_period_ms\":%lu,\"scan_hz\":%.2f,\"errors\":%lu,"
    "\"checksum_errors\":%lu,\"last_error\":%d,\"packet_len\":%u,\"sdk_start\":%d,"
    "\"uptime_s\":%lu,\"have_scan\":%s}",
    (unsigned long)receivedBytes, (unsigned long)bytesPerSec,
    (unsigned long)ESP.getFreeHeap(), (unsigned long)rawPerSec,
    (unsigned long)validPerSec, (unsigned long)totalRaw,
    (unsigned long)totalValid, (unsigned long)totalZeros,
    (unsigned long)packets, (unsigned long)completedScans,
    (unsigned long)lastRaw, (unsigned long)lastValid,
    (unsigned long)lastZero, (unsigned long)lastQualityZero,
    (double)zeroPct, (unsigned long)(haveScan ? now - lastScanAt : 0),
    (unsigned long)lastScanPeriod, (double)lidar->getCurrentScanFreqHz(),
    (unsigned long)errors, (unsigned long)checksumErrors, lastError,
    (unsigned)lastPacketLen, sdkStart, (unsigned long)(now / 1000),
    haveScan ? "true" : "false");
  if (n < 0 || (size_t)n >= sizeof(jsonStatus)) {
    server.send(500, "text/plain", "Status buffer too small");
    return;
  }
  server.send(200, "application/json", jsonStatus);
}

static void handleScan() {
  // 360 one-degree bins from last COMPLETED revolution; 0 means no return.
  size_t used = 0;
  int n = snprintf(jsonScan, sizeof(jsonScan), "{\"seq\":%lu,\"points\":[",
                   (unsigned long)completedScans);
  if (n < 0 || (size_t)n >= sizeof(jsonScan)) {
    server.send(500, "text/plain", "Buffer error"); return;
  }
  used = (size_t)n;
  for (int i = 0; i < 360; ++i) {
    n = snprintf(jsonScan + used, sizeof(jsonScan) - used,
                 "%s%u", i ? "," : "", (unsigned)completedMm[i]);
    if (n < 0 || (size_t)n >= sizeof(jsonScan) - used) {
      server.send(500, "text/plain", "Scan buffer too small"); return;
    }
    used += (size_t)n;
  }
  n = snprintf(jsonScan + used, sizeof(jsonScan) - used, "]}");
  if (n < 0 || (size_t)n >= sizeof(jsonScan) - used) {
    server.send(500, "text/plain", "Scan buffer end error"); return;
  }
  server.send(200, "application/json", jsonScan);
}

static void handleSector() {
  int angle = server.hasArg("angle") ? server.arg("angle").toInt() : 0;
  int halfWidth = server.hasArg("half") ? server.arg("half").toInt() : 8;
  angle = ((angle % 360) + 360) % 360;
  halfWidth = constrain(halfWidth, 1, 30);
  SectorResult r = calculateSector(angle, halfWidth);
  const uint32_t age = haveScan ? millis() - lastScanAt : 0;
  bool reliable = haveScan && age < 1000 && r.hits >= 3;
  int n = snprintf(jsonSector, sizeof(jsonSector),
    "{\"angle\":%d,\"half\":%d,\"median_mm\":%u,\"min_mm\":%u,"
    "\"max_mm\":%u,\"hits\":%u,\"cells\":%u,\"mean_quality\":%u,"
    "\"age_ms\":%lu,\"sufficient\":%s}",
    angle, halfWidth, (unsigned)r.median, (unsigned)r.minimum,
    (unsigned)r.maximum, (unsigned)r.hits, (unsigned)r.cells,
    (unsigned)r.meanQuality, (unsigned long)age,
    reliable ? "true" : "false");
  if (n < 0 || (size_t)n >= sizeof(jsonSector)) {
    server.send(500, "text/plain", "Sector buffer too small"); return;
  }
  server.send(200, "application/json", jsonSector);
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="vi"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Delta2D Radar V7.1</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#081321;color:#e4ebf5;font:14px system-ui,Arial}
main{max-width:1000px;margin:auto;padding:18px}header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap}
h1{font-size:23px;margin:0 0 4px}.sub,.hint{color:#93a9bc}.hint{font-size:12px;line-height:1.6}
.pill{padding:7px 10px;border:1px solid #375168;border-radius:40px;color:#fbbf24}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin:14px 0}
.card,.panel{background:#13243a;border:1px solid #2b435d;border-radius:12px;padding:13px}
.card .k{color:#9fb1c8;font-size:12px}.card .v{font-size:21px;font-weight:750;margin-top:5px}
.layout{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(260px,.85fr);gap:12px}
canvas{display:block;width:100%;height:auto;aspect-ratio:1;background:#0b1b2d;border-radius:9px}
input[type=range]{width:100%;accent-color:#48dcc4}select,button{background:#243c55;border:1px solid #496783;color:#fff;border-radius:7px;padding:7px 10px}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}.field{margin-bottom:15px}.field label{display:flex;justify-content:space-between;margin-bottom:7px;gap:8px}
.big{font-size:31px;font-weight:800;color:#4be8bd}.missing{color:#fbbf24}.small{font-size:12px;color:#afc1d2}
hr{border:0;border-top:1px solid #2b435d;margin:15px 0}.row{display:flex;justify-content:space-between;gap:10px;align-items:center;margin:7px 0}
@media(max-width:700px){.grid{grid-template-columns:repeat(2,1fr)}.layout{grid-template-columns:1fr}}
</style></head><body><main>
<header><div><h1>Delta2D · Radar V7.1</h1><div class="sub">ESP8266 D1 Mini · UART D7 · LDS protocol 0x11</div></div>
<div class="pill" id="health">Đang kết nối...</div></header>
<div class="grid">
<div class="card"><div class="k">UART (B/s)</div><div class="v" id="bps">—</div></div>
<div class="card"><div class="k">Quét (Hz)</div><div class="v" id="hz">—</div></div>
<div class="card"><div class="k">Điểm hợp lệ / vòng</div><div class="v" id="valid">—</div></div>
<div class="card"><div class="k">Zero / vòng</div><div class="v" id="zero">—</div></div>
</div>
<div class="layout">
<section class="panel"><canvas id="radar" width="560" height="560"></canvas>
<div class="row"><span class="small">Một vòng quét đã hoàn tất · 0° chưa hiệu chuẩn hướng trước</span>
<select id="range"><option value="500">0,5 m</option><option value="1000">1 m</option><option value="2000" selected>2 m</option><option value="4000">4 m</option></select></div>
<p class="hint">Các điểm xanh là khoảng cách hợp lệ; vùng màu cam là góc đang đo. Vạch cam không đại diện vật cản. Chọn vùng góc bằng thanh trượt bên phải.</p></section>
<section class="panel">
<h2 style="font-size:18px;margin-top:0">Đo vật cản theo góc</h2>
<div class="field"><label for="angle">Góc tâm <b id="angleLabel">0°</b></label><input id="angle" type="range" min="0" max="359" value="0"></div>
<div class="field"><label for="half">Nửa độ rộng vùng đo <b id="halfLabel">8°</b></label><input id="half" type="range" min="1" max="30" value="8"></div>
<div class="big" id="median">—</div><div class="small" id="sectorInfo">Chưa có dữ liệu vùng góc</div>
<hr><div class="row"><span>Khoảng cách nhỏ nhất</span><b id="minimum">—</b></div>
<div class="row"><span>Khoảng cách lớn nhất</span><b id="maximum">—</b></div>
<div class="row"><span>Chất lượng trung bình</span><b id="quality">—</b></div>
<div class="row"><span>Mẫu thô / vòng</span><b id="raw">—</b></div>
<div class="row"><span>Khoảng cách 0 / vòng</span><b id="zeros">—</b></div>
<div class="row"><span>Vòng quét đã hoàn tất</span><b id="scans">—</b></div>
<div class="row"><span>RAM trống</span><b id="heap">—</b></div>
<div class="row"><span>Lỗi SDK / checksum</span><b id="errors">—</b></div>
<hr><button id="csv">Lưu vòng quét dạng CSV</button>
<p class="hint">Đặt tấm bìa hướng cố định, chọn góc có cụm điểm bìa, đo 40 cm rồi 60 cm. Trung vị chỉ có ý nghĩa khi có đủ ít nhất 3 ô góc có tín hiệu.</p>
</section>
</div><p class="hint">Trang chẩn đoán, không dùng làm hệ thống an toàn robot. 5V chỉ cấp cho VCC LiDAR đã thử nghiệm; GPIO ESP8266 luôn tối đa 3,3V.</p>
</main><script>
const $=id=>document.getElementById(id), canvas=$('radar'),ctx=canvas.getContext('2d');
let bins=Array(360).fill(0),maxMM=2000,scanSeq=0,busy=false;
const angle=$('angle'),half=$('half');
$('range').onchange=()=>{maxMM=Number($('range').value);draw();};
function controls(){ $('angleLabel').textContent=angle.value+'°';$('halfLabel').textContent=half.value+'°';draw(); }
angle.oninput=controls;half.oninput=controls;
function draw(){
 let W=canvas.width,H=canvas.height,cx=W/2,cy=H/2,r=245;
 ctx.clearRect(0,0,W,H);ctx.fillStyle='#0b1b2d';ctx.fillRect(0,0,W,H);
 ctx.strokeStyle='#284760';ctx.lineWidth=1;
 for(let n=1;n<=4;n++){ctx.beginPath();ctx.arc(cx,cy,r*n/4,0,2*Math.PI);ctx.stroke();}
 ctx.beginPath();ctx.moveTo(cx-r,cy);ctx.lineTo(cx+r,cy);ctx.moveTo(cx,cy-r);ctx.lineTo(cx,cy+r);ctx.stroke();
 ctx.fillStyle='#9eb7cb';ctx.font='14px system-ui';ctx.fillText('0°',cx+8,cy-r+16);ctx.fillText('90°',cx+r-41,cy-10);
 for(let i=0;i<360;i++){let d=bins[i];if(!d||d>maxMM)continue;
   let a=i*Math.PI/180,xx=cx+Math.sin(a)*r*d/maxMM,yy=cy-Math.cos(a)*r*d/maxMM;
   ctx.fillStyle='#44e3b9';ctx.fillRect(xx-2,yy-2,4,4);
 }
 let a=Number(angle.value)*Math.PI/180, h=Number(half.value)*Math.PI/180;
 ctx.beginPath();ctx.moveTo(cx,cy);ctx.arc(cx,cy,r,a-h-Math.PI/2,a+h-Math.PI/2,false);ctx.closePath();
 ctx.fillStyle='rgba(255,181,70,.10)';ctx.fill();ctx.strokeStyle='#fbbf69';ctx.stroke();
 ctx.beginPath();ctx.arc(cx,cy,5,0,2*Math.PI);ctx.fillStyle='#ffc56b';ctx.fill();
}
function fmt(n){return Number(n).toLocaleString('vi-VN');}
async function update(){
 if(busy)return;busy=true;
 try{
 const a=angle.value,h=half.value;
 // Sequential, not parallel, to reduce ESP8266 UART starvation during HTTP responses.
 const rs=await fetch('/api/status',{cache:'no-store'});if(!rs.ok)throw Error('status');const s=await rs.json();
 const rr=await fetch('/api/scan',{cache:'no-store'});if(!rr.ok)throw Error('scan');const sc=await rr.json();
 const rc=await fetch('/api/sector?angle='+a+'&half='+h,{cache:'no-store'});if(!rc.ok)throw Error('sector');const t=await rc.json();
 bins=sc.points;scanSeq=sc.seq;draw();
 $('bps').textContent=fmt(s.bps);$('hz').textContent=Number(s.scan_hz).toFixed(2);
 $('valid').textContent=fmt(s.last_valid);$('zero').textContent=Number(s.last_zero_pct).toFixed(1)+'%';
 $('raw').textContent=fmt(s.last_raw);$('zeros').textContent=fmt(s.last_zero);
 $('scans').textContent=fmt(s.scans);$('heap').textContent=fmt(s.heap)+' B';
 $('errors').textContent=s.errors+' / '+s.checksum_errors;
 $('median').textContent=t.sufficient? (t.median_mm/10).toFixed(1)+' cm':'Không đủ điểm';
 $('median').className='big'+(t.sufficient?'':' missing');
 $('sectorInfo').textContent=t.hits+'/'+t.cells+' ô góc có phản hồi · vòng quét '+scanSeq+' · tuổi '+t.age_ms+' ms';
 $('minimum').textContent=t.hits?t.min_mm+' mm':'—';$('maximum').textContent=t.hits?t.max_mm+' mm':'—';
 $('quality').textContent=t.hits?t.mean_quality+' / 255':'—';
 $('health').textContent=(s.have_scan&&s.last_scan_age_ms<1000)?'Radar đang nhận':'Chưa có vòng quét mới';
 }catch(e){$('health').textContent='Mất kết nối / dữ liệu';}
 finally{busy=false;}
}
$('csv').onclick=()=>{
 let content='angle_deg,distance_mm\n'+bins.map((d,i)=>i+','+d).join('\n')+'\n';
 let blob=new Blob([content],{type:'text/csv;charset=utf-8'}),u=URL.createObjectURL(blob),link=document.createElement('a');
 link.href=u;link.download='delta2d_scan_'+scanSeq+'.csv';link.click();setTimeout(()=>URL.revokeObjectURL(u),1000);
};
controls();update();setInterval(update,1200);
</script></body></html>
)HTML";

void setup() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 6, false, 2);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/scan", HTTP_GET, handleScan);
  server.on("/api/sector", HTTP_GET, handleSector);
  server.begin();

  lidar->setScanPointCallback(scanPointCallback);
  lidar->setPacketCallback(packetCallback);
  lidar->setSerialReadCallback(serialReadCallback);
  lidar->setSerialWriteCallback(serialWriteCallback);
  lidar->setMotorPinCallback(motorPinCallback);
  lidar->setInfoCallback(infoCallback);
  lidar->setErrorCallback(errorCallback);

  Serial.begin(BAUD_RATE);
  Serial.setDebugOutput(false);
  Serial.setRxBufferSize(2048);
  Serial.swap(); // RX0 = GPIO13/D7, TX0 = GPIO15/D8. D8 MUST remain disconnected.
  lidar->init();
  sdkStart = (int)lidar->start();
  lastStatsAt = millis();
}

void loop() {
  lidar->loop(); // SDK parses as it reads bytes from hardware UART
  server.handleClient();
  uint32_t now = millis();
  if ((uint32_t)(now - lastStatsAt) >= 1000) {
    bytesPerSec = receivedBytes - prevBytes;
    rawPerSec = totalRaw - prevRaw;
    validPerSec = totalValid - prevValid;
    prevBytes = receivedBytes;
    prevRaw = totalRaw;
    prevValid = totalValid;
    lastStatsAt = now;
  }
  delay(1); // feed Wi-Fi and software watchdog
}
