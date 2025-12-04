// =============================================================================
//       FINAL COMBINED SENSOR GUI - NO MMWAVE DISTANCE
// =============================================================================
import processing.serial.*;

Serial myPort;
int[] gateSignals = new int[9];
int ultrasonicDistance = 0;
float smoothUltrasonic = 0;
float[] smoothGates = new float[9];

color bgColor = color(8, 12, 25);
color gridColor = color(0, 40, 80, 60);
color accentColor = color(0, 220, 255);
color textGlow = color(0, 180, 255, 120);

void setup() {
  size(1280, 720);
  println(Serial.list());
  // !!! CHANGE "COM_PORT_HERE" to your Arduino Nano's actual COM port !!!
  myPort = new Serial(this, "COM4", 115200);
  myPort.bufferUntil('\n');
  textFont(createFont("Consolas", 18));
  smooth();
}

void draw() {
  background(bgColor);
  drawGrid();
  drawHeader();
  drawUltrasonic();
  drawRadar();
}

void serialEvent(Serial p) {
  String received = trim(p.readStringUntil('\n'));
  if (received != null) {
    String[] parts = split(received, ',');
    // Now we expect 10 parts: 9 gates + 1 ultrasonic
    if (parts.length == 10) {
      try {
        for (int i = 0; i < 9; i++) gateSignals[i] = int(parts[i]);
        ultrasonicDistance = int(parts[9]);
      } catch (Exception e) { /* Ignore errors */ }
    }
  }
}

void drawUltrasonic() {
  float cx = width / 4;
  float cy = height / 2 + 50;
  smoothUltrasonic = lerp(smoothUltrasonic, ultrasonicDistance, 0.1);

  fill(accentColor);
  textAlign(CENTER);
  textSize(26);
  text("ULTRASONIC OBSTACLE SENSOR", cx, 96);
  
  noFill();
  strokeWeight(3);
  for (int i = 1; i <= 3; i++) {
    float radius = i * 200;
    color ringColor = (smoothUltrasonic < i * 70 && smoothUltrasonic > 0) ? lerpColor(color(0,255,0), color(255,0,0), i/3.0) : color(20, 60, 60);
    stroke(ringColor);
    ellipse(cx, cy, radius, radius);
  }
  
  textSize(80);
  fill(255);
  if (ultrasonicDistance > 0) {
    text(int(smoothUltrasonic) + " cm", cx, cy);
  } else {
    fill(100);
    textSize(40);
    text("CLEAR", cx, cy);
  }
}

void drawRadar() {
  float cx = width * 3 / 4;
  fill(accentColor);
  textAlign(CENTER);
  textSize(26);
  text("MMWAVE PROXIMITY GATES", cx, 96);

  // Removed the distance display from here

  int n = 9;
  int barWidth = (width/2 - 120) / n;
  int baseX = width/2 + 60;
  int baseY = height - 100;
  
  for (int i = 0; i < n; i++) {
    smoothGates[i] = lerp(smoothGates[i], gateSignals[i], 0.2);
    float val = constrain(smoothGates[i], 0, 100);
    float h = map(val, 0, 100, 0, height - 250);
    float x = baseX + i * barWidth;
    color barColor = lerpColor(color(0,255,100), color(255,50,0), val/100.0);
    noStroke();
    fill(barColor, 220);
    rect(x, baseY, barWidth - 8, -h, 6);
    fill(255);
    textSize(14);
    textAlign(CENTER);
    text("G" + i, x + barWidth/2 - 4, baseY + 20);
    text(int(val), x + barWidth/2 - 4, baseY - h - 15);
  }
}

void drawHeader() {
  fill(accentColor);
  textAlign(LEFT, TOP);
  textSize(32);
  text("MECANUM ROVER - SENSOR TELEMETRY", 40, 25);
  
  stroke(accentColor, 150);
  strokeWeight(2);
  line(40, 70, width - 40, 70);
  line(width/2, 90, width/2, height - 60);
  
  noStroke();
  fill(textGlow);
  rect(40, 70, width - 80, 2);
}
void drawGrid() {
  stroke(gridColor);
  strokeWeight(1);
  for (int i = 0; i < width; i += 40) line(i, 0, i, height);
  for (int j = 0; j < height; j += 40) line(0, j, width, j);
}
