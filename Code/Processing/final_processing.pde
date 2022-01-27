import org.openkinect.freenect.*;
import org.openkinect.freenect2.*;
import org.openkinect.processing.*;
import org.openkinect.tests.*;
import processing.serial.*;
import blobDetection.*;

public static final char HEADER = '|';
public static final char Xc = 'N';
public static final char Yc = 'M';

Kinect kinect;
BlobDetection blobDetector;  // Create a blob Detector
Serial myPort;

boolean mirror = false;
boolean medFilter = false; 
boolean irState = false;    // Use IR camera

PImage currentImage, redImage, depthImage;  // Images to display  //An initial image to hold blobs

int threshold = 120;
int filterSize = 2; 
int[] depth;
float deg;
float avgDepth;
float depthSum = 0;
int avgOffset;
long avgMappedX;
long avgMappedY;
float avgX;
float avgY;
float fX;
float fY;
float realX;
float realY;
float alphaX;
float alphaY;
boolean NaN = false;
float sumX = 0;
float sumY = 0;
int totalPixels = 0;

void setup() {
  // Impostare dimensioni dell'area di lavoro
  size(2140, 1100);
  
  // Nome della porta seriale
  myPort = new Serial(this, "COM5", 9600);
  
  // Inizializzare kinect e le diverse immagini
  kinect = new Kinect(this);
  kinect.initVideo();
  kinect.initDepth();
  
  // Visualizzare immagine 
  currentImage = kinect.getVideoImage();
  depthImage = kinect.getDepthImage();
  
  // Creare un'immagine filtrata (pixel rossi)
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  image(redImage, 640, 0);
}

void draw() {
  stroke(255); 
  background(0);
  currentImage = kinect.getVideoImage();
  image(depthImage, 850, 0);
  image(currentImage.get(), 1500, 0);

  // Creare un'immagine filtrata (pixel rossi)
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  
  //median filtering
   if(medFilter) {
    redImage = medianFilter(redImage, filterSize);
  }
 
  image(redImage, 850, 480);
  fill(255);
  
  // Ricavare le coordinate e inserirle in un vettore
  PVector v = depthToWorld(int(avgX), int(avgY), int(avgDepth)); 
  
  // Inviare coordinate via seriale
  sendMessage(Xc, int(v.x*100));
  sendMessage(Yc, int(v.z*100));
    

  textSize(26);
  text("fps: " + int(frameRate), 10, 30);
  text("avg x: " + int(avgX) + "  avg Y: " + int(avgY) + "\n" + "\n"  + 
      "Coordinates - x: " + floor(v.x*100) + "cm" + ", y: " + floor(v.z*100) +"cm" + "\n" + "\n" + 
      "threshold: " + int(threshold)  + "\n" + "\n" + "totalPixels: " + totalPixels , 1500, 600 );

/* =========================
   Grafico
   =========================*/
  float x;
  float y;
  if(NaN){
    x = 0;
    y = 100;
  } else {
    x = map(v.x*100, 0, 143, 0, 382);
    y = map(v.z*100, 0, 300, 0, 800);
  } 

  x = 400 + int(x);
  y = 900 - int(y);

  fill(0);
  stroke(150);
  arc(400, 900, 1600, 1600, PI + HALF_PI - 0.4974188, PI + HALF_PI + 0.4974188, PIE);
  fill(0, 199, 255);
  rect(300,900,200,50);
  stroke(100);
  line(400, 900, 400, 0); //vertical
  line(0, 900, 800, 900);
  stroke(0, 199, 255);
  line(390, 100, 410, 100);
  text("(0cm ;300cm)", 420, 100);
  text("(0;0)", 420, 890);
  fill(148, 255, 0);
  textSize(18);
  ellipse(x, y, 25, 25);
    if(NaN){
    text("Robot out of view", x+25, y + 10);
  } else {
    text("(" + floor(v.x*100) + "cm; " +  floor(v.z*100) + "cm)", x+25, y + 10);
  } 
}

// Evento seriale
void serialEvent(Serial p) {
  // handle incoming serial data
  String inString = myPort.readStringUntil('\n');
  if(inString != null) {
  println( inString ); // echo text string from Arduino
  }
}

void sendMessage(int tag, int value){
  // send the given index and value to the serial port
  myPort.write(HEADER); //mandare un header in modo che l'arduino sappia che stia arrivando un nuovo valore
  myPort.write(tag); //mandare un tag che identifica il valore
  
  char c = (char)(abs(value) / 256); // msb valore assoluto per correttezza di calcoli con valori negativi
  myPort.write(c);                   // convertire i valori ascii in valori numerici
  c = (char)(abs(value) & 0xff); // lsb
  myPort.write(c); 
  
  if(value < 0) { 
    myPort.write('n'); // Verificare se numero positivo o negativo
  } else {
    myPort.write('p');
  }
  //se non trova abbastanza pixels   
  if(totalPixels < 150 || (tag == Yc && value < 50)){ 
    NaN = true;
    myPort.write('x');
  } else {
    NaN = false;
    myPort.write('y');
  }
}

// convert to radians
float toRadians(float deg) {
   double radians = deg * 3.14159265359 / 180;
   return (float) radians;
}

//convert to deg
float toDeg(float rad) {
   double deg = rad * 180 / 3.14159265359;
   return (float) deg;
}

// le tre funzioni per ricavare l'intensita` di un colore di un pixel

//calculate red channel intensity
float calcRedVal(color c) {
   return  c >> 16 & 0xFF - (c >> 16 & 0xFF + c >> 8 & 0xFF + c & 0xFF) / 3; //RED
}
//calculate green channel intensity
float calcGreenVal(color c) {
   return  c >> 8 & 0xFF - (c >> 16 & 0xFF + c >> 8 & 0xFF + c & 0xFF) / 3; //GREEN
}
//calculate blue channel intensity
float calcBlueVal(color c) {
   return  c >> 0xFF - (c >> 16 & 0xFF + c >> 8 & 0xFF + c & 0xFF) / 3; //GREEN
}

// Ottenere intensita` del colore rosso in ogni pixel dell'immagine RGB -- check calcRedChannel()
  PImage redThreshold(PImage img, int threshold){
  img.loadPixels();
  depth = kinect.getRawDepth();
  
  sumX = 0;
  sumY = 0;
  totalPixels = 0;
  
  // Step through every pixel and see if the redness > threshold
  for (int x = 0; x < img.width; x += 1) {
    for (int y = 0; y < img.height; y +=1) {
    
      int offset = x + y * img.width;
    
      if (calcRedVal(img.pixels[offset]) >= threshold ) { 
        // Set pixel to white
        img.pixels[offset] = color(255, 255, 255);
        
        sumX += x;
        sumY += y;
        totalPixels++;

      } else {
        // Set pixel to black
        img.pixels[offset] = color(0, 0, 0);
      }
    }
  }
  img.updatePixels();
  
  avgX = 1.055 * (sumX / totalPixels);
  avgY = 0.94  * (sumY / totalPixels);
  avgMappedX = map((long) avgX,0,1280,0,1280);
  avgMappedY = map((long) avgY,0,960,0,960);
  avgOffset = int(avgMappedX + avgMappedY * depthImage.width);
  //avgDepth = depth[avgOffset];
  avgDepth = depth[avgOffset];
  
  fill(150,255,0);
  ellipse(avgX + 850, avgY, 8, 8);
  //fill(150, 255, 0);
  //ellipse(avgMappedX, avgMappedY, 30, 30);

  return img;
}

//convert raw depth data to meters
float rawDepthToMeters(float depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Convertire i valori del Kinect in valori reali (metri)
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  // Impostare il vettore in modo da contenere le tre coordinate dello spazio fisico
  PVector result = new PVector();
  double depth =  rawDepthToMeters(depthValue);//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

//map function for java
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



// Median filter
PImage medianFilter(PImage img, int size) { 
  img.loadPixels(); 
  PImage temp;
  int[] tempPixels = img.get().pixels;
  
  // Step through every pixel in the image that is not a border pixel
  for(int y = size; y < img.height - size; y++) {
    for(int x = size; x < img.width - size; x++) {
      // Get a block of pixels of size (2*size+1)^2 around each 
      temp = img.get(x-size, y-size, 2*size+1, 2*size+1);
      // Find the median element
      tempPixels[y * img.width + x] = sort(temp.pixels)[(2*(2*size+1)-1) / 2];
    } 
  }
  
  // Update the pixels in the image
  img.pixels = tempPixels;
  img.updatePixels();
  return img; 
}

// Mappatura tasti per diversi comandi
void keyPressed() {
 if(key == 'h'){
    mirror = !mirror;
    kinect.enableMirror(mirror); // enable mirror mode
 } else if (key == 'w') { 
   //blobDetector.setThreshold(threshold / 255.0);
   threshold++; // turn up threshold
 } else if (key == 's') { 
   //blobDetector.setThreshold(threshold / 255.0);
   threshold--; // turn down threshold
 }else if (key  == 'i') {
   irState = !irState; // toggle ir state, useful when kinect freezes
   kinect.enableIR(irState);
 } else if (key == 'm') {
   medFilter = !medFilter; // toggle med filter
 } else if (key == CODED) {
    if (keyCode == UP) {
      deg++; // tilt kinect upwards
    } else if (keyCode == DOWN) {
      deg--; // tilt kinect downwards
    }
    deg = constrain(deg, 0, 30);
    kinect.setTilt(deg);
  }
}