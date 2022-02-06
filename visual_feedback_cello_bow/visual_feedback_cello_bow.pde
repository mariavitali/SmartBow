import oscP5.*;
import netP5.*;
import processing.serial.*;

// Serial communication to teensy
Serial teensyPort;                 // Serial port number to read from
static String data;    // IMUs x angles difference. Data from teensy.
float angle = 0;


// get data from pure data
OscP5 oscP5;
NetAddress myRemoteLocation;
float freq = 0;


//bow shape
float x_bow;
float y_bow;
float h = 10; 
float l = 2000;

//rotation
float x_trans;
float y_trans;

//rotation centre
float x_c;
float y_c;

// color
int c = 200;

// visual cue to display error feedback
Cue Cue_01;

// analysis of pure data input (understand which string is playing)
CelloString Current_string;


PShape bot;
PShape f;


/* print data on externatl file */
// error/correct counter
int limit_angle = 15;  
boolean wrong_angle = false;
float error_duration;
float correct_duration;

//txt file with error duration
PrintWriter output;

void keyPressed() {
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
  exit(); // Stops the program
}


void setup() {
  size(1280, 720);

  x_bow = width/2 - l/2;
  y_bow = height/2 - h/2;


  // define the point of rotation (in this case, setted in the middle of the canva)
  x_c = width/2;
  y_c = height/2;

  bot = loadShape("hand_03.svg");
  f = loadShape("cello_f.svg");
  
  // Serial port information to communicate with teensy
  String portName = "/dev/ttyACM0";
  teensyPort = new Serial(this, portName);
  
  // oscP5 to communicate with pure data
  oscP5 = new OscP5(this, 32000);
  myRemoteLocation = new NetAddress("127.0.0.1", 12000);
  
  // initialize visual cue
  Cue_01 = new Cue();

  // inizialize object to communicate with pure data
  Current_string = new CelloString();
  oscP5.plug(this, "receive_freq", "/freq");

  
  // Create a new file in the sketch directory
  output = createWriter("error_duration.txt");
  
}

void draw() {

  // telling which string is currently played
  Current_string.SetString(freq);
  
  if( teensyPort.available() > 0 ) {    // if data is available
    data = teensyPort.readStringUntil('\n');
    String[] s = splitTokens(data, " ");
    if (s[0].equals("angleDifference") == true){
      try {
         angle = float(trim(s[1])) * (-1);
      }
      catch(Exception e) {
        println("Error in converting value"); 
      }
    }
  }
  
  //check if the bow's angle is higher/lower than a certain angle
  if (angle >= limit_angle 
    || angle <= -limit_angle) {
    wrong_angle = true;
  } else {
    wrong_angle = false;
  }
  
  // check how much time the bow is kept wrong
  if (wrong_angle) {
    error_duration = error_duration + 1;
  } else if (!wrong_angle) {
    correct_duration = correct_duration + 1;
  }
  
  background(255);

  //Alfa is our angle expressed in radians
  float alfa = radians(angle);

  float raggio = sqrt(pow(x_c, 2) + pow(y_c, 2));

  //we need beta (if not, when the angle is 0 "x_trans" is equal to "x_c - raggio" instead of 0)
  float beta = acos(x_c/raggio); 

  //shape translation (Processing rotate the following shapes starting from the canva's origin)
  x_trans = x_c - (cos(alfa + beta)* raggio);
  y_trans = y_c - (sin(alfa + beta)* raggio);


  //angle feedback
  noFill();
  stroke(200);
  line(0, height/2, width, height/2);

  if (alfa >= 0) {
    arc(x_c, y_c, 250, 250, 0, alfa);
  } else {
    alfa = -alfa;
    arc(x_c, y_c, 250, 250, 2*PI -alfa, 2*PI);
    alfa = -alfa;
  }

  //cello's "f"
  f.disableStyle();
  fill(200);
  noStroke();
  shape(f, x_c - 250, y_c - 150, 500, 300);


  //it's important to put "translate" before "rotate"
  translate( x_trans, y_trans);
  rotate(alfa);

  // the shape representing the bow
  stroke(0);
  fill(255);
  rect(x_bow, y_bow, l, h);

  // to draw again static things, we need to neutralize the previous changement
  rotate(- alfa);
  translate( - x_trans, - y_trans);

  // the hand of the cellist
  // this is an arbitrary value. The hand is going up and down following this x coordinate
  float x_mano = 1040;

  // this value depends on the centre of rotation of the bow
  float y_mano = y_c + sin(alfa)*((x_mano - x_c)/(cos(alfa)));

  fill(255);
  shape(bot, x_mano - 75, y_mano - 100, 150, 200);


  //blinking cue
  if (wrong_angle) {
    Cue_01.display(4);
  }

  //check some values
  println("----------");
  println("Current angle: ", angle);
  println("Error condition: ", wrong_angle);
  println("Error duration (s): ", error_duration / 50);
  println("Correct duration (s): ", correct_duration / 50);

  output.println("----------");
  output.println("Current angle: " + angle);
  output.println("Error condition: " + wrong_angle);
  output.println("Error duration (s): " + error_duration / 50);
  output.println("Correct duration (s): " + correct_duration / 50);
}





void receive_freq(float value){
  print("value received from pd: ");
  println(value);
  freq = value;
}





/*********************************************************************************/

class CelloString {

  //strings frequencies
  int string_01 = 220;
  int string_02 = 146;
  int string_03 = 98;
  int string_04 = 65;

  int range = 5;

  String playing;

  void SetString ( float hz) {

    if (hz > (string_01 - range) && hz < (string_01 + range)) {
      playing = "String 1 (A)";
    } else if (hz > (string_02 - range) && hz < (string_02 + range)) {
      playing = "String 2 (D)";
    } else if (hz > (string_03 - range) && hz < (string_03 + range)) {
      playing = "String 3 (G)";
    } else if (hz > (string_04 - range) && hz < (string_04 + range)) {
      playing = "String 4 (C)";
    } else {
      playing = "None";
    }

    textSize(20);
    textAlign(CENTER);
    fill(0);
    text("You're playing " + playing, width/2, height - 30);
  }
}



class Cue {

  int l = 30;
  int t = 0;

  void display(int frequency) {

    int n = 50/ (2*frequency);
    t++;

    if (t < n) {
      noStroke();
      fill(255, 0, 0);
      rect(0, 0, l, height);
      rect(width - l, 0, l, height);
    } else if (t == (2*n)) {
      t = 0;
    }
  }
}
