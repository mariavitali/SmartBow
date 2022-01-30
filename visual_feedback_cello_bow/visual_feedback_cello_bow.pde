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

//color
int c = 200;


PShape bot;
PShape f;

void setup() {
  size(1280, 720);

  x_bow = width/2 - l/2;
  y_bow = height/2 - h/2;


  // define the point of rotation (in this case, setted in the middle of the canva)
  x_c = width/2;
  y_c = height/2;

  bot = loadShape("hand_03.svg");
  f = loadShape("cello_f.svg");
}

void draw() {

  //mouse position sets the degree
  int angle = mouseX - 100;
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


  //check some values
  println("----------");
  println("angle:", angle);
  println("raggio:", raggio);
}
