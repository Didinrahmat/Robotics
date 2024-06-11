#define proximity 35
#define LPWMB 41  //Motor X
#define RPWMB 43
#define ENB 7

#define LPWMA 47  //Motor Y
#define RPWMA 49
#define ENA 6
#define LIM1 45
#define LIM2 37

#define LPWMV 51
#define RPWMV 53
#define ENV  5

int kondisiY = 1;  //Variabel lengan
int kondisiYb = 1;  //Variabel lengan X
int kondisiYatas = 1;
int kondisiYatasb = 1;
int count = 0;
int count2 = 0;
int count3 = 0;
int prox;
int kecMotorX = 60;

int kecMotorY = 240;  //variabel lengan Y
int limitAtas;
int limitBawah;
int kondisiX = 0;
int kondisiXb = 1;
int kondisiXatas = 0;
int kondisiXatasb = 1;
int toggle = 0;
int kecMotorV = 255;

int nextKondisi = 0;

//char pesan;
int ambil;
int buang;
int ambil2;
int buang2;
char pesan2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LPWMA, OUTPUT);
  pinMode(RPWMA, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(LIM1, INPUT_PULLUP);
  pinMode(LIM2, INPUT_PULLUP);
  pinMode(LPWMB, OUTPUT);
  pinMode(RPWMB, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(proximity, INPUT);
  pinMode(LPWMV, OUTPUT);
  pinMode(RPWMV, OUTPUT);
  pinMode(ENV, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  prox = digitalRead(proximity);
  limitAtas = digitalRead(LIM1);
  limitBawah = digitalRead(LIM2);


  //  String d_ata = Serial.readStringUntil('\n');  // Membaca data yang diterima hingga karakter newline
  //  //Serial.println("yes");  // Mengirim "yes" untuk setiap data yang diterima
  //
  //if (d_ata == "ambilBa"){
  //    bawahAmbil();
  //    Serial.println("oke");
  //  } else if (d_ata == "buangBa"){
  //    buangBawah();
  //    Serial.println("oke");
  //  }


  //Serial.println(prox);
  //Serial.println(limitAtas);
  //Serial.println(prox);
  //  skenario1();
  //sampahBawah();
  //buangSampah();
  //  bawahAmbil();
  //buangBawah();
  //  ambilSampah();
  //  buangSampah2();
  //  Serial.println(ambil);
  //  Serial.println (buang);
  if (ambil == 1) {
    bawahAmbil();
  }
  if (buang == 1) {
    buangBawah();
  }
  if (ambil2 == 1) {
    atasAmbil();
  }
  if (buang2 == 1) {
    buangAtas();
  }
  if (Serial.available() > 0) {
    prox = digitalRead(proximity);
    limitAtas = digitalRead(LIM1);
    limitBawah = digitalRead(LIM2);

    String pesan = Serial.readStringUntil('\n');

//        pesan2 = (Serial.read)();
//    
//        if (pesan2 == 'A' ) {
//          ambil = 1;
//        }
//        if (pesan2 == 'B') {
//          buang = 1;
//        }
//        if (pesan2 == 'C') {
//          ambil2 = 1;
//        }
//        if (pesan2 == 'D') {
//          buang2 = 1;
//        } if (pesan2 == 'E') {
//          normal();
//        }

////        Serial.println("baca");
        if (pesan == "A" ) {
          ambil = 1;
        }
        if (pesan == "B") {
          buang = 1;
        }
        if (pesan == "C") {
          ambil2 = 1;
        }
        if (pesan == "D") {
          buang2 = 1;
        } if (pesan == 'E') {
          normal();
        }



  }

}
