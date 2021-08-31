/*
 * Alarme_Boussier.ino
 *
 * Created: 2/13/2015 9:40:48 AM
 * Author: JOVM
 */ 

//On utilisera que le port B pour les interruption "pin change" (pin D8):
//#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts

//-------- Ajout des librairies --------//
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <Adafruit_BMP085.h>
#include <PinChangeInt.h>
/*#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>*/
//-----------------------------------------//


//-------- Declaration des broches --------//
#define FONA_RX 5
#define FONA_TX 6
#define FONA_RST 7
#define fenetre 2
#define Porte 3
#define RI 8
#define Sirene 9
#define Activation 10
#define secteur 12
#define ACT_LED 13
//-----------------------------------------//

//---Declaration des états de la machine --//
#define S_VEILLE 14
#define S_ACTIVATION 15
#define S_ACTIVEE 16
#define S_CONFIRMATION 17
#define S_DECLENCHEMENT 18
#define S_DECLENCHEE 19
#define S_CMDGSM 20
#define S_DESACTIVATION 21
#define S_SERIAL 22
#define S_SECTEUR 23
#define AUTORISATION_ENVOI 24 //En debug pour eviter les sms inutiles
//-----------------------------------------//



//-----Declaration des classes-------------//
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX); //liaison série logicielle pour le module FONA
Adafruit_FONA fona = Adafruit_FONA(FONA_RST); //Création du module FONA
Adafruit_BMP085 bmp; //Création du module BMP085
//-----------------------------------------//

//-----Declaration des variables-----------//
char replybuffer[45];
char command;
volatile int sense_fenetre = LOW;
volatile int sense_porte = LOW;
volatile int prev_state = S_VEILLE;
volatile int state = S_VEILLE;
volatile unsigned long Time = 0;
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t Activation_sms = 0;
uint8_t Act_state = 0;
uint8_t prev_act_state = 0;
uint8_t DEBUG = 0;
unsigned long Temporisation = 30000;
//-----------------------------------------//

void setup() {
  
  //-----Initialisation des broches----------//
  pinMode(Sirene, OUTPUT);
  digitalWrite(Sirene, LOW);
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);
  pinMode(Activation, INPUT_PULLUP);
  pinMode(RI, INPUT);
  pinMode(secteur, INPUT);
  pinMode(fenetre, INPUT);
  pinMode(Porte, INPUT);
  //-----------------------------------------//
  
  //-----Initialisation module GSM-----------//
  Serial.begin(9600);
  Serial.println(F("Tests de base du module FONA"));
  Serial.println(F("Initialisation....(10 secondes environ..)"));
  fonaSS.begin(4800);
  // See if the FONA is responding
  if (! fona.begin(fonaSS)) {  // make it slow so its easy to read!
    Serial.println(F("Aucun FONA detecte"));
    while (1);
  }
  delay(6000);
  if(!fona.setPBstorage(1)){
    Serial.println(F("Impossible de configurer le repertoire"));
  }
  if (fona.deleteAllSMS(4)) {
    Serial.println(F("SMS supprimes!"));
    } else {
    Serial.println(F("Suppression SMS impossible !"));
  }
  Serial.println(F("Module FONA OK"));
  //-----------------------------------------//
  
  //--------Initialisation module BMP085-----//
  if (!bmp.begin()) {
    Serial.println(F("Aucun capteur BMP085 trouve, verifiez le cablage!"));
    while (1) {}
  }
  Serial.println(F("Capteur BMP085 operationel"));
  //-----------------------------------------//
  
  //--Activation de l'interruption sur la broche Ring indicator--//
  attachPinChangeInterrupt(RI,Ring_Indicator,RISING);
  //--------------------------------------------------------------//
  
  //--Activation de l'interruption sur la broche secteur--//
  attachPinChangeInterrupt(secteur,Perte_secteur,CHANGE);
  //--------------------------------------------------------------//
  
  //-------------- Gestion etat initial Activation ---------------//
  if (digitalRead(Activation)){
    prev_act_state = 1;
    Serial.println(prev_act_state);
  }
  digitalWrite(ACT_LED,HIGH);
  delay(500);
  digitalWrite(ACT_LED,LOW);
  //--------------------------------------------------------------//

 // Serial.println(F("Init terminee"));
}

void loop() {
  
  switch(state){
    
    case S_VEILLE: //Etat de veille, on attend l'activation de l'alarme
      debug_fct();
      Act_state = digitalRead(Activation);
      if ( (Act_state != prev_act_state) || Activation_sms){
        prev_act_state = Act_state;
        Activation_sms = 1;
        state = S_ACTIVATION;
        Time = millis();
        Serial.println(F("Activation"));
      }
      break;
      
    case S_ACTIVATION: //Après temporisation, activation des interruption "porte" et "fenetre"
      debug_fct();
      if (millis() > Time + Temporisation){
        Activation_alarme();
        state = S_ACTIVEE;
        Serial.println(F("Activee"));
        envoi_sms("Activee");
      }
      break;
    
    case S_ACTIVEE: //Etat d'attente d'une interruption
      debug_fct();
      Act_state = digitalRead(Activation);
      if ( (Act_state != prev_act_state) || !Activation_sms){
        prev_act_state = Act_state;
        Activation_sms = 0;
        state = S_DESACTIVATION;
      }
      break;
      
    case S_CONFIRMATION: //Apparition d'une interruption, confirmation du declenchement
      delay(3);
      if ( !digitalRead(fenetre) || !digitalRead(Porte) )
      {
        state = S_DECLENCHEMENT;
      }
      else {
        sense_fenetre = LOW;
        sense_porte = LOW;
        Activation_alarme();
        state = S_ACTIVEE;
      }
      break;
      
    case S_DECLENCHEMENT: //Declenchement confirmé, declechement de la sirene et envoi SMS
      debug_fct();    
      if (sense_fenetre == HIGH){
        sense_fenetre = LOW;
        digitalWrite(Sirene, HIGH);
        Serial.println(F("FENETRE"));
        envoi_sms("Fenetre");
      }
      else if ( sense_porte == HIGH ) {
        sense_porte = LOW;
        Serial.println(F("PORTE"));
        delay(Temporisation);
        digitalWrite(Sirene, HIGH);
        envoi_sms("Porte");
      }
      state = S_DECLENCHEE;
      break;
      
    case S_DECLENCHEE: //Etat d'attente d'une desactivation d'alarme (Manuelle ou SMS)
      debug_fct();
      Act_state = digitalRead(Activation);
      if ( (Act_state != prev_act_state) || !Activation_sms){
        prev_act_state = Act_state;
        Activation_sms = 0;
        state = S_DESACTIVATION;
      }
      break;
      
    case S_DESACTIVATION: //La sirene est coupée
      debug_fct();
      desactivation_alarme();
      Serial.println(F("Desactivation"));
      delay(1); //Filtrage rebond activation
      envoi_sms("Desactivee");
      state = S_VEILLE;
      break;
      
    case S_CMDGSM: //Etat de gestion d'une commande GSM
      debug_fct();
      Serial.println("CMDGSM");
      uint16_t smslen;
      if (! fona.getSMSSenderPB(1, replybuffer, 40)) {
        Serial.println("orignine SMS inconnue");
        if (!fona.deleteAllSMS(4)) {
          Serial.println(F("Pb suppr SMS"));
        }
        state = prev_state;
        break;
      }
      if (!fona.readSMS(1, replybuffer, 1, &smslen)) { // pass in buffer and max len!
        Serial.println("Probleme de lecture SMS!");
        state = prev_state;
        break;
      }
      //Serial.println(replybuffer);
      command = replybuffer[0];
      gestion_cmdgsm(&command);
      if (!fona.deleteAllSMS(4)) {
        Serial.println(F("Pb suppr SMS"));
      } 
      state = prev_state;
      break;
      
    case S_SERIAL: //Etat de gestion commande console
      Serial.print(F("ALARME_BOUSSIER> "));
      while (Serial.available()){
        gestion_console();
      }       
      state = prev_state;
      break;
      
    case S_SECTEUR:
      Serial.println(F("Secteur"));
      delay(3);
      if(digitalRead(secteur)){
        envoi_sms("Perte 220V");
        state = prev_state;
        attachPinChangeInterrupt(secteur,Perte_secteur,CHANGE);
      }
      else {
        envoi_sms("220V OK");
        state = prev_state;
        attachPinChangeInterrupt(secteur,Perte_secteur,CHANGE);
      }
      break;
  }

}

void printMenu(void) {
  Serial.println(F("-------------------------------------"));
  Serial.println(F("[?] Afficher le menu"));
  Serial.println(F("[T] Regler la temporisation"));
  Serial.println(F("[A] Afficher le repertoire"));
  Serial.println(F("[E] Ajouter/Modifier une entree"));
  Serial.println(F("[D] Supprimer une entree"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));
}

void serialEvent(){
    prev_state = state;
    state = S_SERIAL;
}

void Activation_alarme(){
  attachInterrupt(0,s_fenetre,LOW);
  attachInterrupt(1,s_porte,LOW);
  digitalWrite(ACT_LED, HIGH);
}

#pragma GCC optimize ("-O0")
#pragma GCC push_options
void envoi_sms(char *Message){
  Serial.println(F("Envoi_SMS"));
  #ifdef AUTORISATION_ENVOI
  uint8_t used = fona.getPBused();
  if (used == 0) {
    Serial.println(F("repertoire vide"));
    return;
  }
  char Sent_MSG[40];
  char Time_Stamp[23];
  char Number[40];
  uint16_t Type = 0;
  char Text[14];
  fona.getTime(Time_Stamp, 23);
  strcpy(Sent_MSG, Time_Stamp);
  strcat(Sent_MSG,":");
  strcat(Sent_MSG, Message);
  Serial.println(Sent_MSG);
  for (uint8_t t = 1; t <= used; t++){
    if(!fona.ReadPBentry(t,Number,&Type,Text)){
      Serial.println(F("Lecture impossible"));
      return;
    }
    if (!fona.sendSMS(Number, Sent_MSG)) {
      Serial.println(F("Envoi SMS impossible"));
      return;
      } else {
      Serial.println(F("Envoye!"));
    } 
  }
  #endif
}
#pragma GCC pop_options

void desactivation_alarme(){
  digitalWrite(Sirene, LOW);
  digitalWrite(ACT_LED, LOW);
  detachInterrupt(0);
  detachInterrupt(1);
}

#pragma GCC optimize ("-O0")
#pragma GCC push_options
void gestion_cmdgsm(char *cmd){
  switch(*cmd){
    
    case 'T':{
      char Temperature[15];
      int32_t Temp = bmp.readTemperature_int();
      int32_t Temp_unite = Temp / 10;
      int32_t Temp_decimale = Temp % 10;
      sprintf(Temperature,"T=%d.%d *C", Temp_unite, Temp_decimale);
      //Serial.println(Temperature);
      envoi_sms(Temperature);
      break;
    }
    
    case 'A':{
      Serial.println(F("SMSACT"));
      Activation_sms = 1;
      break;
    }
    
    case 'D':{
      Serial.println(F("SMSDEACT"));
      Activation_sms = 0;
      break;
    }

    case 'S':{
      if(digitalRead(secteur)){
        envoi_sms("220V KO");
      }
      else {
        envoi_sms("220V OK");
      }
    }
  }
}
#pragma GCC pop_options


#pragma GCC optimize ("-O0")
#pragma GCC push_options
void gestion_console(){
  
  char cons_cmd = Serial.read();
  Serial.println(cons_cmd);
  switch (cons_cmd) {
    
    case '?': {
      printMenu();
      break;
    }
    
    case 'T': {
      Serial.println(F("#-----------------------------------------------------#"));
      Serial.println(F("Reglage de la tempo d'activation et declenchement porte"));
      Serial.println(F("Temporisation (secondes) ="));
      uint16_t Tempo = readnumber();
      Serial.println();
      Temporisation = Tempo * 1000;
      Serial.print(F("Temporisation reglee sur : "));
      Serial.print(Temporisation);
      Serial.println(F(" ms"));
      Serial.println(F("#-----------------------------------------------------#"));
      break;
    }
    
    case 'A': {
      Serial.println(F("#-----------------------------------------------------#"));
      Serial.println(F("Afficher le repertoire"));
      uint8_t used = fona.getPBused();
      uint8_t total = fona.getPBtotal();
      
      if (used == 0){
        Serial.println(F("Aucune entree dans le repertoire"));
        Serial.println(F("#-----------------------------------------------------#"));
        break;
      }
      Serial.println("Index ; Numero ; Type ; Nom");
      for (uint8_t i = 1;i <= used;i++){
        char Number[40];
        uint16_t Type = 0;
        char Text[14];
        if(!fona.ReadPBentry(i,Number,&Type,Text)){
          Serial.println("Lecture impossible");
          Serial.println(F("#-----------------------------------------------------#"));
          break;
        }
        Serial.print(i);
        Serial.print(F(" ; "));
        Serial.print(Number);
        Serial.print(F(" ; "));
        Serial.print(Type);
        Serial.print(F(" ; "));
        Serial.println(Text);
      }
      Serial.print(used);
      Serial.print(F(" utilisee(s) / "));
      Serial.println(total);
      Serial.println(F("#-----------------------------------------------------#"));
      break;
    }
    
    case 'E': {
      Serial.println(F("#-----------------------------------------------------#"));
      Serial.println(F("Ajouter/Modifier une entree"));
      char Number[40];
      char Text[14];
      uint8_t total = fona.getPBtotal();
      Serial.print(F("Index (1-"));
      Serial.print(total);
      Serial.print(F(") (0 pour ajouter au premier index dispo.) :"));
      uint8_t Index = readnumber();
      Serial.println();
      Serial.print(F("Type (129 format local 0XXXXXXXXX / 145 format international +336XXXXXXXX) :"));
      uint8_t Type = readnumber();
      Serial.println();
      Serial.print(F("Numero (selon format choisi) :"));
      readline(Number, 40);
      Serial.println(Number);
      Serial.print(F("Texte associe (14 caracteres max.) :"));
      readline(Text, 14);
      Serial.println(Text);
      if(!fona.WritePBentry(Index,Number,Type,Text)){
        Serial.println(F("Ecriture impossible"));
        Serial.println(F("#-----------------------------------------------------#"));
        break;
      }
      Serial.println(F("#-----------------------------------------------------#"));
      break;
    }
    
    case 'D': {
      Serial.println(F("#-----------------------------------------------------#"));
      Serial.println(F("Supprimer une entree"));
      uint8_t total = fona.getPBtotal();
      Serial.print(F("Index de lentree a detruire (1-"));
      Serial.print(total);
      Serial.println(F(") :"));
      uint8_t Index = readnumber();
      Serial.println();
      if(!fona.DeletePBentry(Index)){
        Serial.println(F("Supression impossible"));
        Serial.println(F("#-----------------------------------------------------#"));
        break;
      }
      Serial.println(F("Entree supprimee"));
      Serial.println(F("#-----------------------------------------------------#"));
      break;
    }
        
    default: {
      Serial.println(F("Commande inconnue"));
      printMenu();
      break;
    }
  
    }
    flushSerial();
}
#pragma GCC pop_options

void flushSerial() {
  while (Serial.available())
  Serial.read();
}
 
void s_fenetre(){
    sense_fenetre = HIGH;
    state = S_CONFIRMATION;
    detachInterrupt(0);
}

void s_porte(){
    sense_porte = HIGH;
    state = S_CONFIRMATION;
    detachInterrupt(1);
}

void Ring_Indicator(){
  prev_state = state;
  state = S_CMDGSM;
}

void Perte_secteur(){
  prev_state = state;
  state = S_SECTEUR;
  detachPinChangeInterrupt(secteur);
}

void debug_fct(){
  if (DEBUG == 1){
    Serial.print(F("Etat de la machine:"));
    Serial.println(state);
  }
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}

uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;
  
  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while(Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
        continue;
        
        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }
    
    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
