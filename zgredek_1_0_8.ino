/*
 * Created: 2016.11.29
 * Author: witek, SP3JDZ
 * na bazie mojej wersji złomka
 *
CHANGELOG
2018.11.17 - v.1.0.8a
	- konfigurowalny adres PCF8574 lub PCF8574A (PCF_ADDRESS w pliku zgredek.h)
2017.04.30 - v.1.0.8
	- nowe filtry pasmowe (10 sztuk)
		- sterowanie kodem BCD z PCF8574A w VFO
		- 5 MHz
2017.04.21 - v.1.0.7
	- przejściowe dopasowanie do nowej płyty głównej Gracika
		- filtr 9 MHz (PP9)
		- obsługa klawisza włączenia wzmacniacza w.cz.
		- uruchomienie generatora CW (CWO - CLK2 w Si5351)
		- obsługa portu sztorc PB0
		- 4 pasma z PCF8574A
2017.03.06 - v.1.0.6
	- split
		- poprawienie zrównania częstotliwości VFO A i B (A=B) w trybach SSB
2017.02.28 - v.1.0.5
	- porządki
2017.02.25 - v.1.0.4
	- opcja wyłączania BFO (tylko same VFO)
	- wyłączenie wyjścia kluczowanie (nieobsługiwane)
		- sztorc także nieobsługiwany
2017.02.11 - v.1.0.3
	- dopasowanie kodu do nowego schematu i fabrycznej płytki
2016.12.15 - v.1.0.2
	- włączenie i obsługa BFO - zrobione
2016.12.13 - v.1.0.1
	- zmiana obsługi klawiatury z analogowej na cyfrową
	- usunięcie klucza
2016.11.29 - v.1.0.0
	- praca nad schematem
	- zmiana generatora na Si5351
TODO
	- kluczowanie CWO przestało działać (generator nie generuje)
	- CAT
	- klucz telegraficzny
 */
#define software_version "1.0.8a"
#define eeprom_magic_number 22
#define VFO			0
#define BFO			1
#define CWO			2
#include <EEPROM.h>
#include <TimerOne.h>
#include <LCD5110_Graph.h>
#include <PCF8574.h>
#include <PCF8575.h>
#include <Wire.h>
#include <Bounce2.h>
#include "zgredek.h"
PCF8574 expander;
PCF8575 pa;
Bounce krok = Bounce();
Bounce rit = Bounce();
Bounce split = Bounce();
Bounce tryb = Bounce();
Bounce ptt = Bounce();
Bounce sztorc = Bounce();

// inicjalizujemy wyświetlacz
// lcd    - arduino
// sclk   - PIN 2
// sdin   - PIN 4
// dc     - PIN 5
// reset  - PIN 7 - nieużywany; do procesora tylko 4 linie
// sce    - PIN 6
LCD5110 myGLCD(2,4,5,7,6);	// port nr 7 - atrapa - ustawiana na wysoki...
extern uint8_t TinyFont[];       //czcionka z biblioteki.....dodałem małe fonty (SP6IFN)
extern uint8_t SmallFont[];      //czcionka z biblioteki
extern uint8_t MediumNumbers[];  //czcionka z biblioteki

//inicjalizujemy enkoder
//A0 - w lewo
//A1 - w prawo
//kondensatory na wejściach należy usunąć
int8_t enc_delta;							// -128 ... 127
// funkcja sprawdzająca stan wejść od enkodera (co 1 ms)
void encode_read()
{
	static int8_t last;
	int8_t nowy;
	int8_t diff;
	nowy = 0;
	if (digitalRead(A0) == LOW)
	nowy = 3;
	if (digitalRead(A1) == LOW)
	nowy ^= 1;								// convert gray to binary
	diff = last - nowy;						// difference last - nowy
	if( diff & 1 ){							// bit 0 = value (1)
		last = nowy;							// store nowy as next last
		enc_delta += (diff & 2) - 1;		// bit 1 = direction (+/-)
	}
}
int8_t encode_read1( void )					// read single step encoders
{
	int8_t val;
	noInterrupts();
	val = enc_delta;
	enc_delta = 0;
	interrupts();
	return val;								// counts since last call
}
int8_t encode_read2( void )					// read two step encoders
{
	int8_t val;
	noInterrupts();
	val = enc_delta;
	enc_delta &= 1;
	interrupts();
	return val >> 1;
}
int8_t encode_read4( void )					// read four step encoders; funkcja dla enkodera kwadraturowego
{
	int8_t val;
	noInterrupts();
	val = enc_delta;
	enc_delta &= 3;
	interrupts();
	return val >> 2;
}

//*****************************************************************************************************************************
//stałe do modyfikacji każdy ustawia to co potrzebuje
// porty do sterowania FPP (CD4028):
#define PORT_A		5
#define PORT_B		4
#define PORT_C		3
#define PORT_D		2
/*
#define PORT_160M		2	// port w PCF8574 dla pasma 1,8MHz NC
#define PORT_80M		2	// port w PCF8574 dla pasma 3,5 MHz PCF8574
#define PORT_40M		3	// port w PCF8574 dla pasma 7 MHz PCF8574
#define PORT_30M		4	// port w PCF8574 dla pasma 10 MHz PCF8574
#define PORT_20M		5	// port w PCF8574 dla pasma 14 MHz PCF8574
#define PORT_17M		5	// port w atmedze dla pasma 18 MHz NC
#define PORT_15M		5	// port w atmedze dla pasma 21 MHz NC
#define PORT_12M		5	// port w PCF8574 dla pasma 24 MHz NC
#define PORT_10M		5	// port w PCF8574 dla pasma 28 MHz NC
*/

// porty w PCF8575 w PA:
#define PA_160M		6	
#define PA_80M		5
#define PA_60M		4
#define PA_40M		4
#define PA_30M		3
#define PA_20M		3
#define PA_17M		2
#define PA_15M		2
#define PA_12M		1
#define PA_10M		1
#define PA_6M		0
#define PA_NO		8
#define PA_05		12	
#define PA_1		11
#define PA_2		10
#define PA_4		9

const int swr_pad = A2;                               	// wejście dla napięcia z fali padającej
const int swr_odb = A6;                               	// wejście dla napięcia z fali odbitej
const int s_metr_port = A7;                           	//wejście dla s-metra
// const int ptt_input = 12;                             	//wejście PTT procesor musi wiedzieć czy nadajemy czy odbieramy by zrealizować RIT-a
const int AMP_on_port = 7;
//const int dit_input = 0;			      				// wejście dla kropek z manipulatora
//const int dah_input = 1;								// wejście dla kresek z manipulatora
//const int key_output = 8;								// wyjście kluczujące nadajnika
//const int sound_out = 13;								// monitor - podsłuch klucza telegraficznego
const int ssb_cw_out = 3;								// wyjście przełącznika SSB/CW
const int filtr_CW_port = 6;
const byte NO_out = 7;
const int VFO_A_port = 0;
const int VFO_B_port = 1;
//const int sound_freq = 700;								// częstotliwość podsłuchu
const int contrast = 70;                              	//kontrast wyświetlacza
long frequency;
//const long if_frequency = 10694950;                   	// częstotliwość środkowa filtru kwarcowego
//const int IFoffset = 1990;								// odstęp BFO od częstotliwości środkowej filtru kwarcowego
const int CWoffset = 700;  								//w Hz
const int mode = 0;                                   	//tryby pracy: 0-pośrednia, 1-generator, 2-lub wyżej, mnożnik razy 2 lub więcej
long step_value = 10;                               	//domyślny krok syntezy
const unsigned long s_metr_update_interval = 100;     	//interwał odświeżania s-metra w msec
const long rit_range = 2000;                          	//zakres pracy RIT +/- podana wartość, domyślnie 2000Hz max 9999Hz jeśli dasz wiecej posypie się wyświetlanie
const long rit_step = 10;                             	//krok działania RIT-a domyślnie 50Hz
const byte bod_lvl = 1;                                	//konfiguracja napięcia odniesienia 0 - 5V, 1 - 1.1V
//byte dit_time_ms = 55;									// czas kropki w ms (1200/(ilość grup))
//boolean dit_flg = false;
boolean flag = false;
boolean jest_PA = false;
boolean sztorc_on = false;
boolean last_sztorc_state = false;
//*****************************************************************************************************************************
//zmienne wewnętrzne pomocnicze, czyli zmienne które są nadpisywane automatycznie w trakcie działania programu
//jeśli nie trzeba proszę ich nie modyfikować.
char buffor[] = "              ";              //zmienna pomocnicza do wyrzucania danych na lcd
int enc_sum = 0;                               //zmienna pomocnicza do liczenia impulsów z enkodera
unsigned long s_metr_update_time = 0;          //zmienna pomocnicza do przechowywania czasu następnego uruchomienia s-metra
long frequency_to_dds = 0;                     //zmienna pomocnicza przechowuje częstotliwość którą wysyłam do DDS-a
byte rit_state = 0;                             //stan RIT-a 0-rit off, 1-rit on, 2-rit on enkoder odchyłkę rit
boolean ptt_on = false;                        //stan przycisku PTT
// boolean strojenie_on = false;					// stan przycisku strojenie
boolean ssb_on = false;							// stan przełącznika SSB/CW
boolean last_ptt_state = false;                //poprzedni stan PTT potrzebne do wykrywania zmianu stanu PTT
long rit_frequency = 0;                        //domyślna wartość poprawki
/*
boolean key_mode = false;						// wskaźnik przejścia na pracę kluczem telegraficznym
*/
boolean jest_opis_smetra = false;				// czy jest opis smetra
boolean filtr_CW_on = false;					// wskaźnik, czy jest włączony filtr CW
boolean split_on = false;						// wskaźnik przejścia na tryb pracy w splicie
boolean qrp_on = false;							// wskaźnik do pracy QRP (przekaźnik N/O w PA nie będzie sterowany)
byte AMP_state = 1;							// wskaźnik włączenia wzmacniacza i tłumika
byte tryb_pracy = 0;							// tryby pracy: 0 - CW, 1 - CW VOX, 2 - LSB, 3 - USB
//unsigned long VOX_time = 0;					// zmienna do odmierzania czasu od ostatniej kreski lub kropki na potrzeby VOX CW
byte active_vfo = 0;                             //0 - vfo A, 1 - vfo B
// zmienne do pasm
const byte ile_pasm = 10;
typedef struct {
  long lower_limit;
  long upper_limit;
  long mid;
  byte moc;
} band_t;

typedef struct {
  byte band;
  long freq[ile_pasm];
  byte mode[ile_pasm];
} vfo_t;

band_t bands[ile_pasm] = {
  {1798000, 2002000, 1815000, 0}, 
  {3498000, 3802000, 3722000, 0},
  {5350000, 5368000, 5357000, 0},
  {6998000, 7202000, 7015000, 0},
  {10098000, 10152000, 10101000, 0},
  {13998000, 14352000, 14015000, 0},
  {18066000, 18170000, 18070000, 0},
  {20998000, 21452000, 21015000, 0},
  {24888000, 24992000, 24900000, 0},
  {27998000, 29702000, 28015000, 0}
};
struct config_t {
	//long if_frequency = 10694950;                   	// częstotliwość środkowa filtru kwarcowego u mnie poprzednia
	//long if_frequency = 9000027L;                   	// aktualna częstotliwość środkowa filtru kwarcowego (PP9)
	//long if_frequency = 9000400L;                   	// aktualna częstotliwość środkowa filtru kwarcowego (drabinka CW)
	long if_frequency = 9000000L;                   	// aktualna częstotliwość środkowa filtru kwarcowego - standard
	uint8_t status_pasma[ile_pasm] = {1, 1, 1, 1, 2, 1, 1, 1, 1, 1};		// 0 - wyłączone; 1 - włączone; 2 - włączone i domyślne (włącza się po załączeniu zasilania)
	int mode = 0;
	//unsigned int dit_time_ms = 80;
	unsigned int bk_time = 500;
	//long si_rezonator = 25025000L;	// częstotliwość kwarcu w Si5351
	//long si_rezonator = 27003110L;	// częstotliwość kwarcu w Si5351
	long si_rezonator = 27004060L;	// częstotliwość kwarcu w Si5351
	//int IFoffset = 1990;			// odstęp BFO od częstotliwości środkowej filtru kwarcowego
	//int IFoffset = 1787;			// odstęp BFO od częstotliwości środkowej filtru kwarcowego - 600Hz względem spadku o 6dB
	//int IFoffset = 650;			// odstęp BFO od częstotliwości środkowej filtru kwarcowego - 650Hz względem środka filtru CW
	int IFoffset = 1500;			// odstęp BFO od częstotliwości środkowej filtru kwarcowego - standard
} configuration ;

byte modes[ile_pasm]={0, 2, 0, 2, 0, 3, 3, 3, 3, 3};
byte prev_port_TRX = 0;	// port w TRX przez zmianą pasma
byte prev_port_PA = 0; 	// port w końcówce przed zmianą pasma
int8_t moc = 0; // umowny poziom mocy we wzmacniaczu mocy od 0 do 15
 
vfo_t vfo[2];
byte config_dirty = 0;


//*****************************************************************************************************************************

//FUNKCJE
//funkcja do obsługi wyświetlania zmiany częstotliwości
void show_frequency(){
  if(rit_state != 1){                              //Jeśli Enkoder pracuje jako VFO zaoszczędzimy trochę czasu procesora jesli enkoder pracuje jako RIT
    long f_prefix = frequency/1000;                //pierwsza część częstotliwości dużymi literkami
    long f_sufix = frequency%1000;                 //obliczamy resztę z częstotliwości
    sprintf(buffor,"%05lu",f_prefix);              //konwersja danych do wyświetlenia (ładujemy częstotliwość do stringa i na ekran)
    myGLCD.setFont(MediumNumbers);                 //ustawiamy czcionkę dla dużych cyfr
    myGLCD.print(buffor,1,13);                     //wyświetlamy duże cyfry na lcd
    sprintf(buffor,".%03lu",f_sufix);              //konwersja danych do wyświetlenia (ładujemy częstotliwość do stringa i na ekran)
    myGLCD.setFont(SmallFont);                     //ustawiamy małą czcionkę
    myGLCD.print(buffor,60,22);                    //wyświetlamy małe cyfry na lcd
  }
  if(rit_state == 1)
  {                              //jeśli RIT jest włączony i enkoder pracuje jako RIT wyświetlamy zmiany częstotliwości RIT-a
    myGLCD.setFont(TinyFont);                      //ustawiamy małą czcionkę
    sprintf(buffor,"%05lu",abs(rit_frequency));     //przygotowujemy bufor z zawartością aktualnej wartości RIT
    myGLCD.print(buffor,CENTER,2);                 //drukowanie na lcd
    if(rit_frequency < 0){                          //obsługa znaku poprawki RIT jeśli mniejsza niż 0
      myGLCD.print("-",28,2);                      //drukujemy minus
    }else if(rit_frequency > 0){                    //jeśli większa niż zero to
      myGLCD.print("+",28,2);                      //drukujemy plus
    }                                        
    else{
      myGLCD.print("0",28,2);                      //jeśli poprawka zerowa wrzucam zero zamiast plusa czy minusa
    }
  }
  myGLCD.update();                                 //wysyłamy dane do bufora wyświetlacza
}

//funkcja do wyświetlania aktualnego kroku syntezera za pomocą podkreślenie odpowiedniej cyfry
void show_step(){       
       myGLCD.clrLine(0,31,83,31);          //czyszczę cała linię gdzie wyświetlam podkreślenie
       myGLCD.clrLine(0,32,83,32);          //czyszczę druga linię tak samo podkreśliniki są grube na dwa piksele
  switch(step_value){                       //przełącznik reaguje na bieżącą wartość kroku syntezy
     case 200:
        myGLCD.drawLine(67, 31, 72, 31);    //pokreślam 100Hz
        myGLCD.drawLine(67, 32, 72, 32);    //pokreślam 100Hz
     break;
    case 10:
        myGLCD.drawLine(73, 31, 78, 31);    //pokreślam 10Hz
        myGLCD.drawLine(73, 32, 78, 32);    //pokreślam 10Hz
    break; 
    case 1000:
        myGLCD.drawLine(51, 31, 59, 31);    //pokreślam 1kHz
        myGLCD.drawLine(51, 32, 59, 32);    //pokreślam 1kHz
    break;
    case 10000:
        myGLCD.drawLine(39, 31, 47, 31);    //pokreślam 10kHz
        myGLCD.drawLine(39, 32, 47, 32);    //pokreślam 10kHz
    break;
    case 100000:
        myGLCD.drawLine(27, 31, 35, 31);    //pokreślam 100kHz
        myGLCD.drawLine(27, 32, 35, 32);    //pokreślam 100kHz
    break;    
  }
 myGLCD.update();  //jak już ustaliliśmy co rysujemy to wysyłamy to do LCD
}

//funkcja ustawiająca częstotliwość DDS-a
void set_frequency(int plus_or_minus){
	int correction = 0;                                                                //lokalna zmienna pomocnicza
	frequency = vfo[active_vfo].freq[vfo[active_vfo].band];
	tryb_pracy = vfo[active_vfo].mode[vfo[active_vfo].band];
	if(rit_state == 2 || rit_state == 0)
	{                                            //jeśli nie obsługuję RIT-a to manipuluje częstotliwością
		frequency = frequency + plus_or_minus * step_value;                                          //częstotliwość = częstotliwość + krok
	}
	if(rit_state == 1)
	{                                                              //jesli obsługuję rita
		rit_frequency = rit_frequency + plus_or_minus * rit_step;                      //częstotliwość poprawki zwiększam o krok poprawki
		rit_frequency = constrain(rit_frequency, -rit_range, rit_range);                 //limitujemy poprawkę RIT do wartości z konfiguracji
	}
	if(rit_state != 0 && ptt_on == false)
	{                                           //jeśli jesteśmy w trybie włączonego RIT-a
		correction = rit_frequency;                                                    //lokalna zmienna pomocnicza przyjmuje wartość RIT by można to było dodać do czestotliwości
	}
	frequency = constrain(frequency, bands[vfo[active_vfo].band].lower_limit, bands[vfo[active_vfo].band].upper_limit);       //limitowanie zmiennej częstotliwości tej na wyświetlaczu
	if(mode == 0) //tryb pracy syntezy 0 - pośrednia
	{
		//a tutaj obliczam częstotliwość wynikową dla pracy w trybie pośredniej + ew.poprawka z RIT
		switch(tryb_pracy)
		{
			case 0: // CW
				frequency_to_dds = frequency + configuration.if_frequency - configuration.IFoffset + CWoffset + correction;
				break;
			case 1: // CW VOX - tak jak CW
				frequency_to_dds = frequency + configuration.if_frequency - configuration.IFoffset + CWoffset + correction;
				break;
			case 2: // LSB
				frequency_to_dds = frequency + configuration.if_frequency - configuration.IFoffset + correction;
				break;
			case 3: // USB
				frequency_to_dds = frequency + configuration.if_frequency + configuration.IFoffset + correction;
				break;
		}
	}
	else
	{                                                                           //tryby pracy 1 - mnożnik * 1 generator lub 2 i więcej mnożnik
		frequency_to_dds = (frequency + correction) * mode;                            //mnożymy częstotliwość przez tryb pracy no i pamiętamy o poprawce
	}
	SetFrequency(frequency_to_dds);
	vfo[active_vfo].freq[vfo[active_vfo].band] = frequency;
#if defined(DEBUG)
	Serial.print("freq_to_dds: ");
	Serial.println(frequency_to_dds);
#endif
  }

//wskaźnik s-metra by nie przeszkadzał w pracy enkodera zrobiony jest na pseudo współdzieleniu czasu.
//każdorazowe wykonanie tej funkcji przestawia czas następnego jej wykonania o czas podany w konfiguracji domyślnie 100msec
void show_smetr(){                                
  if(millis() >= s_metr_update_time){                              //sprawdzam czy już jest czas na wykonanie funkcji
     myGLCD.clrLine(1, 45, 83, 45);                                //czyścimy stare wskazanie s-metra linia pierwsza
     myGLCD.clrLine(1, 46, 83, 46);                                //czyścimy stare wskazanie s-metra linia druga
     int s_value = analogRead(s_metr_port);                                 //czytamy wartość z wejścia gdzie jest podpięty sygnał s-metra
	int s_position = 0;  
	 //Serial.println(s_value);
      if(s_value < 85) // < S4
	  {
		s_position = 2;
      }
      else if(s_value == 85) // S4
	  {   
		s_position = 6;
      }
      else if(s_value >= 86 && s_value < 88) // S5
	  {  
		s_position = 14;
      }
      else if(s_value >=88  && s_value < 93) // S6
	  {  
		s_position = 22;
      }
      else if(s_value >= 93 && s_value < 106) // S7
	  {  
		s_position = 30;
      }
      else if(s_value >= 106 && s_value < 128) // S8
	  {   
		s_position = 38;
      }
      else if(s_value >= 128 && s_value < 165) // S9
	  {  
		s_position = 46;
      }
      else if(s_value >= 165 && s_value < 215) // +10dB
	  {  
		s_position = 50;
      }
      else if(s_value >= 215 && s_value < 267) // +20dB
	  {  
		s_position = 56;
      }
      else if(s_value >= 267 && s_value < 325) // +30dB
	  {  
		s_position = 62;
      }
      else if(s_value >= 325 && s_value < 381) // +40dB
	  {  
		s_position = 68;
      }
      else if(s_value >= 381 && s_value < 466) // +50dB
	  {  
		s_position = 74;
      }
      else if(s_value >= 466 && s_value < 523) // +60dB
	  {  
		s_position = 80;
      }
      else if(s_value >= 523) // >65dB
	  {  
		s_position = 83;
      }
	 //Serial.println(s_position);
     myGLCD.drawLine(1, 45, s_position, 45);                       //rysuję nową linię wskazania s metra
     myGLCD.drawLine(1, 46, s_position, 46);                       //rysuję nową linię wskazania s metra
     myGLCD.update();                                              //wysyłam dane do bufora wyświetlacza
     s_metr_update_time = millis() + s_metr_update_interval;       //ustawiam czas kolejnego wykonania tej funkcji
  }
  if (!jest_opis_smetra)
  {
	myGLCD.setFont(TinyFont);                           //najmniejsza czcionka
	//myGLCD.print("S1.3.5.7.9.+20.40.60.", CENTER, 38);  //opis dla s-metra
	myGLCD.print("S4.5.6.7.8.9.20.40.60", CENTER, 38);  //opis dla mojego s-metra po wyskalowaniu
	jest_opis_smetra = true;
  }
}

//funkcja która obsługuje klawisz RIT-a
void rit_swich(){
 myGLCD.setFont(TinyFont); 
 switch(rit_state){
  case 1:  //rit aktywny, enkoder steruje wartoscią RIT-a
  /*
     myGLCD.drawLine(28, 0, 51, 0);             //podkreślam wartość RIT
     myGLCD.drawLine(28, 8, 51, 8);             //podkreślam wartość RIT
     myGLCD.drawLine(72,0,83,0);                //podkreślam oznaczenie rit
     myGLCD.drawLine(72,8,83,8);                //podkreślam oznaczenie rit
	 */
     show_frequency(); 
  break;
  case 2:  //rit aktywny, enkoder steruje częstotliwością - tryb nieużywany
  /*
     myGLCD.clrLine(28, 0, 51, 0);              //anuluję podkreślenie wartości RIT
     myGLCD.clrLine(28, 8, 51, 8);              //anuluję podkreślenie wartości RIT
     myGLCD.drawLine(72,0,83,0);                //podkreślam oznaczenie rit
     myGLCD.drawLine(72,8,83,8);                //podkreślam oznaczenie rit
	 */
  break;
  case 0:  //rit nie jest aktywny
     /* myGLCD.clrLine(28, 0, 51, 0);              //anuluję podkreślenie wartości RIT
     myGLCD.clrLine(28, 8, 51, 8);              //anuluję podkreślenie wartości RIT
     myGLCD.clrLine(72,0,83,0);                 //anuluję oznaczenie rit
     myGLCD.clrLine(72,8,83,8);                 //anuluję oznaczenie rit
	 */
     sprintf(buffor,"       ");  //czyszczę miejsce po wartości RIT gdy pracujemy bez niego
     myGLCD.print(buffor,CENTER,2);             //przygotowuję dane do wysyłki na LCD
  break; 
 }
  myGLCD.update();                              //i wypluwamy to na lcd
  set_frequency(0);                             //ustawiam częstotliwość pracy syntezy na wypadek gdy wartość rit jest różna od zera
}
//funkcja do obsługi zmiany VFO
void change_vfo(){
	myGLCD.setFont(TinyFont);                    //mała czcionka
	switch(active_vfo){                           //sprawdzam jakie VFO zostało wybrane
		case 0:                                   //jeśli A to:
		myGLCD.print("V/A", 56,2);               //sygnalizuję które VFO pracuje
		// frequency_vfo_b = frequency;             //zawartość częstotliwość odkładam do pamięci VFO B
		//vfo[1].freq[vfo[1].band] = frequency;
		//frequency = frequency_vfo_a;             //wczytuję zawartość częstotliwości z pamięci VFO A
		//frequency = vfo[0].freq[vfo[0].band];
		expander.digitalWrite(VFO_A_port, LOW); // led zielony VFO A włączony
		expander.digitalWrite(VFO_B_port, HIGH); // led czerwony VFO B wyłączony
		break;
		case 1:
		myGLCD.print("V/B", 56,2);               //sygnalizuję które VFO pracuje
		// frequency_vfo_a = frequency;             //zawartość częstotliwość odkładam do pamięci VFO A
		//vfo[0].freq[vfo[0].band] = frequency;
		// frequency = frequency_vfo_b;             //wczytuję zawartość częstotliwości z pamięci VFO B
		//frequency = vfo[1].freq[vfo[1].band];
		expander.digitalWrite(VFO_A_port, HIGH); // led zielony VFO A wyłączony
		expander.digitalWrite(VFO_B_port, LOW); // led czerwony VFO B włączony
		break;
	}
	set_frequency(0);                            //ustawiam częstotliwość dla syntezera
	show_frequency();                            //pokazuję częstotliwość na LCD
	//delay(200);                                  //małe opóźnienie
}
//sygnalizacja PTT (sygnalizacja to skutek uboczny dla RIT-a musimy wiedzieć czy odbieramy czy nadajemy)
void ptt_switch(){
  if(last_ptt_state != ptt_on)
  {            //sprawdzam czy zmienił się stan PTT jeśli tak to robię update na LCD
	myGLCD.setFont(TinyFont);                //ustawiam małą czcionkę
    if(ptt_on == true)
	{                    //jeśli TX
		myGLCD.print("T", 0,2);             //to zapalamy T do TX
	    expander.digitalWrite(NO_out, HIGH);
		if (qrp_on == false)
		{
			pa.digitalWrite(PA_NO, HIGH);	// przekaźnik w PA
		}
		//expander.digitalWrite(5, LOW);
		if (split_on)
		{
			if (active_vfo == 0) // jeśli aktualnie jest wybrane VFO_A - przełączenie na VFO_B, jeśli jest B - nie rób nic
			{
				active_vfo = 1;
				change_vfo();
			}
		}
    }
	else
	{                                 //jesli RX
		myGLCD.print("R", 0,2);             //zapalamy R do RX
		expander.digitalWrite(NO_out, LOW);
		if (qrp_on == false)
		{
			pa.digitalWrite(PA_NO, LOW);	// przekaźnik w PA
		}
		//expander.digitalWrite(5, HIGH);
		if (split_on)
		{
			active_vfo = 0;
			change_vfo();
		}
    }
    myGLCD.update();                       //wyrzucam do bufora wyświetlacza
    set_frequency(0);                      //ustawiam częstotliwość (bo może być różna dla TX i RX)
    last_ptt_state = ptt_on;               //uaktualniam poprzedni stan PTT
  }
}
//tutaj rysujemy domyślne elementy ekranu będziemy to robić raz,
//tak by nie przerysowywać całego ekranu podczas pracy syntezy
void show_template(){
  myGLCD.setFont(TinyFont);                           //najmniejsza czcionka
  myGLCD.print("RX", 0,2);                            //Sygnalizacja TX RX będzie tutaj
  myGLCD.print("CW", 12,2);
  myGLCD.print("V/A", 56,2);                          //startuję od VFO A
  //myGLCD.print("RIT", 72,2);                          //Sygnalizacja pracy RIT-u tutaj
  // zbędna
  if (jest_PA)
  {
	myGLCD.print("PA", 76,2);                          //Sygnalizacja komunikacji z PA po I2C
  }
  else
  {
	myGLCD.print("  ", 76, 2);
  }
  myGLCD.print("S4.5.6.7.8.9.20.40.60", CENTER, 38);  //opis dla s-metra
  jest_opis_smetra = true;
  myGLCD.drawRect(0, 44, 83, 47);                     //rysujemy prostokąt dla s-metra podając koordynaty narożników
  myGLCD.update();                                    //i wypluwamy to na lcd
}

// wskaźnik poziomu mocy oraz SWR
void show_swr(){
	// umowne wzmocnienie wzmacniacza mocy od 0 do 15
    sprintf(buffor,"%2u",bands[vfo[active_vfo].band].moc);              //konwersja danych do wyświetlenia (ładujemy moc do stringa i na ekran)
    myGLCD.setFont(SmallFont);                     //ustawiamy małą czcionkę
    myGLCD.print(buffor,72,14);                    //wyświetlamy małe cyfry na lcd

	int swr_position;
	// linijka wskaźnika mocy
    myGLCD.clrLine(1, 46, 83, 46);                                //czyścimy stare wskazanie
	// fala padająca
    int swr_pad_value = analogRead(swr_pad);                                 //odczyt wartości fali padającej
    myGLCD.drawLine(1, 45, 83, 45);                       //rysowanie nowej linii wskazania mocy padającej - cała linia
	// fala odbita
	int swr_odb_value = analogRead(swr_odb);						//odczyt wartości fali odbitej
	if (swr_odb_value <= swr_pad_value)
	{
		swr_position = map(swr_odb_value,0,swr_pad_value,1,83);				//przeskalowanie
	}
	else
	{
		swr_position = 83;
	}
	myGLCD.drawLine(1, 46, swr_position, 46);                       //rysowanie nowej linii wskazania SWR
	myGLCD.setFont(TinyFont);                           //najmniejsza czcionka
	myGLCD.print("..1,5.2...3..5..9..oo", CENTER, 38);  //opis dla swr-metra
	jest_opis_smetra = false;
    myGLCD.update();                                              //wysyłam dane do bufora wyświetlacza
}
void set_moc_PA(uint8_t moc)
{
	// ustawienie tłumika
	switch (moc)
	{
		case 0:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 1:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 2:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 3:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 4:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 5:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 6:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 7:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, LOW);
			break;
		case 8:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 9:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 10:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 11:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, LOW);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 12:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 13:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, LOW);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 14:
			pa.digitalWrite(PA_05, LOW);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, HIGH);
			break;
		case 15:
			pa.digitalWrite(PA_05, HIGH);
			pa.digitalWrite(PA_1, HIGH);
			pa.digitalWrite(PA_2, HIGH);
			pa.digitalWrite(PA_4, HIGH);
			break;
	}
}
void switch_bands()
{
	pa.digitalWrite(prev_port_PA, LOW);
	switch (vfo[active_vfo].band)
	{
		case 0: // 1,8MHz
			set_FPP(2);
			pa.digitalWrite(PA_160M, HIGH);
			prev_port_PA = PA_160M;
			break;
		case 1: // 3,5 MHz
			set_FPP(0);
			pa.digitalWrite(PA_80M, HIGH);
			prev_port_PA = PA_80M;
			break;
		case 2: // 5 MHz
			set_FPP(4);
			pa.digitalWrite(PA_60M, HIGH);
			prev_port_PA = PA_60M;
			break;
		case 3: // 7 MHz
			set_FPP(5);
			pa.digitalWrite(PA_40M, HIGH);
			prev_port_PA = PA_40M;
			break;
		case 4: // 10 MHz
			set_FPP(9);
			pa.digitalWrite(PA_30M, HIGH);
			prev_port_PA = PA_30M;
			break;
		case 5: // 14 MHz
			set_FPP(7);
			pa.digitalWrite(PA_20M, HIGH);			
			prev_port_PA = PA_20M;
			break;
		case 6: // 18 MHz
			set_FPP(6);
			pa.digitalWrite(PA_17M, HIGH);			
			prev_port_PA = PA_17M;
			break;
		case 7: // 21 MHz
			set_FPP(3);
			pa.digitalWrite(PA_15M, HIGH);			
			prev_port_PA = PA_15M;
			break;
		case 8: // 24 MHz
			set_FPP(1);
			pa.digitalWrite(PA_12M, HIGH);			
			prev_port_PA = PA_12M;
			break;
		case 9: // 28 MHz
			set_FPP(8);
			pa.digitalWrite(PA_10M, HIGH);			
			prev_port_PA = PA_10M;
			break;
	}
	tryb_pracy = vfo[active_vfo].mode[vfo[active_vfo].band];
	if (tryb_pracy < 2)	// CW
	{
		digitalWrite(ssb_cw_out, LOW);
	}
	else	// SSB
	{
		digitalWrite(ssb_cw_out, HIGH);
	}
	show_mode();
	set_frequency(0);
	show_frequency();
	if (jest_PA)
	{
		moc = bands[vfo[active_vfo].band].moc;
		set_moc_PA(moc);
		// umowne wzmocnienie wzmacniacza mocy od 0 do 15
		sprintf(buffor,"%2u",bands[vfo[active_vfo].band].moc);              //konwersja danych do wyświetlenia (ładujemy moc do stringa i na ekran)
	}
    myGLCD.setFont(SmallFont);                     //ustawiamy małą czcionkę
    myGLCD.print(buffor,72,14);                    //wyświetlamy małe cyfry na lcd
    myGLCD.update();                               //wysyłam dane do bufora wyświetlacza
}
void set_moc(int8_t enc)
{
	static int8_t licznik_impulsow = 0;
	if (enc > 0)
	{
		//moc++;
		licznik_impulsow++;
	}
	if (enc < 0)
	{
		//moc--;
		licznik_impulsow--;
	}
	if (licznik_impulsow != 0 && licznik_impulsow%10 == 0)
	{
		if (licznik_impulsow > 0)
		{
			moc++;
		}
		else
		{
			moc--;
		}
		licznik_impulsow = 0;
		moc = constrain(moc, 0, 15);
		bands[vfo[active_vfo].band].moc = moc;
		set_moc_PA(moc);
#if defined(DEBUG)
		Serial.print("moc: ");
		Serial.println(moc);
#endif
		config_dirty = 1;
	}
}
int read_settings_from_eeprom() {

  // returns 0 if eeprom had valid settings, returns 1 if eeprom needs initialized
  
    if (EEPROM.read(0) == eeprom_magic_number){
    
      byte* p = (byte*)(void*)&bands;
      unsigned int i;
      int ee = 1; // starting point of bands struct
      for (i = 0; i < sizeof(bands); i++){
        *p++ = EEPROM.read(ee++);  
      }
    
      return 0;
    } else {
      return 1;
    }
  
  return 1;

}
void write_settings_to_eeprom(int initialize_eeprom) {  
 
  if (initialize_eeprom) {
    EEPROM.write(0,eeprom_magic_number);
  }

  const byte* p = (const byte*)(const void*)&bands;
  unsigned int i;
  int ee = 1;  // starting point of bands struct
  for (i = 0; i < sizeof(bands); i++){
    EEPROM.write(ee++, *p++);  
  }
  config_dirty = 0;
}

void check_eeprom_for_initialization(){
  // read settings from eeprom and initialize eeprom if it has never been written to
  if (read_settings_from_eeprom()) {
    write_settings_to_eeprom(1);
  }
}

void check_for_dirty_configuration()
{
	if (config_dirty)
	{
		write_settings_to_eeprom(0);
	}
}
unsigned char getKey()
{
	unsigned char kod_klawisza = 0;
	krok.update();
	if (krok.read() == LOW)
	{
		kod_klawisza |= (1<<3);
#if defined(DEBUG)
		Serial.print("kl po krok: ");
		Serial.println(kod_klawisza, BIN);
#endif
	}
	rit.update();
	if (rit.read() == LOW)
	{
		kod_klawisza |= (1<<2);
#if defined(DEBUG)
		Serial.print("kl po rit: ");
		Serial.println(kod_klawisza, BIN);
#endif
	}
	split.update();
	if (split.read() == LOW)
	{
		kod_klawisza |= (1<<1);
#if defined(DEBUG)
		Serial.print("kl po split: ");
		Serial.println(kod_klawisza, BIN);
#endif
	}
	tryb.update();
	if (tryb.read() == LOW)
	{
		kod_klawisza |= 1;
#if defined(DEBUG)
		Serial.print("kl po mode: ");
		Serial.println(kod_klawisza, BIN);
#endif
	}
	return kod_klawisza & 0x0f;
}

//*****************************************************************************************************************************
//setup funkcja odpalana przy starcie
void set_step()
{
	// obsługa klawisza zmiany kroku
	if (split_on)
	{
		// przełączanie VFO A/ VFO B
		if (active_vfo == 0)
		{
			active_vfo = 1;
		}
		else
		{
			active_vfo = 0;
		}
	   change_vfo();
	}
	else
	{
		switch (step_value)
		{
		case 100000:                        //jeśli krok jest 100kHz ustaw 10kHz
			step_value = 10000;
			break;
		case 1000:                         //jeśli krok jest 10kHz ustaw 1kHz
			step_value = 20;
			break;
		case 200:                          //jeśli krok jest 1kHz ustaw 50Hz
			step_value = 10;
			break;
		case 10:
			step_value = 200;
			break;
		}
		show_step();                          //pokazuję zmianę kroku na lcd
		delay(200);
	}
}
void set_split()
{
	// obsługa włączenia trybu pracy split
	if (split_on)
	{
		split_on = false;
		if (active_vfo == 1) // przejście na VFO_B po wyłączeniu splitu
		{
			active_vfo = 0;
			change_vfo();
		}
		expander.digitalWrite(VFO_A_port, HIGH); // led zielony VFO A wyłączony
		expander.digitalWrite(VFO_B_port, HIGH); // led czerwony VFO B wyłączony
	}
	else
	{
		split_on = true;
		if (active_vfo == 1)
		{
			active_vfo = 0;
			change_vfo();
		}
		expander.digitalWrite(VFO_A_port, LOW); // led zielony VFO A włączony
		expander.digitalWrite(VFO_B_port, HIGH); // led czerwony VFO B wyłączony
	}
	delay(200);
}
void set_rit()
{
//obsługa klawisza włączenia funkcji RIT i zrównania VFO A i B (A=B)
	if (split_on)
	{
		if (active_vfo == 0)
		{
			vfo[1].band = vfo[0].band;
			vfo[1].mode[vfo[1].band] = vfo[0].mode[vfo[0].band];
			vfo[1].freq[vfo[0].band] = vfo[0].freq[vfo[0].band];
		}
		else
		{
			vfo[0].band = vfo[1].band;
			vfo[0].mode[vfo[0].band] = vfo[1].mode[vfo[1].band];
			vfo[0].freq[vfo[1].band] = vfo[1].freq[vfo[1].band];
		}
	}
	else
	{
		switch (rit_state)
		{                     //przełącznik trybu pracy z RIT
		case 1: //jesli tryb jest 1 (enkoder pracuje jako rit wartość RIT dodaję do częstotliwości)
			rit_state = 0;                       //ustaw tryb 0
			break;
		case 2: //jeśli tryb jest 2 (nieużywany!) (enkoder pracuje jako enkoder wartość RIT dodaję do częstotliwości)
			rit_state = 0;                        //ustaw tryb 0
			break;
		case 0: //jeśli tryb jest 0 (enkoder pracuje jako enkoder wartość RIT wyłączony)
			rit_state = 1;                        //ustaw tryb 1
			break;
		}
		rit_swich();                    //odpalam funkcję do obsługi trybu pracy
	}
}
void set_mode()
{
// obsługa klawisza zmiany trybu pracy
	if (split_on == false)
	{
		switch (tryb_pracy)
		{
		case 0:
			tryb_pracy = 1; //CW VOX
			digitalWrite(ssb_cw_out, LOW);
#if defined(jest_BFO)
			Set_BFO_Frequency(configuration.if_frequency - configuration.IFoffset);
#endif
			break;
		case 1:
			tryb_pracy = 2;	// LSB
			digitalWrite(ssb_cw_out, HIGH);
#if defined(jest_BFO)
			Set_BFO_Frequency(configuration.if_frequency - configuration.IFoffset);
#endif
			break;
		case 2:
			tryb_pracy = 3;	// USB
			digitalWrite(ssb_cw_out, HIGH);
#if defined(jest_BFO)
			Set_BFO_Frequency(configuration.if_frequency + configuration.IFoffset);
#endif
			break;
		case 3:
			tryb_pracy = 0;	// CW
			digitalWrite(ssb_cw_out, LOW);
#if defined(jest_BFO)
			Set_BFO_Frequency(configuration.if_frequency - configuration.IFoffset);
#endif
			break;
		}
		vfo[active_vfo].mode[vfo[active_vfo].band] = tryb_pracy;
		set_frequency(0);
	}
	else	// przełączanie QRP/QRO
	{
		if (qrp_on)
		{
			qrp_on = false;
			myGLCD.print(" PA", 72, 2);
		}
		else
		{
			qrp_on = true;
			myGLCD.print("QRP", 72, 2);
		}
	}
}
void show_mode()
{
	myGLCD.setFont(TinyFont);              //ustawiam małą czcionkę
	if (split_on == false)
	{
		switch (tryb_pracy)
		{
		case 0:
			myGLCD.print("CW ", 12, 2);
			break;
		case 1:
			myGLCD.print("CWX", 12, 2);
			break;
		case 2:
			myGLCD.print("LSB", 12, 2);
			break;
		case 3:
			myGLCD.print("USB", 12, 2);
			break;
		}
	}
	else	// stan QRP/QRO
	{
		if (qrp_on)
		{
			myGLCD.print("QRP", 72, 2);
		}
		else
		{
			myGLCD.print(" PA", 72, 2);
		}
	}
	myGLCD.update();                       //wyświetlenie
}
void set_filtr()
{
	// obsługa klawisza włączania filtru CW
	if (filtr_CW_on)
	{
		expander.digitalWrite(filtr_CW_port, LOW);
		filtr_CW_on = false;
	}
	else
	{
		expander.digitalWrite(filtr_CW_port, HIGH);
		filtr_CW_on = true;
	}
	delay(200);
}
void set_AMP_on()
{
	// obsługa klawisza włączania wzmacniacza w.cz. i tłumika
	switch (AMP_state) {
		case 0:
			AMP_state = 1;
			digitalWrite(AMP_on_port, HIGH);
			break;
		case 1:
			AMP_state = 0;
			digitalWrite(AMP_on_port, LOW);
			break;
		default:
			AMP_state = 0;
			break;
	}
}

void set_CWO()
{
	if (last_sztorc_state != sztorc_on)
	{
		if (sztorc_on)
		{
			clk_enable(CWO);
		}
		else
		{
			clk_disable(CWO);
		}
		last_sztorc_state = sztorc_on;
	}
}
void set_FPP(byte kod_pasma)
{
	switch (kod_pasma) {
		case 0:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, LOW);
			break;
		case 1:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, HIGH);
			break;
		case 2:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, HIGH);
			expander.digitalWrite(PORT_A, LOW);
			break;
		case 3:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, HIGH);
			expander.digitalWrite(PORT_A, HIGH);
			break;
		case 4:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, HIGH);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, LOW);
			break;
		case 5:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, HIGH);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, HIGH);
			break;
		case 6:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, HIGH);
			expander.digitalWrite(PORT_B, HIGH);
			expander.digitalWrite(PORT_A, LOW);
			break;
		case 7:
			expander.digitalWrite(PORT_D, LOW);
			expander.digitalWrite(PORT_C, HIGH);
			expander.digitalWrite(PORT_B, HIGH);
			expander.digitalWrite(PORT_A, HIGH);
			break;
		case 8:
			expander.digitalWrite(PORT_D, HIGH);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, LOW);
			break;
		case 9:
			expander.digitalWrite(PORT_D, HIGH);
			expander.digitalWrite(PORT_C, LOW);
			expander.digitalWrite(PORT_B, LOW);
			expander.digitalWrite(PORT_A, HIGH);
			break;
		default:
			break;
	}
}

void setup()
{
	delay(500);
#if defined(DEBUG)
	Serial.begin(1200);                // start serial
	Serial.println("setup poczatek");
#endif
	Wire.begin();
	krok.attach(KROK_PIN, INPUT_PULLUP);
	rit.attach(RIT_PIN, INPUT_PULLUP);
	split.attach(SPLIT_PIN, INPUT_PULLUP);
	tryb.attach(TRYB_PIN, INPUT_PULLUP);
	ptt.attach(PTT_PIN, INPUT_PULLUP);
	sztorc.attach(SZTORC_PIN, INPUT_PULLUP);

  expander.begin(PCF_ADDRESS);							// adres expandera PCF8574 (nóżki 1,2,3 czyli A0,A1,A2 do masy)

  expander.pinMode(NO_out, OUTPUT);
  expander.digitalWrite(NO_out, LOW);
  expander.pinMode(filtr_CW_port, OUTPUT);
  expander.digitalWrite(filtr_CW_port, LOW); // po włączeniu filtr CW wyłączony
  byte error = 0;

  if(bod_lvl == 1)
  {                             //tutaj przy starcie ustawiam właściwe napięcie odniesienia
	  analogReference(INTERNAL);                  //dla ATmega168 i ATmega328, INTERNAL = 1.1V w arduino mega wpisujemy INTERNAL1V1
  }

  pinMode(s_metr_port,INPUT);             //ustawiam tryb pracy wejścia s-metra
  //pinMode(ptt_input,INPUT_PULLUP);        //ustawiam tryb pracy wejścia PTT
  pinMode(A0, INPUT_PULLUP);					// wejście enkodera
  pinMode(A1, INPUT_PULLUP);					// wejście enkodera
//  pinMode(dit_input, INPUT_PULLUP);				// wejście kropek
//  pinMode(dah_input, INPUT_PULLUP);				// wejście kresek
  //pinMode(key_output, OUTPUT);					// wyjście kluczujące
  //digitalWrite(key_output, LOW);				// wstępne ustawienie na 0
  pinMode(ssb_cw_out, OUTPUT);					// wyjście przełączania SSB/CW
  pinMode(AMP_on_port, OUTPUT);					// włączanie i wyłączanie wzmacniacza w.cz.
  digitalWrite(AMP_on_port, LOW);

  expander.pinMode(VFO_A_port, OUTPUT);
  expander.digitalWrite(VFO_A_port, HIGH); // led zielony VFO A wyłączony
  expander.pinMode(VFO_B_port, OUTPUT);
  expander.digitalWrite(VFO_B_port, HIGH); // led czerwony VFO B wyłączony
  expander.pinMode(PORT_A, OUTPUT);
  expander.pinMode(PORT_B, OUTPUT);
  expander.pinMode(PORT_C, OUTPUT);
  expander.pinMode(PORT_D, OUTPUT);
  Wire.beginTransmission(0x21);
  error = Wire.endTransmission();

  if (error == 0)
  {
	jest_PA = true;
	pa.begin(0x21); // adres PCF8575 (A1 i A2 do masy)
	pa.pinMode(PA_160M, OUTPUT);
	pa.pinMode(PA_80M, OUTPUT);
	pa.pinMode(PA_40M, OUTPUT);
	pa.pinMode(PA_30M, OUTPUT);
	pa.pinMode(PA_20M, OUTPUT);
	pa.pinMode(PA_17M, OUTPUT);
	pa.pinMode(PA_15M, OUTPUT);
	pa.pinMode(PA_12M, OUTPUT);
	pa.pinMode(PA_10M, OUTPUT);
	pa.pinMode(PA_6M, OUTPUT);
	pa.pinMode(PA_NO, OUTPUT);
	pa.pinMode(PA_05, OUTPUT);
	pa.pinMode(PA_1, OUTPUT);
	pa.pinMode(PA_2, OUTPUT);
	pa.pinMode(PA_4, OUTPUT);
  }
	else
	{
		jest_PA = false;
	}
  vfo[0].band = 1;  //początkowe ustawienia pasma po starcie systemu
  vfo[1].band = 1;
  tryb_pracy = modes[1];
  prev_port_PA = PA_80M;
//  prev_port_TRX = PORT_30M;
  check_eeprom_for_initialization();

  delay(200);
  oe_setup();

  switch_bands();
  for(byte i=0; i<ile_pasm;i++)
  {
    vfo[0].freq[i] = bands[i].mid;
    vfo[1].freq[i] = bands[i].mid;
    vfo[0].mode[i] = modes[i];
    vfo[1].mode[i] = modes[i];
  }
  Timer1.initialize(250); // set a timer of length 1ms - odczyt wejść enkodera będzie się odbywał co 1ms
  Timer1.attachInterrupt( encode_read ); // attach the service routine here
  myGLCD.InitLCD(contrast);               //odpalamy lcd ustawiamy kontrast
  myGLCD.clrScr();                        //czyścimy ekran z ewentualnych śmieci
  myGLCD.setFont(TinyFont);                     //czas na reklamę, mała czcionka
  myGLCD.print("Zgredek ver.",0,2);              //lokowanie produktu przy starcie
  myGLCD.print(software_version, 48,2);         //numer wersji
  myGLCD.update();
  delay(1000);                                  //opóźnienie
  myGLCD.clrScr();                              //koniec reklam czyścimy ekran
#if defined(jest_BFO)
	write_register(17, B01101101);			// CLK1: powered up, integer mode, PLL_B, not inverted, Multisynth, 8mA
	Set_BFO_Frequency(configuration.if_frequency - configuration.IFoffset);		// LSB na starcie
	write_register(18, B01101101);			// CLK2: powered up, integer mode, PLL_B, not inverted, Multisynth, 6mA
	simple_set_frequency(configuration.if_frequency - configuration.IFoffset + CWoffset, CWO);	// częstotliwość dla CWO
#if defined(DEBUG)
	ch_status();
#endif
#endif
  set_frequency(0);                       //odpalamy syntezer i ustawiamy częstotliwość startową
  show_frequency();                       //pokazujemy częstotliwość na lcd
  show_step();                            //pokazujemy krok syntezy
  show_template();                        //pokazujemy domyślne stałe elementy LCD
  show_mode();
} // koniec setupu

void loop(){

  //obsługa PTT
  ptt.update();
  if(ptt.read() == LOW)
  {       //odczytuję wejście PTT jeśli jest aktywne
    ptt_on = true;                         //ustawiam zmienną pomocniczą na prawda (flaga)
  }
  else
  {
		ptt_on = false;                        //ustawiam zmienną pomocniczą na fałsz (zdejmuję flagę)
  }
  ptt_switch();

//obsługa enkodera
int enc = encode_read4();
if (enc != 0)
{
	if (ptt_on)
	{
		if (jest_PA)
		{
			set_moc(enc);
		}
	}
	else
	{
		set_frequency(enc);
		show_frequency();
	}
}
if (!ptt_on) 
{
	// sprawdzenie, czy były zmiany w konfiguracji i ewentualny zapis do EEPROM
    check_for_dirty_configuration();

  //obsługa przycisków - tylko podczas odbioru
	unsigned char kod_klawisza;
	kod_klawisza = getKey();
	if (kod_klawisza)
	{
		if (kod_klawisza == getKey())
		{
			#if defined(DEBUG)
				Serial.print("kod klawisza: ");
				Serial.println(kod_klawisza);
			#endif
			switch(kod_klawisza)
			{
				case KROK_KEY:			// STP/A/B
					set_step();
					break;
				case MODE_KEY:		// MODE
					set_mode();
					show_mode();
					break;
				case SPLIT_KEY:		// SPL
					set_split();
					break;
				case RIT_KEY:		// RIT//A/B
					set_rit();
					break;
				case UP_KEY:	// UP
					if (vfo[active_vfo].band == ile_pasm - 1)
					{
						vfo[active_vfo].band = 0;
					}
					else
					{
						vfo[active_vfo].band++;
					}
					switch_bands();
					break;
				case DOWN_KEY:
					if (vfo[active_vfo].band == 0)
					{
						vfo[active_vfo].band = ile_pasm - 1;
					}
					else
					{
						vfo[active_vfo].band--;
					}
					switch_bands();
					break;
				case FILTR_CW_KEY:
					set_filtr();
					break;
				case AMP_on_KEY:
					set_AMP_on();
					break;
				default:
					break;
			}
			delay(200);
		}
	}
  show_smetr();                               //wywołuję funkcję do obsługi s-metra
  //Serial.println(freeRam());                //testowanie ilości dostępnego RAM-u, aby zadziałało należy odkomentować funkcję poniżej
}
else // ptt_on
{
	show_swr();
	sztorc.update();
	if (sztorc.read() == LOW)
	{
		sztorc_on = true;
	}
	else
	{
		sztorc_on = false;
	}
	set_CWO();
}
}

//KONIEC PROGRAMU
//*****************************************************************************************************************************

//testowanie ilości dostępnego RAMU
//int freeRam () {
// extern int __heap_start, *__brkval;
// int v;
// return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
// }

