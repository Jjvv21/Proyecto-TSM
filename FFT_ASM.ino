#include <arduinoFFT.h>

#define samples 64
#define sampleFreq 7000

const int pinEntrada = A2;    // Pin analógico A2
const int pinLed = 2;         // Pin digital 2 para el LED
const int pinBuzzer = 4; 
const int muestras = 128;     // Número de muestras
bool outputActivated = false; 
double vReal[muestras];      
double vImag[muestras];       
double previousValue;
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, sampleFreq); // Instancia de la clase ArduinoFFT

// Constantes relacionadas con la FFT
const double frecuenciaMuestreo = 7000.0;  // Frecuencia de muestreo en Hz

void setup() {
  Serial.begin(9600);          
  pinMode(pinEntrada, INPUT); 
  pinMode(pinLed, OUTPUT);     
  pinMode(pinBuzzer, OUTPUT);    
  digitalWrite(pinLed, LOW);   
  digitalWrite(pinBuzzer, HIGH);
}

void loop() {
  // Captura de muestras
  for (int i = 0; i < muestras; i++) {
    vReal[i] = lowPassFilter(analogRead(pinEntrada));
    vImag[i] = 0;  // Las muestras imaginarias siempre son 0 en este caso

    long start = micros();
    while ((micros() - start) < 1000000 / 7000);
  }

  normalize(vReal, muestras);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  double peakFrequency = FFT.MajorPeak(); // Obtener frecuencia pico
  Serial.println(peakFrequency / 2);
  Serial.println(outputActivated);

  if (!outputActivated && peakFrequency / 2 >= 95 && peakFrequency / 2 <= 110) {
    outputActivated = true; 
    digitalWrite(pinLed, HIGH); 
    digitalWrite(pinBuzzer, LOW);
    delay(500);
    digitalWrite(pinBuzzer, HIGH);
    

  } else if (outputActivated && peakFrequency / 2 >= 395 && peakFrequency / 2 <= 410) {
    digitalWrite(pinLed, LOW); 
    outputActivated = false; 

  }
}

double lowPassFilter(double input) {
  double alpha = 0.9;
  previousValue = alpha * input + (1 - alpha) * previousValue;
  return previousValue;
}

void normalize(double* signal, int length) {
  double mean = 0;

  // Calcular la media de la señal
  for (int i = 0; i < length; i++) {
    mean += signal[i];
  }
  mean /= length;

  // Restar la media de cada muestra
  for (int i = 0; i < length; i++) {
    signal[i] -= mean;
  }
}