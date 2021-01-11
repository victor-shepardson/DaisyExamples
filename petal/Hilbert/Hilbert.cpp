#include "daisy_petal.h"
#include "daisysp.h"
#include <complex>
#include <math.h>

using namespace std;
using namespace daisy;
using namespace daisysp;

class HilbertAP
{
  // allpass stage for IIR Hilbert Transform:
  // http://yehar.com/blog/?p=368
private:
  float x1, x2, y1, y2, c;
public:
  HilbertAP(float c_) {
    x1 = x2 = y1 = y2 = 0;
    c = c_;
  }
  ~HilbertAP() {}
  float Process(float x){
    float y = c * (x + y2) - x2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }
};

class Hilbert
{
private:
  HilbertAP realAP[4] = {
    HilbertAP(0.47944111608296202665),
    HilbertAP(0.87624358989504858020),
    HilbertAP(0.97660296916871658368),
    HilbertAP(0.99749940412203375040)
  };
  HilbertAP imagAP[4] = {
    HilbertAP(0.16177741706363166219),
    HilbertAP(0.73306690130335572242),
    HilbertAP(0.94536301966806279840),
    HilbertAP(0.99060051416704042460)
  };
  float delayed;
public:
  Hilbert() {
    delayed = 0;
  }
  ~Hilbert() {}
  complex<float> Process(float x){
    float yr, yi;
    yr = yi = x;
    for (int i=0; i<4; i++){
      yr = realAP[i].Process(yr);
      yi = imagAP[i].Process(yi);
    }
    auto ret = complex<float>(delayed, yi);
    delayed = yr;
    return ret;
  }
};

class EnvFollow
{
private:
  float ms_since_peak_;
  float hold_ms_; //time to hold peak without decaying
  float decay_samp_; //decay factor each sample
  float env_;
  float sample_ms_;
public:
  EnvFollow(){
    ms_since_peak_ = 0;
    env_ = 0;
  }
  ~EnvFollow(){}
  void Init(float sample_rate, float hold_ms=30., float decay_ms=3.){
    // decay_ms_: time to decay by half
    sample_ms_ = 1000./sample_rate;
    hold_ms_ = hold_ms;
    // decay_ms_ = decay_ms;
    decay_samp_ = pow(0.5, 1000./(sample_rate*decay_ms));
  }
  float Process(float x){
    // envelope a nonnegative signal x
    if (x > env_){
      ms_since_peak_ = 0;
      env_ = x;
    }
    if (ms_since_peak_ > hold_ms_){
      env_ *= decay_samp_;
    }
    ms_since_peak_ += sample_ms_;
    return env_;
  }
};

DaisyPetal petal;
Hilbert hilbert;
EnvFollow env_follow;
Phasor phasor;
Oscillator lfo;
CrossFade fader;

bool bypassOn;

Parameter shiftFreqParam;
float shift_freq;

void UpdateControls();

void AudioCallback(float **in, float **out, size_t size)
{
    UpdateControls();

    for(size_t i = 0; i < size; i++)
    {
        complex<float> quad = hilbert.Process(in[0][i]);

        float env = env_follow.Process(abs(quad));
        // TODO: knob interpolates between these with no envelope at 12:00
        phasor.SetFreq(shift_freq*(1.-pow(2., -40.*env)));
        // phasor.SetFreq(shift_freq*env);

        float phi = TWOPI_F*phasor.Process();
        float q0 = cos(phi);
        float q1 = sin(phi);

        // out[0][i] = quad.real();
        // out[1][i] = quad.imag();

        out[0][i] = quad.real()*q0 + quad.imag()*q1;
        out[1][i] = quad.real()*q0 - quad.imag()*q1;
    }
}

int main(void)
{
    float samplerate;
    petal.Init();
    samplerate = petal.AudioSampleRate();

    shiftFreqParam.Init(petal.knob[0], 0, 20, Parameter::LINEAR);
    shift_freq = 0;

    phasor.Init(samplerate, 1., 0.);
    env_follow.Init(samplerate);

    petal.StartAdc();
    petal.StartAudio(AudioCallback);

    int i = 0;
    while(1)
    {
        petal.ClearLeds();

        petal.SetFootswitchLed((DaisyPetal::FootswitchLed)0, !bypassOn);

        petal.SetRingLed((DaisyPetal::RingLed)i, 0, 1, 1);
        i++;
        i %= 8;

        petal.UpdateLeds();
        System::Delay(60);
    }
}


void UpdateControls()
{
    petal.ProcessDigitalControls();

    //knobs
    shift_freq = pow(shiftFreqParam.Process(),3.0);

    //bypass switch
    if(petal.switches[0].RisingEdge())
    {
        bypassOn = !bypassOn;
    }
}
