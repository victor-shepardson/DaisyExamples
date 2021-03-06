#include "daisy_petal.h"
#include "daisysp.h"
#include <complex>
#include <math.h>

using namespace std;
using namespace daisy;
using namespace daisysp;

//TODO:
// pitch shifter class
// toggle to double pitch range
// stereo spread instead of up/down?
//   freq up / pitch down?
// toggle dry to pre-post saturation
// LFO
// saturator footswitch

//immediate ideas:
// do something with phase fuzz?
// pitch tracker
// transient modulated delay

//further ideas:
//slow chroma decomp + fast transient separator?

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T> T wrap(T val) {
  if (val > T(1))
    return val - T(2);
  if (val < T(-1))
    return val + T(2);
  return val;
}

class QOsc
{
private:
  complex<float> state;
  complex<float> inc;
  float sr_;
public:
  QOsc(){
    inc = state = complex<float>(1.);
    sr_ = 48000;
  }
  ~QOsc() {}
  void Init(float sr, float freq=0.){
    sr_ = sr;
    state = complex<float>(1);
    SetFreq(freq);
  }
  void SetFreq(float freq){
    inc = pow(complex<float>(-1), freq/sr_);
  }
  complex<float> Process(){
    state *= inc; //* (1 + 1e-2 - 1e-2*state*conj(state));
    return state;
  }
};

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
// Phasor phasor;
QOsc quad_osc;
Oscillator lfo;
CrossFade fader;

bool bypassOn, saturate, pitch_shift_env, lfo_enabled;

//controls
float shift_freq, env_shift_amt, drive, gain, pitch, dry, pitch_toggle;

//global for updating LEDs
float phasor_freq;

void UpdateControls();

float phi_prev = 0;
float shifted_phi = 0;

void AudioCallback(float **in, float **out, size_t size)
{
    UpdateControls();

    for(size_t i = 0; i < size; i++)
    {
        // input signal in quadrature
        complex<float> quad = hilbert.Process(in[0][i]);

        // envelope follower
        float env = env_follow.Process(abs(quad));

        //shift_env knob does nothing at 12:00
        //at negative values, shift is suppressed by envelope
        // at positive values, shift scales with envelope
        float shift_env;
        if (env_shift_amt < 0){
          shift_env = exp(50.f*env_shift_amt*env);
        } else {
          shift_env = pow(2, env_shift_amt*3)*pow(env+1e-15, env_shift_amt*2);
        }
        lfo.SetFreq(pow(2.f, shift_env*5.f-2.f));
        lfo.SetAmp(pow(env,0.25f));


        // TEMP: PHASE FUZZ
        float phi = -atan2(quad.imag(), quad.real())/PI_F;
        float delta_phi = phi - phi_prev;
        // unwrap phase
        delta_phi = wrap(delta_phi);

        float phi_mult = pow(2.f, pitch_shift_env?pitch*shift_env:pitch);
        delta_phi *= phi_mult;

        shifted_phi += delta_phi;
        // shifted_phi = fmod(shifted_phi*0.5f+0.5f, 1.f)*2.f-1.f;
        shifted_phi = wrap(shifted_phi);
        // if (shifted_phi > 1.f){
        //   shifted_phi -=2.f;
        // }
        phi_prev = phi;
        phi = shifted_phi;

        // float ceiling = 1e-1;
        // float noise_floor = 1e-3;
        // float phi_env =
        //   sqrt(max(0.f, env-noise_floor) / ceiling) * ceiling;
        // out[0][i] = out[1][i] = phi * phi_env;
        // continue;

        // TODO: wavetables/octave+ here?
        quad = abs(quad)*(
          cos(shifted_phi*PI_F*pitch_toggle) +
          1.fi*sin(shifted_phi*PI_F*pitch_toggle)
        );

        phasor_freq = shift_freq*shift_env*(lfo_enabled?lfo.Process():1.f);
        // quadrature oscillator
        quad_osc.SetFreq(phasor_freq);
        complex<float> q = quad_osc.Process();
        // phasor.SetFreq(phasor_freq);
        // float phi = TWOPI_F*phasor.Process();
        // float q0 = cos(phi);
        // float q1 = sin(phi);

        // frequency shift
        float shifted_up = quad.real()*q.real() + quad.imag()*q.imag();
        float shifted_down = quad.real()*q.real() - quad.imag()*q.imag();

        // float shifted_up = quad.real()*q0 + quad.imag()*q1;
        // float shifted_down = quad.real()*q0 - quad.imag()*q1;

        //saturate
        float out_gain, out_left, out_right;
        if(saturate){
          float saturate_level = 1e-1*(env+1.f/drive);
          float sat_gain = drive/saturate_level;
           out_left = sin(sat_gain*shifted_up);
           out_right = sin(sat_gain*shifted_down);
           out_gain = saturate_level*gain;
         }
         else{
           out_left = shifted_up;
           out_right = shifted_down;
           out_gain = gain;
         }

        if (bypassOn){
          out[0][i] = in[0][i];
          out[1][i] = in[1][i];
        }
        else{
          out[0][i] = out_left * out_gain + dry*in[0][i];
          out[1][i] = out_right * out_gain + dry*in[1][i];
        }
    }
}

int main(void)
{
    float samplerate;
    petal.Init();
    samplerate = petal.AudioSampleRate();

    shift_freq = 0;
    env_shift_amt = 0;

    // phasor.Init(samplerate, 1., 0.);
    quad_osc.Init(samplerate, 0.);
    env_follow.Init(samplerate);

    lfo.Init(samplerate);
    lfo.SetAmp(1);
    lfo.SetFreq(3);
    lfo.SetWaveform(Oscillator::WAVE_SIN);

    petal.StartAdc();
    petal.StartAudio(AudioCallback);

    float ring_phase = 0;
    uint32_t frame_us = 10000;
    while(1)
    {
        petal.ClearLeds();

        petal.SetFootswitchLed(
          (DaisyPetal::FootswitchLed)0, !bypassOn);
        petal.SetFootswitchLed(
          (DaisyPetal::FootswitchLed)1, petal.switches[1].Pressed());
        petal.SetFootswitchLed(
          (DaisyPetal::FootswitchLed)2, lfo_enabled);
        petal.SetFootswitchLed(
          (DaisyPetal::FootswitchLed)3, saturate);


        float ring_steps = frame_us * 1e-6 * abs(phasor_freq) * 8.f;
        float freq_color = sgn(phasor_freq) * log2(abs(phasor_freq)/30.f + 1.)/2.;
        float vr, vg, vb;
        vr = vg = vb = 0;
        // white < magenta < red < yellow > green > cyan > white
        if (freq_color < -4){
          // white
          vr = vg = vb = 1;
        } else if (freq_color < -3){
          // white - magenta
          vr = vb = 1;
          vg = -3 - freq_color;
        } else if (freq_color < -2){
          // magenta - red
          vr = 1;
          vb = -2 - freq_color;
        } else if (freq_color < -1){
          // red - yellow
          vr = 1;
          vg = 2 + freq_color;
        } else if (freq_color < 1){
          // yellow
          vr = vg = 1;
        } else if (freq_color < 2){
          // yellow - green
          vr = 2 - freq_color;
          vg = 1;
        } else if (freq_color < 3){
          // green - cyan
          vg = 1;
          vb = freq_color - 2;
        } else if (freq_color < 4){
          // cyan - white
          vg = vb = 1;
          vr = freq_color - 3;
        } else {
          // white
          vr = vb = vg = 1;
        }
        if (ring_steps >= 8) {
          for(int i=0; i<8; i++){
            petal.SetRingLed((DaisyPetal::RingLed)i, vr, vg, vb);
          }
        }
        else {
          for(int i=int(ring_phase); i<=int(ring_phase+ring_steps+1); i++){
            float v;
            if (i < ring_phase)
              v = 1 - (ring_phase - i);
            else if (i < ring_phase + ring_steps)
              v = 1;
            else
              v = 1 - (i - ring_phase - ring_steps);
            petal.SetRingLed((DaisyPetal::RingLed)(i%8), v*vr, v*vg, v*vb);
          }
        }
        ring_phase = fmod(ring_phase + ring_steps, 8.f);

        petal.UpdateLeds();
        System::DelayUs(frame_us);
    }
}


void UpdateControls()
{
    petal.ProcessDigitalControls();
    petal.ProcessAnalogControls();

    //knobs
    gain = pow(2., petal.GetKnobValue(DaisyPetal::KNOB_1)*8.f-4.f);
    dry = pow(petal.GetKnobValue(DaisyPetal::KNOB_2), 4.f);

    //TODO: coarse/fine switch, linear region?
    shift_freq =
      pow(petal.GetKnobValue(DaisyPetal::KNOB_3)*16.f, 3.0f)
      * (petal.switches[4].Pressed()?1:-1);
    pitch =
      petal.GetKnobValue(DaisyPetal::KNOB_4)
      * (petal.switches[5].Pressed()?1:-1);

    env_shift_amt = petal.GetKnobValue(DaisyPetal::KNOB_5)*2.f-1.f;
    //switch 7
    pitch_shift_env = petal.switches[6].Pressed();


    drive = pow(2., petal.GetKnobValue(DaisyPetal::KNOB_6)*4.f);

    //switch 1: bypass
    if(petal.switches[0].RisingEdge())
    {
        bypassOn = !bypassOn;
    }

    //switch 2: +octave
    pitch_toggle = petal.switches[1].Pressed()?2.f:1.f;

    //switch 3: lfo on/off
    if(petal.switches[2].RisingEdge())
    {
        lfo_enabled = !lfo_enabled;
    }

    //switch 4: saturate on/off
    if(petal.switches[3].RisingEdge())
    {
        saturate = !saturate;
    }


}
