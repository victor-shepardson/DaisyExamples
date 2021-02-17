## Description
Hilbert transform-based effects: frequency shifter, monophonic pitch shifter, and envelope follower.

## Inputs
* IN L (MONO): input
* IN R: [disabled]
* EXP: [disabled]

## Controls
top row:
* KNOB 1: gain.
  * 12:00: unity gain
* KNOB 2 dry amount.
  * left: wet only
  * right: wet + dry
* Encoder:
  * KNOB: [no function]
  * button: [no function]
  * lights: display the current frequency shift.

middle row:
* KNOB 3: frequency shift amount.
  * left: no freq shift.
  * right: extreme freq shift.
  * SWITCH 5: frequency shift up/down.
* KNOB 4: pitch shift amount.
  * left: no pitch shift.
  * right: 1 octave.
  * SWITCH 6: pitch shift up/down.
* KNOB 5: envelope effect.
  * left: suppress shifting on attack.
  * 12:00: no effect.
  * right: shift more on attack.
  * SWITCH 7: envelope effect.
    * up: KNOB 5 applies to pitch shift and freq shift.
    * down: KNOB 5 applies to freq shift only.
* KNOB 6: wavefold amount.
  * left: slight saturation
  * right: extreme sinusoidal splat
  * no effect if SWITCH 4 is off.

footswitches:
* SWITCH 1: bypass (no true bypass unfortunately)
* SWITCH 2: momentary +octave
* SWITCH 3: LFO effect toggle.
  * interacts with envelope (KNOB 5)
  * makes more of a vibrato when dry is off / chorus when dry is on (KNOB 2)
* SWITCH 4: wavefold effect toggle.

## Outputs
* OUT L: mono out
* OUT R: stereo out (freq shift in opposite direction)

## Recipes
leslie / rotating speaker:
  * stereo outs to two amps
  * clean (SWITCH 4 off)
  * no dry (KNOB 2 hard left)
  * no pitch shift (KNOB 4 hard left)
  * small freq shift (KNOB 3 far left)
  * light envelope (KNOB 5 slightly left)

octave fuzz:
  * tone down on guitar
  * pitch shift +octave (KNOB 4 hard right, SWITCH 6 up)
  * don't evelope pitch (SWITCH 7 down)
  * wavefold (SWITCH 4 on, KNOB 6 ~12:00)

goofy-ass sounds:
  * everything else
