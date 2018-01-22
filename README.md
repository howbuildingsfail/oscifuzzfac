# oscifuzzfac
Arduino-controlled fuzz factory with MIDI controlled LFOs


All spi trimmers are 10k - so we'll have slightly different settings to consider if you are copying settings from t'internet


#Notes / Testing / TODO

##Frequency

There are two 'bands' of frequency at the moment - the range is massive, but difficult to control precisely - we'd like to get a relationship between frequency and bpm at some point. The range should be from (say) 90 to (say) 218 - probably by way of a lookup table. 

We could possibly do this via a multiplier on the second frequency channel - so 65 is a scale of 1.. will take some thinking about

##Waveshape

We have sin(0-31), saw(32-63), square(64-95) and triangle(96-127)


##Mag / Centre

There's no checking for bounds here, so if your combination of Centre and Mag strays outside the range 0-255, you'll get artefacts. Some people might like this, and it keeps the coding fast to leave out checking...

##Phase

Not sure how well this works yet, because it uses the mozzi `setStart` function - which might mean that the shape loops around to this set start point - changing the shape entirely...

###Comp

Tested with other pots to manual,  D=127, stab=gate=64. All works fine - good candidate for synthing!

###Drive,Gate, Stab

All work as notes above


###noteon settings

There are a couple of no



# MIDI Table

## MIDI Control Changes


| Channel No | Knob | Effect |
| --- | :--- | :-- |
|  16 | Comp | Coarse LFO freq change |
|  17 |      | Fine Comp LFO freq change |
|  18 |      | Select Comp wavetable (Sin(0-32),Tri(33-63),Saw(64-95),Squ(96-127) |
|  19 |      | LFO sweep size |
|  20 |      | LFO sweep centre |
|  21 |      | LFO phase |
| --- | :--- | :-- |
|  32 | Dist | Coarse LFO freq change |
|  33 |      | Fine Comp LFO freq change |
|  34 |      | Select Comp wavetable (Sin(0-32),Tri(33-63),Saw(64-95),Squ(96-127) |
|  35 |      | LFO sweep size |
|  36 |      | LFO sweep centre |
|  37 |      | LFO phase |
| --- | :--- | :-- |
|  48 | Gate | Coarse LFO freq change |
|  49 |      | Fine Comp LFO freq change |
|  50 |      | Select Comp wavetable (Sin(0-32),Tri(33-63),Saw(64-95),Squ(96-127) |
|  51 |      | LFO sweep size |
|  52 |      | LFO sweep centre |
|  53 |      | LFO phase |





