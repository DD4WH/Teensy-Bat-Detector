# Teensy-Bat-Detector
(c) by Frank, DD4WH

Build a semiprofessional bat detector with the Teensy 3.6 microcontroller &amp; Teensy audio board for less than 100 €

I developed this bat detector in the hope that it will promote the study of bat calls by everyone who is interested in the natural history and bioacoustics of bats AND/OR interested in the technical details of how the detection of ultrasound signals can be performed with high tech-low cost equipment. Students of high schools, colleges and universities (who mostly have a limited budget) now have the possibility to obtain, build, test and work with this bat detector in order to obtain a more detailed understanding of bats bioacoustics and ecology. 

(Or just use it for casual listening while having a rendezvous sitting at a beautiful lake in summer evenings or to show off with the latest hightech colour screen gadget at a party). 

In the end, hopefully, it will enable more people to be able to "hear" and investigate (ultra-)sound and better understand animals using sound for navigation and/or communication like bats, birds, grasshoppers, aquatic mammals and many more.

![](https://github.com/DD4WH/Teensy-Bat-Detector/blob/master/IMG_2172.JPG)

See the WIKI for details:
https://github.com/DD4WH/Teensy-Bat-Detector/wiki

Discussion on the Teensy forum (mainly on software issues of the bat detector):
https://forum.pjrc.com/threads/38988-Bat-detector

Features:
-	Heterodyne frequency translation
-	Time expansion by playback of ultrasonic recordings in slower speed
-	Record ultrasound on microSD card for later analysis with computer software, eg. the brilliant (and free!) Bat Explorer (http://www.batlogger.com/en/support/download.html)
-	Spectrum display of the ultrasound (frequency vs. Audio power level)
-	Spectrogram display = waterfall display (frequency vs. Time vs. Audio Power level)
-	0 to 96kHz frequency span (96kHz is possible only with a suitable MIC)
- open source software (MIT licence) that can easily be extended for further functions

Thanks to Frank B (https://github.com/FrankBoesing), who kindly provided the code to change the samplerate and for the waterfall display. Without that, this bat detector would not have been possible!

