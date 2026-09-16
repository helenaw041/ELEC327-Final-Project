# Ultrasonic Phased Array

Mixed-signal ultrasonic phased-array board integrating multi-rail power conversion, op-amp envelope buffering, and 40 kHz carrier analog switches to drive a 4-channel transducer array.
On MSPM0, using DMA.

MSP code in 'Final Project' Folder
Board in .zip, and post-fab image in Board.jpg
Refer to 'Writeup.pdf' for more information.

### Future Work
Fully debugged up until transducer output. Output was very quiet--transducer footprint is larger than the wavelength of the 40kHz (causing aperture smearing) and did not allow for enough constructive interference and tuning of phases to amplify audio. 
