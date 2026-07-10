# PALMETTO
PALMETTO is a one-way, store-on-arrival text messaging and telemetry protocol for narrowband VHF FM voice channels, using audio frequency-shift keying (AFSK). 

How to start the reciever service. 
  ./fsk_rxd -f 147550000 -e 40 -p 60 -g 200 -q 18 2>> fsx_rxd.log

How to start the transmitter.
  python3 FSKModIc.py & 

How to start the receiver.
  python3 FSKDemodIc.py & 

You will need.
  pip install numpy scipy sounddevice pyserial
  sudo apt-get install python3-tk

  DigRig mobile TNC with the universal cable for your radio
  Nooelec NESDR SMArt V5 Bundle and a USBA to USBC cable

FM Player
  fsk_wfmd FM broadcast radio player. 120 second buffer, so just wait. 
  ./fsk_wfmd -f 89300000 &

Wave File Analyzer
  python3 wav_analyzer.py wavFile minFQ maxFQ FFT startSec endSec zoomSec mark space
  python3 wav_analyzer.py fsk_260706.163413.wav 1000 4000 2400 3.0 4.25 3.33 1200 2400
