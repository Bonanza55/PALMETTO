# PALMETTO
PALMETTO is a one-way, store-on-arrival text messaging and telemetry protocol for narrowband VHF FM voice channels, using audio frequency-shift keying (AFSK). 

How to start the reciever service. 
  ./fsk_rxd -f 147550000 -p 60 -g 420 2>> rxd.stderr.log &

How to start the transmitter.
  python3 FSKModIc.py & 

How to start the receiver.
  python3 FSKDemodIc.py & 

You will need.
  pip install numpy scipy sounddevice pyserial
  sudo apt-get install python3-tk

  DigRig mobile TNC with the universal cable for your radio
  Nooelec NESDR SMArt V5 Bundle and a USBA to USBC cable
