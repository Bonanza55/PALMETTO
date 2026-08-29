PALMETTO Digital Messaging System w/ Mesh

Welcome to PALMETTO, a unified simplex packet VHF/UHF radio transceiver 
and FSK demodulation package for off grid radio operators.

MURS for non hams, and simplex [147.550] for hams. 

Radio must be connected and turned on and at least 2 operators. 

I. Installation & Setup
-----------------------
1. Extract Archive: 
   - mkdir PALMETTO
   - Download [your os]tar.gz.part-[aa,ab,ac,ad] file into PALMETTO
     - Ex: cat dist_macos_arm.tar.gz.part-* > dist_macos_arm.tar.gz
   - MacOS [ARM]: tar -zxvpf dist_macos_arm.tar.gz 
   - Linux [PI5]: tar -zxvpf dist_linux_arm.tar.gz 
   - Linux [X86]: tar -zxvpf dist_linux_x86.tar.gz

II. Running the System
--------_-------------
1. Start the GUI:
   ./palmetto.mac [--node-id 0] [-e passkey] &
   ./palmetto.pi5 &
   ./palmetto.lin &

   optional
     --node-id [0,1,2,etc] is useful in a mesh.
     -e [passkey] shared passkey for encryption. Must be the same on all nodes.

2. To view the log file run:
     cat audio_rx_demod.log

III. Hardware Requirements & Connections
--------------------------------------
- Transceiver: Tera TR-500 (for HAM) or TR-505 (for MURS). See other options. 
  - Radios.txt
- Interface: Digirig Mobile. About $55.00 each.
- Cables: 
  - Digirig Interface Cable for Icom HT (Gigaparts ZDR-ICOHT-CB). About $23.00 each.
  - USB-C to USB-B cable. About $18.00 each


You might have to debug the setup. Just use AI. Contact me at n4omg@pm.me
