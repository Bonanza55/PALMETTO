PALMETTO Digital Messaging System

Welcome to PALMETTO, a unified packet VHF/UHF radio transceiver 
and FSK demodulation package off grid radio operators.

MURS for non hams, and simplex [147.550] for hams. 

1. Installation & Setup
-----------------------
1. Extract Archive: 
   - mkdir PALMETTO
   - Copy [your os]tar.gz.part-[aa,ab,ac] file into PALMETTO
     - cat dist_macos_arm.tar.gz.part-* > dist_macos_arm.tar.gz
   - MacOS [ARM]: tar -zxvpf dist_macos_arm.tar.gz .
   - Linux [PI5]: tar -zxvpf dist_pi5_arm.tar.gz .
   - Linux [X86]: tar -zxvpf dist_linux_x86.tar.gz .
2. Install Dependencies:
   - macOS: Run "brew install portaudio"
   - Linux: Run "sudo apt install libportaudio2 portaudio19-dev"
3. Build Binaries (Choose Option 1 or Option 2):
   - Option 1: Run "make clean" followed by "make all" (See gcc install instructions).
       - Install_Gcc_Linux.txt
       - Install_Gcc_MacOS.txt
   - Option 2: Skip compilation and run the pre-built binaries.
4. Remove the *tar.gz.part-* files but keep the *.tar.gz file. 

2. Hardware Requirements & Connections
--------------------------------------
- Transceiver: Tera TR-500 (for HAM) or TR-505 (for MURS). See other options. 
  - Radios.txt
- Interface: Digirig Mobile. About $55.00 each.
- Cables: 
  - Digirig Interface Cable for Icom HT (Gigaparts ZDR-ICOHT-CB). About $23.00 each.
  - USB-C to USB-B cable. About $18.00 each

3. Running the System
---------------------
Start the background listener and graphical user interface using the following terminal commands:

1. Start the Listener:
   ./audio_rx_demod -v > audio_rx_demod.log 2>&1 &

2. Start the GUI:
   ./palmetto.mac &
   ./palmetto.pi5 &
   ./palmetto.lin &
