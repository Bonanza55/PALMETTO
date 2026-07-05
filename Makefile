# --- Compiler and Flags ---
CC       := clang
CFLAGS   := -Wall -Wextra -O3 -std=c99 -arch arm64
LDFLAGS  := -lm
PTHREAD_LDFLAGS := -lpthread

# Homebrew paths for Apple Silicon C dependencies
BREW_PREFIX := /opt/homebrew
SDR_CFLAGS  := -I$(BREW_PREFIX)/include -I$(BREW_PREFIX)/include/rtl-sdr
SDR_LDFLAGS := -L$(BREW_PREFIX)/lib -lrtlsdr

TUI_LDFLAGS := -L$(BREW_PREFIX)/opt/ncurses/lib -lncurses -lm
TUI_CFLAGS  := -I$(BREW_PREFIX)/opt/ncurses/include

# --- Targets and Sources ---
MOD_TARGET  := fsk_mod
DEMOD_TARGET:= fsk_demod
TUI_TARGET  := fsk_tui
RX_TARGET   := fsk_rx
RXD_TARGET  := fsk_rxd
DSP_TARGET  := fsk_dsp

MOD_SRC     := fsk_mod.c
DEMOD_SRC   := fsk_demod.c
TUI_SRC     := fsk_tui.c
RX_SRC      := fsk_rx.c
RXD_SRC     := fsk_rxd.c
DSP_SRC     := fsk_dsp.c

# --- Default Build Rule ---
.PHONY: all
all: $(MOD_TARGET) $(DEMOD_TARGET) $(TUI_TARGET) $(RX_TARGET) $(RXD_TARGET) $(DSP_TARGET)

# --- Modulator Target ---
$(MOD_TARGET): $(MOD_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(MOD_SRC) -o $(MOD_TARGET) $(LDFLAGS)

# --- Demodulator Target ---
$(DEMOD_TARGET): $(DEMOD_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(DEMOD_SRC) -o $(DEMOD_TARGET) $(LDFLAGS)

# --- Curses TUI Target ---
$(TUI_TARGET): $(TUI_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(TUI_CFLAGS) $(TUI_SRC) -o $(TUI_TARGET) $(TUI_LDFLAGS)

# --- Native RTL-SDR Receiver Target ---
$(RX_TARGET): $(RX_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(SDR_CFLAGS) $(RX_SRC) -o $(RX_TARGET) $(LDFLAGS) $(SDR_LDFLAGS)

# --- Squelch-Gated RX Capture Daemon Target ---
$(RXD_TARGET): $(RXD_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(SDR_CFLAGS) $(RXD_SRC) -o $(RXD_TARGET) $(LDFLAGS) $(SDR_LDFLAGS) $(PTHREAD_LDFLAGS)

# --- DSP FM Bridging Demodulator Target ---
$(DSP_TARGET): $(DSP_SRC)
	@echo "[CC] $< -> $@"
	@$(CC) $(CFLAGS) $(DSP_SRC) -o $(DSP_TARGET) $(LDFLAGS)

# --- Clean Rule ---
.PHONY: clean
clean:
	@echo "[CLEANING]"
	@rm -f $(MOD_TARGET) $(DEMOD_TARGET) $(TUI_TARGET) $(RX_TARGET) $(RXD_TARGET) $(DSP_TARGET)
