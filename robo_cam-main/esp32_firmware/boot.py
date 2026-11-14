# ==============================================================================
# RiceTrack Rover — ESP32 Boot Configuration (boot.py)
# ==============================================================================
# Target MCU: ESP32-WROOM-32 DevKit V1
# Sets CPU clock to 240 MHz and suppresses OS debug logs on UART0.
# ==============================================================================

try:
    import machine
    machine.freq(240000000)  # 240 MHz max performance
except (ImportError, AttributeError):
    pass

try:
    import esp
    esp.osdebug(None)  # Disable vendor OS debug output on UART0
except (ImportError, AttributeError):
    pass

try:
    import gc
    gc.collect()
except Exception:
    pass
