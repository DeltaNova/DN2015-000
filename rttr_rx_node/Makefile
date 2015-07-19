# Project Specific Makefile
# =========================

# NOTE: Whitespace chars at end of line can result in invalid entries


# Project Variables
# =================

# run 'make boards' for options

# Atmega 328p, 3.3V @ 8MHz
BOARD := pro328
# Atmega 328p, 5V @ 16MHz
#BOARD := pro5v328
#BOARD := atmega328

# Adjust library path as required (space separated)
LIBRARYPATH := /usr/share/arduino/libraries ~/Git/RFM69W/libraries

# Include Arduino Makefile
# ========================
include ~/bin/arduino.mk
# Arduino Makefile downloaded from:
# http://ed.am/dev/make/arduino-mk

# Override some of the included Makefile settings
# ===============================================

# Add '-g' to compiler flag to embed debug info (doesn't end up in .hex)
CPPFLAGS += -g
# Generate program .map at link stage
LINKFLAGS += -Wl,-Map,$(TARGET).map
ICSPFLAGS += -c usbtiny -p m328p
# Locate avr-objdump on system
OBJDUMP := $(call findsoftware,avr-objdump)

# Create Additional Targets
.PHONY: list icsp icsp_test

# Create a listing of asm and C source code
list: $(TARGET).elf
	@echo "Generating Listing"
	$(OBJDUMP) -h -S $(TARGET).elf > $(TARGET).lst

icsp: $(TARGET).hex
	@echo "Upload via ICSP"
	$(AVRDUDE) $(ICSPFLAGS) -v -e -U flash:w:$(TARGET).hex -U lock:w:0x0f:m

icsp_test:
	@echo "ICSP Comms Test"
	$(AVRDUDE) $(ICSPFLAGS)

# Override clean target to remove additonal files
clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET).elf $(TARGET).hex $(ARDUINOLIB) *~
	rm -rf .lib .dep
	rm -f $(TARGET).map $(TARGET).lst
