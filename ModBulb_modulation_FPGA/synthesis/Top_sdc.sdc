# Top Level Design Parameters

# Clocks

create_clock -name {Top|CLK} -period 50.000000 -waveform {0.000000 25.000000} CLK
create_clock -name {Top|spi_sck_i} -period 50.000000 -waveform {0.000000 25.000000} spi_sck_i

# False Paths Between Clocks


# False Path Constraints


# Maximum Delay Constraints


# Multicycle Constraints


# Virtual Clocks
# Output Load Constraints
# Driving Cell Constraints
# Wire Loads
# set_wire_load_mode top

# Other Constraints
