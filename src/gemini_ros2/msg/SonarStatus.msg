std_msgs/Header header

# Source Device: 2 = Beamformer, 4 = Data Acquisition Board
uint8 source_device

# Safety Flags (Beamformer Only)
uint16 shutdown_status
bool over_temperature_shutdown
bool out_of_water_shutdown

# Beamformer Temperatures (Celsius)
float64 bf_die_temp
float64 bf_pcb_temp
float64 bf_comms_temp
float64 bf_tx_temp
float64 bf_psu_temp

# Data Acquisition Temperatures (Celsius)
float64 da_pcb_temp
float64 da_afe0_top_temp
float64 da_afe0_bot_temp
float64 da_afe1_top_temp
float64 da_afe1_bot_temp
float64 da_afe2_top_temp
float64 da_afe2_bot_temp
float64 da_afe3_top_temp
float64 da_afe3_bot_temp