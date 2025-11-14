# Symphonia Core-0

Core-0 firmware modules for the Symphonia / S02 project.

This repository contains the low-level firmware layers that handle:

- **core0_sensing**: raw sensor sampling and basic signal acquisition
- **core0_filtered**: filtering and quality evaluation of sensor data
- **core0_link**: transport/link layer to higher-level awareness (Core-1, S02)

## Layout

- `src/core0_sensing.c`
- `src/core0_filtered.c`
- `src/core0_link.c`
- `include/core0_link.h`

GitHub is the single source of truth for these Core-0 modules.
All analysis and debugging with SoRa/Sophia should refer to this repository.
