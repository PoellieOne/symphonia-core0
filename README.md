# Symphonia Core-0
Version: Mirror v1.0 (Gist-based)

Core-0 firmware modules for the Symphonia / S02 project.

This repository contains the low-level firmware layers that handle:

- **core0_sensing**: raw sensor sampling and basic signal acquisition
- **core0_filtered**: filtering and quality evaluation of sensor data
- **core0_link**: transport/link layer to higher-level awareness (Core-1, S02)

## Gist Mirrors (public, shared code crystals)

- `src/core0_sensing.c`
	https://gist.githubusercontent.com/PoellieOne/f83af4d1a8e590c7d0898d3f99a4cddd/raw/1e449d13e724c237b108dddbddbe16fd02fb2cf9/core0_sensing.c
- `src/core0_filtered.c`
	https://gist.githubusercontent.com/PoellieOne/4417cfe659bd0afc87f97dea4fa5da06/raw/a33669627983451cdcc2eea4295e488eeb7e90a3/core0_filtered.c
- `src/core0_link.c`
	https://gist.githubusercontent.com/PoellieOne/3bc4a173ab8e14297e3c7c8785a4eeca/raw/95790dde77dccc38238cef47775f550e0559299a/core0_link.c
- `include/core0_link.h`
	https://gist.githubusercontent.com/PoellieOne/b473c6a821509434a0304cc0e6355210/raw/5fbbc3b08c3990b10cbe3580b2f546bca4a7ebde/core0_link.h
- `include/core0_filtered.h`
	https://gist.githubusercontent.com/PoellieOne/b3b4b13cf83d82be0f05f214eb7090cd/raw/38d5476b1c3a025ec387acbd08d59f885eab1a8a/core0_filtered.h

GitHub is the single source of truth for these Core-0 modules.
All analysis and debugging with SoRa/Sophia should refer to this repository.
