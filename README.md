# Core-0 Firmware V1.1 - Gemigreerde Bestanden

## 📁 Overzicht

| Bestand | Grootte | Beschrijving |
|---------|---------|--------------|
| `core0_config.h` | 18.5 KB | Centrale configuratie header |
| `core0_config.c` | 16.7 KB | Config implementatie + defaults |
| `core0_sensing.c` | 24.9 KB | Hall sensor capture (gemigreerd) |
| `core0_filtered.c` | 15.2 KB | Filter layer (gemigreerd) |
| `core0_filtered.h` | 4.4 KB | Filter header |
| `core0_link.c` | 15.2 KB | UART/framing (gemigreerd) |
| `core0_link.h` | 5.5 KB | Link header |

## ✅ Wat is gewijzigd

### 1. Alle hardcoded waarden → CFG_* macros

**Voorheen (verspreid over bestanden):**
```c
// core0_sensing.c
#define A_low_th  1610
#define A_high_th 2540

// core0_filtered.c  
fr->dvdt_min_q15 = 15;

// core0_link.c
#define UART_BAUD 115200
```

**Nu (alles in core0_config.h):**
```c
// Eén plek voor alle parameters
CFG_SENS_A.low_th      // Sensor A threshold
CFG_FILT.dvdt_min_q15  // Filter drempel
CFG_HW.uart_baud       // UART snelheid
```

### 2. Run Modes (compile-time → runtime)

**Voorheen:**
```c
#define DBG_BYPASS_EMIT 1  // Compile-time switch
```

**Nu:**
```c
g_cfg.runmode = RUNMODE_DEBUG_BYPASS;  // Runtime
// Of gebruik macro:
if (IS_DEBUG_BYPASS) { ... }
```

### 3. V1.1 Feature Flags

Nieuwe features achter flags (default UIT = V1.0 gedrag):

| Flag | Default | Effect |
|------|---------|--------|
| `CFG_DIR.enabled` | `false` | Direction hint uit A↔B timing |
| `CFG_SENS.use_improved_fit` | `false` | Lineaire fit error berekening |
| `CFG_SENS.use_fast_fit` | `false` | Snelle 3-punt fit variant |

## 🚀 Integratie Stappen

### Stap 1: Bestanden kopiëren

```
components/core0/
├── core0_config.h    ← NIEUW
├── core0_config.c    ← NIEUW
├── core0_sensing.c   ← VERVANGEN
├── core0_filtered.h  ← VERVANGEN
├── core0_filtered.c  ← VERVANGEN
├── core0_link.h      ← VERVANGEN
└── core0_link.c      ← VERVANGEN
```

### Stap 2: CMakeLists.txt updaten

```cmake
idf_component_register(
    SRCS 
        "core0_config.c"    # ← Toevoegen
        "core0_sensing.c"
        "core0_filtered.c"
        "core0_link.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_adc
)
```

### Stap 3: Compileren

```bash
idf.py build
```

### Stap 4: V1.1 Features Inschakelen (optioneel)

In `core0_config.h`:
```c
// Verander defaults:
#define DIR_ENABLED_DEFAULT             true   // Was: false
#define SENS_USE_IMPROVED_FIT_DEFAULT   true   // Was: false
```

Of runtime:
```c
g_cfg.direction.enabled = true;
g_cfg.sensing.use_improved_fit = true;
```

## 🔍 Belangrijke Wijzigingen per Bestand

### core0_sensing.c

| Oud | Nieuw |
|-----|-------|
| `A_low_th` | `CFG_SENS_A.low_th` |
| `baseline_A = 2048.0f` | `baseline_A = CFG_SENS_A.baseline_init` |
| `POOL_MIN_DT_US` | `CFG_SENS.pool_min_dt_us` |
| `noise * 5.0f` | `compute_fit_error_v10()` of `_v11()` |
| `dir = DIR_NONE` | `determine_direction_v11()` als enabled |

### core0_filtered.c

| Oud | Nieuw |
|-----|-------|
| `fr->dvdt_min_q15 = 15` | `CFG_FILT.dvdt_min_q15` |
| `fr->target_evps = 150` | `CFG_FILT.target_evps` |
| `1.5f` (strong mult) | `CFG_FILT.qlevel.dvdt_strong_mult` |
| `cost = 0.5f` | `CFG_FILT.token_cost.strong` |

### core0_link.c

| Oud | Nieuw |
|-----|-------|
| `UART_BAUD 115200` | `CFG_HW.uart_baud` |
| `link_init_tx(&g_tx, 512)` | `link_init_tx(&g_tx, CFG_LINK.tx_queue_len)` |
| `#if USE_FILTER_STATS` | `if (CFG_LINK.use_filter_stats)` |

## 📊 Verwacht Gedrag

### Met V1.0 defaults (feature flags uit)

- **dir_hint**: Altijd `DIR_NONE`
- **fit_error**: `noise × 5.0f`
- **Exact hetzelfde gedrag als originele firmware**

### Met V1.1 features aan

- **dir_hint**: `DIR_CW` of `DIR_CCW` bij pairs, `DIR_NONE` bij singles
- **fit_error**: Lineaire regressie residual (0-255 schaal)
- **Verwachte direction ratio**: >8:1 bij schone rotatie

## ⚠️ Let Op

1. **Include volgorde**: `core0_config.h` moet EERST geïnclude worden
2. **Init volgorde**: `core0_config_init()` moet EERST aangeroepen worden in `app_main()`
3. **Geen ESP-IDF dependencies in config.h**: Alleen standaard C types

## 🧪 Testen

Na flashen, check serial output:

```
I (xxx) S02-Core0: S02-Core0-Sensing v1.1.0 (mode: PRODUCTION)
I (xxx) S02-Core0: V1.1 Features: dir_hint=OFF, improved_fit=OFF
```

Of met features aan:

```
I (xxx) S02-Core0: V1.1 Features: dir_hint=ON, improved_fit=ON
```
