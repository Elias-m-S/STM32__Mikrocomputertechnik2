# Programmentwurf 2 — Mikrocomputertechnik

> STM32G474RE · EduShield · GY-521 · Power Modes · EEPROM · UART · I2C/DMA

---

## Überblick

Dieses Projekt ist die Umsetzung von **Programmentwurf 2** der Vorlesung *Mikrocomputertechnik*.  
Schwerpunkte sind Power-Management (Shutdown/Wakeup), nicht-flüchtiger Datenspeicher (emuliertes EEPROM) sowie serielle Kommunikation über I2C und UART mit DMA-Unterstützung.

**Plattform:** Nucleo Board mit STM32G474RE + EduShield  
**Sensor:** GY-521 (MPU-6050) Beschleunigungssensor  
**Toolchain:** ARM GNU Toolchain ≥ 12.3.Rel1  
**Sprache:** C (C99)

---

## Funktionsübersicht

### 🔋 Power Modes (`powermodes.c`)
| Verhalten | Beschreibung |
|---|---|
| Wakeup via Button | User Button B1 weckt den Prozessor aus dem Shutdown Mode |
| Wakeup via Timer | Nach 10 Sekunden automatisches Aufwachen |
| Shutdown | Beim zweiten Loslassen von B1 im RUN Mode |
| LED D1 | Blinkt mit 1 Hz im RUN Mode |
| LED D2 | Leuchtet bei Button-Wakeup |
| LED D3 | Leuchtet bei Timer-Wakeup |

### 💾 Datenspeicher (`storage.c`)
| Feature | Beschreibung |
|---|---|
| EEPROM-Emulation | STM eigener Emulator, erste zwei Pages der zweiten Flash Bank |
| Startup-Counter | 32-Bit, Identifier 1 — Anzahl der Startups seit Partitionierung |
| Laufzeit | 32-Bit, Identifier 2 — akkumulierte Laufzeit in ms |
| Persistierung | Bei Shutdown sowie alle 5 Sekunden |

### 📡 Beschleunigungssensor (`acceleration.c`)
| Feature | Beschreibung |
|---|---|
| Schnittstelle | I2C an PC8/PC9, Versorgung über PC5 |
| Konfiguration | Wird durch Taster SW2 im RUN State ausgelöst |
| Messbereiche | Drehrate ±1000 °/s, Linearbeschleunigung ±16g, Tiefpass 10 Hz |
| Auslesen | Sekündlich per DMA, solange LED D0 leuchtet |
| Shutdown | Spannungsversorgung und alle LEDs werden vor Shutdown deaktiviert |

### 🖥️ Datenaufbereitung & UART-Ausgabe (`processing.c`)
| Feature | Beschreibung |
|---|---|
| Ausgabeformat | `Drehrate: VAAA VBBB VCCC; Linear: VXX.X, VYY.Y, VZZ.Z` |
| Einheiten | Linearbeschleunigung in m/s², Drehrate in °/s |
| Vorzeichen | `+` für ≥ 0, `-` für < 0 |
| Max. Linearwert | 99.9 (absolute Beschränkung) |
| UART | LPUART an PA2/PA3, 115200 Baud, 8N1, DMA |

---

## Projektstruktur

```
/source
├── powermodes.c      # Power-Management: Shutdown, Wakeup, LEDs
├── storage.c         # EEPROM-Emulation, Laufzeit, Startup-Counter
├── acceleration.c    # I2C-Treiber, GY-521 Konfiguration & Auslesen (DMA)
├── processing.c      # Datenaufbereitung & LPUART-Ausgabe (DMA)
└── ...               # Basis-Framework (mitgeliefertes ZIP-Archiv)
FileList.mak          # Build-Konfiguration (C_SOURCES)
```

---

## Build

```bash
# Mit der ARM GNU Toolchain (≥ 12.3.Rel1) bauen
make all
```

Der generierte Code sowie die Build-Ausgabe sind im Archiv enthalten (siehe Lieferumfang).  
Peripherie wurde mit **CubeMX** konfiguriert.  
Quellcode ist mit **Astyle** formatiert, Dokumentation **Doxygen**-konform.

---

## Hardware-Setup

```
Nucleo STM32G474RE
├── EduShield
│   ├── LED D0  — GY-521 konfiguriert & aktiv
│   ├── LED D1  — RUN Mode Heartbeat (1 Hz)
│   ├── LED D2  — Wakeup durch Button
│   ├── LED D3  — Wakeup durch Timer
│   └── SW2     — GY-521 Konfiguration auslösen
├── User Button B1  — Wakeup / Shutdown-Trigger
└── GY-521
    ├── I2C  → PC8 (SCL), PC9 (SDA)
    ├── VCC  → PC5 (GPIO)
    └── UART → PA2 (TX), PA3 (RX) — LPUART
```

---

## Hinweise

- Die Verwendung von generativer KI ist gemäß Anforderung Q9 **nicht zulässig**.
- Quelltext darf **nicht** zwischen Prüfungsteilnehmern geteilt werden.
- Anforderungen Q1–Q11 (C99, Toolchain, Astyle, Doxygen, CubeMX) wurden eingehalten.
