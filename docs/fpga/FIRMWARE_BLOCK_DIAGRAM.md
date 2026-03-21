# CaribouLite FPGA Firmware Block Diagram

## Target: Lattice iCE40LP1K-QN84 (1280 LUTs)

---

## System Overview

```
                        ┌─────────────────────────────────────────────────────────┐
                        │              CaribouLite FPGA (iCE40LP1K)               │
                        │                                                         │
  125 MHz Crystal ──────┤►i_glob_clock                                            │
                        │    │                                                    │
                        │    ▼                                                    │
                        │ ┌──────────────┐                                        │
                        │ │ CLOCK DIST.  │  w_clock_sys = i_glob_clock ÷ 2        │
                        │ │ (÷2 toggle)  ├──► 64 MHz system clock domain          │
                        │ └──────────────┘                                        │
                        │                                                         │
                        │                                                         │
  MODEM (AT86RF215)     │                              RASPBERRY PI               │
  ══════════════════    │                              ═══════════════             │
                        │                                                         │
  LVDS RX Clock ────────┤►i_iq_rx_clk_p                                           │
  (64 MHz DDR)          │    │                                                    │
                        │    ▼                                                    │
                        │ ┌──────────┐     lvds_clock_buf (64 MHz)                │
                        │ │ SB_GB    ├──►──────────────────────────                │
                        │ │ (Global  │     (global clock buffer)                  │
                        │ │  Buffer) │                                            │
                        │ └──────────┘                                            │
                        │                                                         │
                        └─────────────────────────────────────────────────────────┘
```

---

## Clock Domains

| Domain | Frequency | Source | Used By |
|--------|-----------|--------|---------|
| `w_clock_sys` | 64 MHz | `i_glob_clock` ÷ 2 | SPI I/F, SYS_CTRL, IO_CTRL, SMI_CTRL, FIFO read/write |
| `lvds_clock_buf` | 64 MHz | Modem LVDS RX clock (SB_GB promoted) | LVDS_RX, LVDS_TX, FIFO write/read |
| `i_spi_sck` | ~5 MHz | RPi SPI master | SPI_SLAVE bit shifting only |
| `i_smi_soe_se` | ~16 MHz | RPi SMI read strobe | SMI_CTRL RX byte clocking |
| `i_smi_swe_srw` | ~16 MHz | RPi SMI write strobe | SMI_CTRL TX byte clocking |

---

## Complete Internal Architecture

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                           FPGA INTERNAL LOGIC                                    │
│                                                                                  │
│  FROM MODEM                                                                      │
│  ══════════                                                                      │
│                                                                                  │
│  i_iq_rx_09_p ──►┌────────────┐                                                 │
│  (0.9 GHz ch)    │  SB_IO DDR │──► w_lvds_rx_09_d0 (rising)                     │
│  (positive)      │  INPUT     │──► w_lvds_rx_09_d1 (falling)                     │
│                  └────────────┘        │                                         │
│                                        ▼                                         │
│                                  ┌───────────┐                                   │
│                                  │ LVDS_RX   │  ◄── lvds_clock_buf              │
│                                  │ (0.9 GHz) │                                   │
│                                  │           │  States: IDLE→I_PHASE→Q_PHASE     │
│                                  │ Sync det: │  Sync: waits for 2'b10 (I-sync)  │
│                                  │ 2'b10 = I │  then 2'b01 (Q-sync)             │
│                                  │ 2'b01 = Q │  Collects 32-bit I/Q frame       │
│                                  └─────┬─────┘                                   │
│                                        │ 32-bit frame + push                     │
│                                        ▼                                         │
│  i_iq_rx_24_n ──►┌────────────┐  ┌─────────┐                                    │
│  (2.4 GHz ch)    │  SB_IO DDR │  │  CHANNEL │     ┌──────────────────┐           │
│  (inverted!)     │  INPUT     │  │   MUX    ├────►│   RX FIFO        │           │
│                  └────────────┘  │ (a2/a3   │     │  (complex_fifo)  │           │
│                       │          │  select)  │     │                  │           │
│                       ▼          └─────────┘     │  ADDR_WIDTH=10   │           │
│                 ┌───────────┐        ▲            │  = 1024 samples  │           │
│                 │ LVDS_RX   │        │            │                  │           │
│                 │ (2.4 GHz) │────────┘            │  WR clk: lvds   │           │
│                 │           │  32-bit frame        │  RD clk: sys    │           │
│                 │ NOTE: data│                     │                  │           │
│                 │ inverted  │                     │  Gray-code CDC   │           │
│                 │ in top.v  │                     │  2-FF sync       │           │
│                 └───────────┘                     └────────┬─────────┘           │
│                                                            │                     │
│                                                            │ 32-bit I/Q          │
│                                                            ▼                     │
│                                                   ┌────────────────┐             │
│                                                   │   SMI_CTRL     │             │
│  TO RASPBERRY PI                                  │                │             │
│  ═══════════════                                  │  RX: 4-byte    │             │
│                                                   │  serializer    │             │
│  io_smi_data[7:0] ◄──────────────────────────────┤  (SOE strobe   │             │
│  (bidirectional)  ──────────────────────────────►│   clocked)     │             │
│                                                   │                │             │
│  i_smi_soe_se    ──────────────────────────────►│  TX: 4-byte    │             │
│  (RX strobe)                                      │  deserializer  │             │
│                                                   │  with SOF/EOF  │             │
│  i_smi_swe_srw   ──────────────────────────────►│  frame markers │             │
│  (TX strobe)                                      │                │             │
│                                                   │  IOC regs:     │             │
│  o_smi_read_req  ◄────────────────────────────────┤  - version     │             │
│  (!rx_fifo_empty)                                 │  - fifo_status │             │
│                                                   │  - ch_select   │             │
│  o_smi_write_req ◄────────────────────────────────┤  - dir_select  │             │
│  (!tx_fifo_full)                                  └───────┬────────┘             │
│                                                           │                      │
│                                                           │ 32-bit I/Q           │
│                                                           ▼                      │
│                                                   ┌──────────────────┐           │
│                                                   │   TX FIFO        │           │
│                                                   │  (complex_fifo)  │           │
│                                                   │                  │           │
│                                                   │  ADDR_WIDTH=10   │           │
│                                                   │  = 1024 samples  │           │
│                                                   │                  │           │
│                                                   │  WR clk: sys     │           │
│                                                   │  RD clk: lvds    │           │
│                                                   │                  │           │
│                                                   │  Gray-code CDC   │           │
│                                                   │  2-FF sync       │           │
│                                                   └────────┬─────────┘           │
│                                                            │                     │
│                                                            ▼                     │
│  TO MODEM                                          ┌───────────┐                 │
│  ═════════                                         │ LVDS_TX   │                 │
│                                                    │           │                 │
│  o_iq_tx_p ◄──┌────────────┐◄──────────────────────┤ States:   │                 │
│               │  SB_IO DDR │                       │ IDLE →    │                 │
│  o_iq_tx_n ◄──│  OUTPUT    │                       │ TX_FRAME →│                 │
│               └────────────┘                       │ TX_GAP    │                 │
│                                                    │           │                 │
│  o_iq_tx_clk_p ◄──┌────────────┐                   │ 10 sync   │                 │
│                    │  SB_IO DDR │◄── lvds_clock_buf │ preamble  │                 │
│  o_iq_tx_clk_n ◄──│  CLK GEN   │                   │ frames    │                 │
│                    └────────────┘                   └───────────┘                 │
│                                                                                  │
│                                                                                  │
│  SPI CONTROL PATH                                                                │
│  ════════════════                                                                │
│                                                                                  │
│  i_sck  ──────►┌────────────┐    ┌──────────┐    ┌───────────┐                   │
│  i_mosi ──────►│ SPI_SLAVE  ├───►│ SPI_IF   ├───►│ SYS_CTRL  │                   │
│  i_ss   ──────►│            │    │          │    │           │                   │
│  o_miso ◄──────┤ 5 MHz      │    │ Opcodes: │    │ Registers:│                   │
│               └────────────┘    │ R/W +    │    │ - version │                   │
│                                  │ CS[1:0]+ │    │ - tx_gap  │                   │
│                                  │ IOC[4:0] │    │ - sync_sel│                   │
│                                  │          │    │ - debug   │                   │
│                                  │ Routes to├───►│ - soft_syn│                   │
│                                  │ 4 modules│    └───────────┘                   │
│                                  │          │                                    │
│                                  │          ├───►┌───────────┐                   │
│                                  │          │    │ IO_CTRL   │──► RF switches    │
│                                  │          │    │           │──► LEDs           │
│                                  │          │    │ RF modes: │──► LNA ctrl       │
│                                  │          │    │ lo_pwr,   │──► Mixer ctrl     │
│                                  │          │    │ bypass,   │──► PMOD pins      │
│                                  │          │    │ rx_lpf,   │                   │
│                                  │          │    │ rx_hpf,   │                   │
│                                  │          │    │ tx_lpf,   │                   │
│                                  │          │    │ tx_hpf    │                   │
│                                  │          │    └───────────┘                   │
│                                  │          │                                    │
│                                  │          ├───► SMI_CTRL (IOC regs)            │
│                                  └──────────┘                                    │
│                                                                                  │
│                                                                                  │
│  SYNC MUX (configurable per channel)                                             │
│  ═══════════════════════════════════                                              │
│                                                                                  │
│  sys_ctrl.sync_type_xx ──► MUX ──► lvds_rx/tx.i_sync_input                       │
│                             ▲                                                    │
│                    ┌────────┴────────┐                                            │
│                    │                 │                                            │
│              Internal sync     PMOD external sync                                │
│              (soft_sync reg)   (io_pmod_in[3:0])                                 │
│                                                                                  │
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## RX Data Path Detail

### LVDS_RX State Machine (per channel)

```
                    ┌──────────┐
                    │   IDLE   │◄──────────────────────────┐
                    │  (2'b00) │                           │
                    └────┬─────┘                           │
                         │                                 │
                    DDR data == 2'b10                      │
                    (I-sync marker)                        │
                         │                                 │
                         ▼                                 │
                    ┌──────────┐                           │
                    │ I_PHASE  │  Collect 12 DDR pairs     │
                    │  (2'b01) │  = 24 bits of I data      │
                    │          │  phase_count: 12→0        │
                    └────┬─────┘                           │
                         │                                 │
                    phase_count==0                         │
                    AND DDR data == 2'b01                  │
                    (Q-sync marker)                        │
                         │                                 │
                         ▼                                 │
                    ┌──────────┐                           │
                    │ Q_PHASE  │  Collect 6 DDR pairs      │
                    │  (2'b11) │  = 12+1 bits of Q data    │
                    │          │  phase_count: 6→0         │
                    └────┬─────┘                           │
                         │                                 │
                    phase_count==0                         │
                    → push 32-bit frame to FIFO            │
                    → return to IDLE ──────────────────────┘
```

### 32-bit RX Frame Format

```
 Bit: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
      ├──┤  ├──────────────────────────────────────────────────┤  ├──────────────────┤  ├──┤  ├──┤
      I-sync          24 I-phase bits (12 DDR pairs)              4 Q bits           1Q  sync
      2'b10           shifted in from modem                       bits               bit  input
```

---

## TX Data Path Detail

### LVDS_TX State Machine

```
                    ┌──────────┐
              ┌────►│   IDLE   │◄──── TX disabled
              │     │          │      (emit sync/zero frames)
              │     └────┬─────┘
              │          │
              │     TX enabled + FIFO not empty
              │     + sync preamble sent (10 frames)
              │          │
              │          ▼
              │     ┌──────────┐
              │     │ TX_FRAME │  Shift out 32-bit frame
              │     │          │  as 16 × 2-bit DDR pairs
              │     │          │  phase_count: 15→0
              │     └────┬─────┘
              │          │
              │     phase_count==0
              │          │
              │          ▼
              │     ┌──────────┐
              └─────┤ TX_GAP   │  Emit i_sample_gap zero frames
                    │          │  (inter-symbol spacing)
                    └──────────┘

  Special: LOOPBACK state (debug) continuously emits test pattern
```

### 32-bit TX Frame Format

```
 Bit: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
      ├──┤  ├──────────────────────────────────────┤  ├──┤  ├──┤  ├──────────────────────────────────┤
      2'b10          I[13:0] data                     TX_EN 2'b01          Q[12:0] data
      I-mark         (14 bits)                        (1=on) Q-mark        (13 bits)
```

---

## SMI Controller Byte Serialization

### RX Direction (FPGA → RPi)

```
                  sys_clk domain                  SOE domain (~16 MHz)
                  ──────────────                  ────────────────────

  RX FIFO ──► o_rx_fifo_pull ──► r_pull_done     SOE strobe edges:
  (rd_clk =        │                  │
   sys_clk)        │              ┌───▼────┐      Edge 0: emit byte[7:0]
                   │              │HOLDING │      Edge 1: emit byte[15:8]
                   │              │REGISTER│             + trigger pull
                   │              │(sys_clk│      Edge 2: emit byte[23:16]
                   │              │ stable)│      Edge 3: emit byte[31:24]
                   │              └───┬────┘             + latch r_rx_holding
                   │                  │                    into r_fifo_pulled_data
                   │                  │           Edge 4: emit next[7:0] ...
                   │                  │
                   │              r_rx_holding ──► r_fifo_pulled_data
                   │              (safe: stable    (SOE domain local copy)
                   │               for 2+ SOE
                   │               cycles before
                   │               SOE reads it)

  CDC safety: r_rx_holding captures FIFO output in sys_clk, then holds
  steady.  The SOE domain reads it only at byte 3 — many sys_clk cycles
  after the last update — eliminating the metastability window.
```

### TX Direction (RPi → FPGA)

```
  io_smi_data[7:0] ──► 4-byte deserializer ──► frame validator ──► TX FIFO

  SWE strobe    Byte input      Validation
  ──────────    ──────────      ──────────
  Edge 0        b0 (MSB=1 SOF)  Must have SOF marker
  Edge 1        b1 (MSB=0)      Data byte
  Edge 2        b2 (MSB=0)      Data byte
  Edge 3        b3 (MSB=0 EOF)  Assemble I/Q, push to FIFO

  Frame assembly:
    I[12:0] = {b0[4:0], b1[6:0], b2[6]}
    Q[12:0] = {b2[5:0], b3[6:0]}
```

---

## Clock Domain Crossing (CDC) Strategy

```
  ┌─────────────────┐          ┌─────────────────┐
  │  LVDS Clock     │          │  System Clock    │
  │  Domain         │          │  Domain          │
  │  (64 MHz)       │          │  (64 MHz)        │
  │                 │          │                  │
  │  LVDS_RX ──────►├── FIFO ──├──► SMI_CTRL RX   │
  │                 │  (Gray   │                  │
  │  LVDS_TX ◄──────├── code   ├──◄ SMI_CTRL TX   │
  │                 │  ptrs +  │                  │
  │                 │  2-FF    │                  │
  │                 │  sync)   │                  │
  └─────────────────┘          └─────────────────┘

  Additional CDC synchronizers:
  ─────────────────────────────
  • smi_ctrl: SOE/SWE strobes → 2-FF sync to sys_clk
  • smi_ctrl: data bus → 3-stage pipeline (d_q1→d_q2→d_q3)
  • lvds_tx:  tx_state, debug_lb, fifo_empty → 2-FF sync to lvds_clk
  • spi_slave: rx_done pulse → 2-FF sync to sys_clk
  • spi_slave: SCK edges → 3-stage pipeline for edge detection
```

---

## SPI Command Interface

```
  RPi SPI Master (5 MHz)
       │
       ▼
  ┌──────────┐     ┌──────────┐
  │SPI_SLAVE │────►│ SPI_IF   │
  │          │     │          │
  │ Byte-    │     │ Opcode:  │
  │ level    │     │ [7]   = R/W
  │ shift    │     │ [6:5] = CS (module select)
  │ register │     │ [4:0] = IOC (register addr)
  └──────────┘     └────┬─────┘
                        │
              ┌─────────┼─────────┬─────────┐
              ▼         ▼         ▼         ▼
         CS=00      CS=01     CS=10     CS=11
        ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
        │SYS_CTRL│ │IO_CTRL │ │SMI_CTRL│ │Reserved│
        │        │ │        │ │        │ │        │
        │0: ver  │ │0: ver  │ │0: ver  │ │        │
        │1: sys_v│ │1: dig  │ │1: fifo │ │        │
        │2: mfg  │ │2: pin  │ │2: ch   │ │        │
        │3: err  │ │3: pmod │ │3: dir  │ │        │
        │5: debug│ │4: pval │ │        │ │        │
        │6: gap  │ │5: rfpin│ │        │ │        │
        │7: sync │ │        │ │        │ │        │
        └────────┘ └────────┘ └────────┘ └────────┘
```

---

## Pin Map Summary

### LVDS Modem Interface
| Signal | Pin | Direction | Mode | Notes |
|--------|-----|-----------|------|-------|
| `i_iq_rx_clk_p` | A3 | Input | LVDS | 64 MHz DDR clock from modem |
| `i_iq_rx_09_p` | A4 | Input | LVDS DDR | 0.9 GHz RX data (positive) |
| `i_iq_rx_24_n` | A2 | Input | LVDS DDR | 2.4 GHz RX data (**inverted in top.v**) |
| `o_iq_tx_p` | B4 | Output | DDR | TX data positive |
| `o_iq_tx_n` | A5 | Output | DDR | TX data negative |
| `o_iq_tx_clk_p` | A10 | Output | DDR | TX clock positive |
| `o_iq_tx_clk_n` | B8 | Output | DDR | TX clock negative |

### SMI Bus (to Raspberry Pi)
| Signal | Pin | Direction | Notes |
|--------|-----|-----------|-------|
| `io_smi_data[0:7]` | A16,B11,B10,B12,B14,A20,A13,A14 | Bidir | 8-bit data bus |
| `i_smi_a2` | A48 | Input | TX/RX direction select |
| `i_smi_a3` | A47 | Input | RX channel select |
| `i_smi_soe_se` | B15 | Input | Read strobe |
| `i_smi_swe_srw` | B13 | Input | Write strobe |
| `o_smi_write_req` | A19 | Output | TX FIFO not full |
| `o_smi_read_req` | B19 | Output | RX FIFO not empty |

---

## Verilog Source Files

| File | Module | LOC | Purpose |
|------|--------|-----|---------|
| `top.v` | `top` | ~300 | Top-level: clock gen, I/O primitives, interconnect |
| `smi_ctrl.v` | `smi_ctrl` | ~350 | SMI bus controller, byte serialization, frame assembly |
| `lvds_rx.v` | `lvds_rx` | ~120 | RX deserializer with I/Q sync detection |
| `lvds_tx.v` | `lvds_tx` | ~200 | TX serializer with sync preamble and gap control |
| `complex_fifo.v` | `complex_fifo` | ~100 | Dual-clock FIFO with Gray-code CDC |
| `spi_if.v` | `spi_if` | ~120 | SPI command parser and module router |
| `spi_slave.v` | `spi_slave` | ~100 | SPI bit-level shift register |
| `sys_ctrl.v` | `sys_ctrl` | ~100 | System config registers (debug, sync, gap) |
| `io_ctrl.v` | `io_ctrl` | ~150 | I/O and RF frontend control |
