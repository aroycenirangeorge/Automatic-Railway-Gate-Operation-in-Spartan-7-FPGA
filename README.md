# Automatic Railway Gate Control System using Spartan7 FPGA
---

## 1. Problem Statement

### 1.1 Real-World Context

Railway level crossings are critical safety points where road vehicles and trains intersect. Manual gate operation can cause delays, human error, and unsafe conditions. An automatic railway gate system uses sensors and control logic to manage gate movement in real time, improving safety, reducing accidents, and ensuring smooth traffic flow without depending on human operators.

### 1.2 Project Objective

Design and implement an automatic railway gate control system using Verilog HDL. The system should:

* Detect the presence of an approaching train.
* Close the gate before the train reaches the crossing.
* Keep the gate closed while the train passes.
* Reopen the gate once the train exits.
* Provide visual indicators (red/green lights) to guide road traffic.

### 1.3 Key Modules in the Design

1. **Sensor Simulation (Entry/Exit):** Simulates proximity sensors on the track to detect train arrival and departure.
2. **FSM Controller:** A finite-state machine processes sensor inputs and controls gate motor, signal lights, and display logic.
3. **Signal Lights (Red/Green):** Visual indicators for road users:

   * **Green:** No train, safe to cross.
   * **Red:** Train approaching or crossing in progress.
4. **7-Segment Display (optional):** Displays a motor/activity value or countdown for demonstration.

### 1.4 Relevant Verilog Concepts Used

* **FSM design:** State registers, next-state logic, and output logic.
* **Sequential logic:** Flip-flops and clocked state transitions.
* **Timing control:** Clock-based refresh for 7-segment display and simple timing counters.
* **Combinational logic:** Decoding BCD values and controlling outputs.

---

## 2. Design and Methodology

### 2.1 Block Diagram

<img width="578" height="686" alt="image" src="https://github.com/user-attachments/assets/4f1767cd-2500-4282-be1c-a71bab426a7a" />

### 2.2 Functional Description

* **IDLE:** No train detected. Gate remains open and green light is ON. System waits for the entry sensor to trigger.
* **INCOMING:** Entry sensor detects an approaching train. Red light turns ON and motor starts closing the gate.
* **CROSSING:** Gate is fully closed while the train crosses. System remains in this state until the exit sensor activates.
* **OUTGOING:** Exit sensor indicates the train has cleared the crossing. Motor opens the gate and the system returns to IDLE.

### 2.3 FSM State Diagram

<img width="972" height="218" alt="image" src="https://github.com/user-attachments/assets/694e8071-fc59-4db2-b991-9680c48ffb83" />

### 2.4 Design Flow

1. Define requirements and state machine behavior.
2. Model the FSM in Verilog (state encoding, next-state logic, outputs).
3. Add peripherals: 7-segment refresh, motor simulation register, LED signals.
4. Simulate using a testbench and waveform viewer (e.g., Vivado simulator).
5. Synthesize and implement on Spartan-7 (pin constraints via XDC file).
6. Test on hardware and document results.

---

## 3. Verilog Coding & Implementation

### Inputs

* `clk` — system clock.
* `rst` — synchronous/asynchronous reset (active high in this design).
* `entry` — sensor input indicating train arrival.
* `exit` — sensor input indicating train departure.

### Outputs

* `red` — red signal LED (stop vehicles).
* `green` — green signal LED (allow vehicles).
* `an` — 4-bit anode control for multiplexed 7-segment display.
* `seg` — 7-segment segment outputs (7 bits).

---

### 3.1 RTL Code — `train_signal.v`

```verilog
`timescale 1ns / 1ps
module train_signal(clk, rst, entry, exit, red, green, an, seg);
    input clk, rst, entry, exit;
    output reg red, green;
    output reg [3:0] an;
    output reg [6:0] seg;

    reg [2:0] current, next;
    reg [12:0] motor;

    parameter IDLE      = 3'b000,
              INCOMING  = 3'b001,
              CROSSING  = 3'b010,
              OUTGOING  = 3'b011;

    always @(posedge clk or posedge rst) begin
        if (rst)
            current <= IDLE;
        else
            current <= next;
    end

    always @(*) begin
        next = current;
        case (current)
            IDLE:     if (entry) next = INCOMING;
            INCOMING: next = CROSSING;
            CROSSING: if (exit) next = OUTGOING;
            OUTGOING: next = IDLE;
        endcase
    end

    always @(*) begin
        red   = 0;
        green = 0;
        motor = 12'd0;

        case (current)
            IDLE:     begin green = 1; motor = 12'd120; end
            INCOMING: begin red = 1;  motor = 12'd0; end
            CROSSING: begin red = 1;  motor = 12'd0; end
            OUTGOING: begin red = 1;  motor = 12'd120; end
        endcase
    end


    reg [15:0] refresh_cnt = 0;
    reg refresh_clk = 0;
    reg [1:0] digit = 0;
    reg [3:0] bcd;

    always @(posedge clk) begin
        refresh_cnt <= refresh_cnt + 1;
        refresh_clk <= refresh_cnt[15];
    end

    always @(posedge refresh_clk) begin
        digit <= digit + 1;
    end

    always @(*) begin
        case(digit)
            2'b00: begin an = 4'b1110; bcd = motor % 10; end
            2'b01: begin an = 4'b1101; bcd = (motor / 10) % 10; end
            2'b10: begin an = 4'b1011; bcd = (motor / 100) % 10; end
            2'b11: begin an = 4'b0111; bcd = (motor / 1000) % 10; end
        endcase
    end

    always @(*) begin
        case(bcd)
            4'd0: seg = 7'b0111111;
            4'd1: seg = 7'b1001111;
            4'd2: seg = 7'b1111001;
            default: seg = 7'b1111111;
        endcase
    end

endmodule
```
---

### 3.2 Testbench — `train_signal_tb.v`

```verilog

module train_signal_tb;
    reg clk, rst, entry, exit;
    wire red, green, an, seg;

    train_signal uut(clk, rst, entry, exit, red, green, an, seg);

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst = 1; entry = 0; exit = 0;
        #20 rst = 0;

        entry = 1;
        #10 entry = 0;

        #50 exit = 1;
        #10 exit = 0;

        #50 $finish;
    end
endmodule
```

---

### 3.3 Constraint File — `train_signal.xdc`

```
set_property -dict {PACKAGE_PIN J2  IOSTANDARD LVCMOS33} [get_ports {rst}]
set_property -dict {PACKAGE_PIN V2  IOSTANDARD LVCMOS33} [get_ports {entry}]
set_property -dict {PACKAGE_PIN U2  IOSTANDARD LVCMOS33} [get_ports {exit}]
set_property -dict {PACKAGE_PIN V6  IOSTANDARD LVCMOS33} [get_ports {red}]
set_property -dict {PACKAGE_PIN V3  IOSTANDARD LVCMOS33} [get_ports {green}]
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports {clk}]
set_property -dict {PACKAGE_PIN D5  IOSTANDARD LVCMOS33} [get_ports {an[0]}]
set_property -dict {PACKAGE_PIN C4  IOSTANDARD LVCMOS33} [get_ports {an[1]}]
set_property -dict {PACKAGE_PIN C7  IOSTANDARD LVCMOS33} [get_ports {an[2]}]
set_property -dict {PACKAGE_PIN A8  IOSTANDARD LVCMOS33} [get_ports {an[3]}]
set_property -dict {PACKAGE_PIN D7  IOSTANDARD LVCMOS33} [get_ports {seg[0]}]
set_property -dict {PACKAGE_PIN C5  IOSTANDARD LVCMOS33} [get_ports {seg[1]}]
set_property -dict {PACKAGE_PIN A5  IOSTANDARD LVCMOS33} [get_ports {seg[2]}]
set_property -dict {PACKAGE_PIN B7  IOSTANDARD LVCMOS33} [get_ports {seg[3]}]
set_property -dict {PACKAGE_PIN A7  IOSTANDARD LVCMOS33} [get_ports {seg[4]}]
set_property -dict {PACKAGE_PIN D6  IOSTANDARD LVCMOS33} [get_ports {seg[5]}]
set_property -dict {PACKAGE_PIN B5  IOSTANDARD LVCMOS33} [get_ports {seg[6]}]
```

---

## 4. Output and Result

### 4.1 Testbench Output

![WhatsApp Image 2025-11-18 at 19 51 52_087118c9](https://github.com/user-attachments/assets/edcb1bcd-8439-428b-b8db-591d8d16910b)


### 4.2 Spartan-7 FPGA Implementation Output

![WhatsApp Image 2025-11-18 at 19 45 47_0a846562](https://github.com/user-attachments/assets/d4972b08-6af0-4da1-a0a3-c7167f096229)
* **When Train Approaches:** Entry sensor asserted → red LED ON → motor (simulated) changes value → 7-seg shows activity.

---

![WhatsApp Image 2025-11-18 at 19 45 50_a5762b31](https://github.com/user-attachments/assets/5937af92-3016-4cbc-87ee-21380eae455b)
* **When Train Passes:** Crossing state maintains red LED until exit sensor asserted.

### 4.3 Result

The Verilog design for automatic railway gate control was implemented and verified using Vivado simulations. FSM-based control ensures predictable operation. The 7-segment display provides a simple visual indication of motor activity or status for demonstration.

---

## Appendix — Files Provided

* `train_signal.v` — RTL implementation
* `train_signal_tb.v` — Testbench
* `train_signal.xdc` — Constraint file

