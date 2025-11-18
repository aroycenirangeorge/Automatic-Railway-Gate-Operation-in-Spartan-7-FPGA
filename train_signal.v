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
