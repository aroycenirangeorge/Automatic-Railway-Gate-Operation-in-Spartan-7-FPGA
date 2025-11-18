
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
