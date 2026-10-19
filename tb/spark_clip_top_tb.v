`timescale 1ns / 1ps

module spark_clip_top_tb;

    localparam CLOCKS_PER_BIT = 8;

    reg         adc_clk = 1'b0;
    reg         sys_clk = 1'b0;
    reg         reset_button = 1'b1;
    reg  [11:0] adc_data = 12'd0;
    wire        uart_line;
    wire        ready_led;
    integer     stimulus_file;
    integer     output_file;
    integer     scanned;
    integer     value;
    integer     byte_count = 0;
    integer     bit_index;
    reg  [7:0]  received;
    reg [1023:0] stimulus_path;
    reg [1023:0] output_path;

    always #5 adc_clk = ~adc_clk;
    always #3.6 sys_clk = ~sys_clk;

    spark_clip_top #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT), .FIFO_ADDR_WIDTH(6)) dut (
        .adc_clk      (adc_clk),
        .adc_data     (adc_data),
        .sys_clk      (sys_clk),
        .reset_button (reset_button),
        .uart_line    (uart_line),
        .ready_led    (ready_led)
    );

    always begin
        @(negedge uart_line);
        repeat (CLOCKS_PER_BIT / 2) @(posedge sys_clk);
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
            repeat (CLOCKS_PER_BIT) @(posedge sys_clk);
            received[bit_index] = uart_line;
        end
        repeat (CLOCKS_PER_BIT) @(posedge sys_clk);
        if (uart_line !== 1'b1) $display("framing error at byte %0d", byte_count);
        $fwrite(output_file, "%02x\n", received);
        byte_count = byte_count + 1;
    end

    initial begin
        if (!$value$plusargs("stimulus=%s", stimulus_path)) stimulus_path = "stimulus.hex";
        if (!$value$plusargs("output=%s", output_path)) output_path = "uart.hex";
        stimulus_file = $fopen(stimulus_path, "r");
        output_file = $fopen(output_path, "w");
        repeat (3) @(posedge adc_clk);
        reset_button = 1'b0;
        wait (ready_led);
        scanned = $fscanf(stimulus_file, "%h\n", value);
        while (scanned == 1) begin
            @(posedge adc_clk);
            adc_data <= value[11:0];
            scanned = $fscanf(stimulus_file, "%h\n", value);
        end
        repeat (200000) @(posedge sys_clk);
        $fclose(output_file);
        $display("bytes %0d", byte_count);
        $finish;
    end

endmodule