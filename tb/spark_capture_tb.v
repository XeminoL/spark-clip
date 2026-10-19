`timescale 1ns / 1ps

module spark_capture_tb;

    reg         clk = 1'b0;
    reg         reset = 1'b1;
    reg         sample_valid = 1'b0;
    reg  [11:0] sample = 12'd0;
    integer     stimulus_file;
    integer     output_file;
    integer     scanned;
    integer     value;
    integer     byte_count = 0;
    integer     idle_cycles;
    reg [1023:0] stimulus_path;
    reg [1023:0] output_path;

    wire        record_valid;
    wire [31:0] start_time;
    wire [11:0] peak_height;
    wire [13:0] peak_offset;
    wire [17:0] burn_length;
    wire        sparked;
    wire [6:0]  level_count;
    wire [6:0]  ladder_address;
    wire [13:0] ladder_offset;
    wire        packer_busy;
    wire        fifo_write;
    wire [7:0]  fifo_data;

    always #5 clk = ~clk;

    spark_capture capture (
        .clk            (clk),
        .reset          (reset),
        .sample_valid   (sample_valid),
        .sample         (sample),
        .reader_busy    (packer_busy),
        .ladder_address (ladder_address),
        .ladder_offset  (ladder_offset),
        .record_valid   (record_valid),
        .start_time     (start_time),
        .peak_height    (peak_height),
        .peak_offset    (peak_offset),
        .burn_length    (burn_length),
        .sparked        (sparked),
        .level_count    (level_count)
    );

    record_packer packer (
        .clk            (clk),
        .reset          (reset),
        .record_valid   (record_valid),
        .start_time     (start_time),
        .peak_height    (peak_height),
        .peak_offset    (peak_offset),
        .burn_length    (burn_length),
        .sparked        (sparked),
        .level_count    (level_count),
        .ladder_address (ladder_address),
        .ladder_offset  (ladder_offset),
        .fifo_full      (1'b0),
        .fifo_write     (fifo_write),
        .fifo_data      (fifo_data),
        .busy           (packer_busy)
    );

    always @(posedge clk) begin
        if (fifo_write) begin
            $fwrite(output_file, "%02x\n", fifo_data);
            byte_count = byte_count + 1;
        end
    end

    initial begin
        if (!$value$plusargs("stimulus=%s", stimulus_path)) stimulus_path = "stimulus.hex";
        if (!$value$plusargs("output=%s", output_path)) output_path = "records.hex";
        stimulus_file = $fopen(stimulus_path, "r");
        output_file = $fopen(output_path, "w");
        if (stimulus_file == 0) begin
            $display("cannot open stimulus");
            $finish;
        end
        repeat (4) @(posedge clk);
        reset <= 1'b0;
        scanned = $fscanf(stimulus_file, "%h\n", value);
        while (scanned == 1) begin
            @(posedge clk);
            sample       <= value[11:0];
            sample_valid <= 1'b1;
            scanned = $fscanf(stimulus_file, "%h\n", value);
        end
        @(posedge clk);
        sample_valid <= 1'b0;
        idle_cycles = 0;
        while (idle_cycles < 1000) begin
            @(posedge clk);
            idle_cycles = packer_busy ? 0 : idle_cycles + 1;
        end
        $fclose(output_file);
        $display("bytes %0d", byte_count);
        $finish;
    end

endmodule