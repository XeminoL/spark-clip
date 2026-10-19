module spark_clip_top #(
    parameter CLOCKS_PER_BIT = 108,
    parameter FIFO_ADDR_WIDTH = 10
) (
    input  wire        adc_clk,
    input  wire [11:0] adc_data,
    input  wire        sys_clk,
    input  wire        reset_button,
    output wire        uart_line,
    output wire        ready_led
);

    (* ASYNC_REG = "TRUE" *) reg [1:0] adc_reset_sync = 2'b11;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sys_reset_sync = 2'b11;
    reg [11:0] adc_sample;
    reg        adc_valid;

    wire adc_reset = adc_reset_sync[1];
    wire sys_reset = sys_reset_sync[1];

    always @(posedge adc_clk or posedge reset_button)
        if (reset_button) adc_reset_sync <= 2'b11;
        else              adc_reset_sync <= {adc_reset_sync[0], 1'b0};

    always @(posedge sys_clk or posedge reset_button)
        if (reset_button) sys_reset_sync <= 2'b11;
        else              sys_reset_sync <= {sys_reset_sync[0], 1'b0};

    always @(posedge adc_clk) begin
        adc_sample <= adc_data;
        adc_valid  <= !adc_reset;
    end

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
    wire        fifo_full;
    wire        fifo_write;
    wire [7:0]  fifo_data;

    spark_capture capture (
        .clk            (adc_clk),
        .reset          (adc_reset),
        .sample_valid   (adc_valid),
        .sample         (adc_sample),
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
        .clk            (adc_clk),
        .reset          (adc_reset),
        .record_valid   (record_valid),
        .start_time     (start_time),
        .peak_height    (peak_height),
        .peak_offset    (peak_offset),
        .burn_length    (burn_length),
        .sparked        (sparked),
        .level_count    (level_count),
        .ladder_address (ladder_address),
        .ladder_offset  (ladder_offset),
        .fifo_full      (fifo_full),
        .fifo_write     (fifo_write),
        .fifo_data      (fifo_data),
        .busy           (packer_busy)
    );

    wire       fifo_empty;
    wire [7:0] uart_data;
    wire       uart_busy;
    reg        byte_pending;
    reg        read_issued;

    async_fifo #(.WIDTH(8), .ADDR_WIDTH(FIFO_ADDR_WIDTH)) bytes (
        .write_clk    (adc_clk),
        .write_reset  (adc_reset),
        .write_enable (fifo_write),
        .write_data   (fifo_data),
        .full         (fifo_full),
        .read_clk     (sys_clk),
        .read_reset   (sys_reset),
        .read_enable  (read_issued),
        .read_data    (uart_data),
        .empty        (fifo_empty)
    );

    always @(posedge sys_clk) begin
        if (sys_reset) begin
            read_issued  <= 1'b0;
            byte_pending <= 1'b0;
        end else begin
            read_issued  <= !fifo_empty && !uart_busy && !byte_pending && !read_issued;
            byte_pending <= read_issued;
        end
    end

    uart_tx #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) serial (
        .clk   (sys_clk),
        .reset (sys_reset),
        .start (byte_pending),
        .data  (uart_data),
        .line  (uart_line),
        .busy  (uart_busy)
    );

    assign ready_led = !adc_reset;

endmodule