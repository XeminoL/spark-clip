module record_packer (
    input  wire                    clk,
    input  wire                    reset,
    input  wire                    record_valid,
    input  wire [31:0]             start_time,
    input  wire [11:0]             peak_height,
    input  wire [13:0]             peak_offset,
    input  wire [17:0]             burn_length,
    input  wire                    sparked,
    input  wire [6:0]              level_count,
    output wire [6:0]              ladder_address,
    input  wire [13:0]             ladder_offset,
    input  wire                    fifo_full,
    output wire                    fifo_write,
    output reg  [7:0]              fifo_data,
    output wire                    busy
);

    localparam MARKER = 8'hA5;

    reg        sending;
    reg [6:0]  word_index;
    reg [1:0]  byte_index;
    wire [7:0] rung_pairs = ({1'b0, level_count} + 8'd1) >> 1;
    wire [6:0] word_total = 7'd3 + rung_pairs[6:0];
    wire [31:0] header = {MARKER, sparked, level_count, 4'd0, peak_height};
    wire [31:0] timing = {peak_offset, burn_length};
    wire [6:0]  rung = {word_index - 7'd3, byte_index[1]};
    wire [15:0] rung_value = (rung < level_count) ? {2'b00, ladder_offset} : 16'd0;
    wire        last_byte = (byte_index == 2'd3) && (word_index == word_total - 7'd1);

    assign ladder_address = rung;
    assign fifo_write     = sending && !fifo_full;
    assign busy           = sending || record_valid;

    always @(*) begin
        case (word_index)
            7'd0:    fifo_data = header[8*byte_index +: 8];
            7'd1:    fifo_data = start_time[8*byte_index +: 8];
            7'd2:    fifo_data = timing[8*byte_index +: 8];
            default: fifo_data = rung_value[8*byte_index[0] +: 8];
        endcase
    end

    always @(posedge clk) begin
        if (reset) begin
            sending    <= 1'b0;
            word_index <= 7'd0;
            byte_index <= 2'd0;
        end else if (!sending) begin
            if (record_valid) begin
                sending    <= 1'b1;
                word_index <= 7'd0;
                byte_index <= 2'd0;
            end
        end else if (!fifo_full) begin
            byte_index <= byte_index + 2'd1;
            if (byte_index == 2'd3)
                word_index <= word_index + 7'd1;
            if (last_byte)
                sending <= 1'b0;
        end
    end

endmodule