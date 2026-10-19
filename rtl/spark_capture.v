module spark_capture #(
    parameter SAMPLE_WIDTH      = 12,
    parameter CALIBRATION_SHIFT = 12,
    parameter TRIGGER_LEVEL     = 24,
    parameter LADDER_SHIFT      = 4,
    parameter LADDER_LEVELS     = 127,
    parameter COLLAPSE_LIMIT    = 64,
    parameter RISE_LIMIT        = 16383,
    parameter BURN_LEVEL        = 16,
    parameter BURN_GAP          = 64,
    parameter BURN_LIMIT        = 262143,
    parameter HOLDOFF           = 65535
) (
    input  wire                           clk,
    input  wire                           reset,
    input  wire                           sample_valid,
    input  wire signed [SAMPLE_WIDTH-1:0] sample,
    input  wire                           reader_busy,
    input  wire [6:0]                     ladder_address,
    output wire [13:0]                    ladder_offset,
    output reg                            record_valid,
    output reg  [31:0]                    start_time,
    output wire [SAMPLE_WIDTH-1:0]        peak_height,
    output reg  [13:0]                    peak_offset,
    output reg  [17:0]                    burn_length,
    output reg                            sparked,
    output reg  [6:0]                     level_count
);

    localparam LEVEL_WIDTH = SAMPLE_WIDTH + 1;
    localparam SUM_WIDTH   = SAMPLE_WIDTH + CALIBRATION_SHIFT;

    localparam CALIBRATE = 3'd0;
    localparam IDLE      = 3'd1;
    localparam RISE      = 3'd2;
    localparam BURN      = 3'd3;
    localparam HOLD      = 3'd4;

    reg [2:0]                    state;
    reg [31:0]                   now;
    reg signed [SUM_WIDTH-1:0]   calibration_sum;
    reg [CALIBRATION_SHIFT-1:0]  calibration_count;
    reg signed [SAMPLE_WIDTH-1:0] offset;
    reg signed [LEVEL_WIDTH-1:0] peak;
    reg [13:0]                   elapsed;
    reg [17:0]                   burn_elapsed;
    reg [6:0]                    quiet;
    reg [15:0]                   hold_count;
    reg [13:0]                   ladder [0:LADDER_LEVELS-1];

    wire signed [LEVEL_WIDTH-1:0] level = sample - offset;
    wire [LEVEL_WIDTH:0]          rung_index = level_count + 7'd1;
    wire signed [LEVEL_WIDTH:0]   next_rung = $signed(rung_index << LADDER_SHIFT);
    wire                          rung_hit = (level_count < LADDER_LEVELS) && ($signed({level[LEVEL_WIDTH-1], level}) >= next_rung);
    wire signed [LEVEL_WIDTH-1:0] half_peak = peak >>> 1;
    wire [13:0]                   elapsed_next = elapsed + 14'd1;
    wire [17:0]                   burn_next = burn_elapsed + 18'd1;
    wire signed [SUM_WIDTH-1:0]   calibration_total = calibration_sum + sample;

    assign peak_height   = peak[SAMPLE_WIDTH-1:0];
    assign ladder_offset = ladder[ladder_address];

    always @(posedge clk) begin
        if (sample_valid && state == IDLE && level >= TRIGGER_LEVEL && level >= (1 << LADDER_SHIFT))
            ladder[0] <= 14'd0;
        else if (sample_valid && state == RISE && rung_hit)
            ladder[level_count] <= elapsed_next;
    end

    always @(posedge clk) begin
        if (reset) begin
            state             <= CALIBRATE;
            now               <= 32'd0;
            calibration_sum   <= 0;
            calibration_count <= 0;
            offset            <= 0;
            peak              <= 0;
            elapsed           <= 14'd0;
            burn_elapsed      <= 18'd0;
            quiet             <= 7'd0;
            hold_count        <= 16'd0;
            record_valid      <= 1'b0;
            start_time        <= 32'd0;
            peak_offset       <= 14'd0;
            burn_length       <= 18'd0;
            sparked           <= 1'b0;
            level_count       <= 7'd0;
        end else begin
            record_valid <= 1'b0;
            if (sample_valid) begin
                now <= now + 32'd1;
                case (state)
                    CALIBRATE: begin
                        calibration_sum   <= calibration_total;
                        calibration_count <= calibration_count + 1'b1;
                        if (&calibration_count) begin
                            offset <= calibration_total >>> CALIBRATION_SHIFT;
                            state  <= IDLE;
                        end
                    end
                    IDLE: begin
                        if (level >= TRIGGER_LEVEL) begin
                            start_time  <= now;
                            elapsed     <= 14'd0;
                            peak        <= level;
                            peak_offset <= 14'd0;
                            sparked     <= 1'b0;
                            burn_length <= 18'd0;
                            level_count <= (level >= (1 << LADDER_SHIFT)) ? 7'd1 : 7'd0;
                            state       <= RISE;
                        end
                    end
                    RISE: begin
                        elapsed <= elapsed_next;
                        if (level > peak) begin
                            peak        <= level;
                            peak_offset <= elapsed_next;
                        end
                        if (rung_hit)
                            level_count <= level_count + 7'd1;
                        if (level < half_peak) begin
                            if (elapsed_next - peak_offset <= COLLAPSE_LIMIT) begin
                                sparked      <= 1'b1;
                                burn_elapsed <= 18'd0;
                                quiet        <= 7'd0;
                                state        <= BURN;
                            end else begin
                                record_valid <= 1'b1;
                                hold_count   <= 16'd0;
                                state        <= HOLD;
                            end
                        end else if (elapsed_next == RISE_LIMIT) begin
                            record_valid <= 1'b1;
                            hold_count   <= 16'd0;
                            state        <= HOLD;
                        end
                    end
                    BURN: begin
                        burn_elapsed <= burn_next;
                        if (level >= BURN_LEVEL) begin
                            burn_length <= burn_next;
                            quiet       <= 7'd0;
                        end else begin
                            quiet <= quiet + 7'd1;
                        end
                        if ((level < BURN_LEVEL && quiet + 7'd1 == BURN_GAP) || burn_next == BURN_LIMIT) begin
                            record_valid <= 1'b1;
                            hold_count   <= 16'd0;
                            state        <= HOLD;
                        end
                    end
                    HOLD: begin
                        if (hold_count >= HOLDOFF - 1) begin
                            if (!reader_busy)
                                state <= IDLE;
                        end else begin
                            hold_count <= hold_count + 16'd1;
                        end
                    end
                    default: state <= CALIBRATE;
                endcase
            end
        end
    end

endmodule