module uart_tx #(
    parameter CLOCKS_PER_BIT = 108
) (
    input  wire       clk,
    input  wire       reset,
    input  wire       start,
    input  wire [7:0] data,
    output reg        line,
    output wire       busy
);

    localparam COUNT_WIDTH = $clog2(CLOCKS_PER_BIT);

    reg [9:0]             frame;
    reg [3:0]             bits_left;
    reg [COUNT_WIDTH-1:0] ticks;

    assign busy = bits_left != 4'd0;

    always @(posedge clk) begin
        if (reset) begin
            line      <= 1'b1;
            frame     <= 10'h3FF;
            bits_left <= 4'd0;
            ticks     <= 0;
        end else if (!busy) begin
            line <= 1'b1;
            if (start) begin
                frame     <= {1'b1, data, 1'b0};
                bits_left <= 4'd10;
                ticks     <= 0;
                line      <= 1'b0;
            end
        end else if (ticks == CLOCKS_PER_BIT - 1) begin
            ticks     <= 0;
            bits_left <= bits_left - 4'd1;
            frame     <= {1'b1, frame[9:1]};
            line      <= (bits_left == 4'd1) ? 1'b1 : frame[1];
        end else begin
            ticks <= ticks + 1'b1;
        end
    end

endmodule