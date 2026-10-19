module async_fifo #(
    parameter WIDTH      = 8,
    parameter ADDR_WIDTH = 10
) (
    input  wire             write_clk,
    input  wire             write_reset,
    input  wire             write_enable,
    input  wire [WIDTH-1:0] write_data,
    output wire             full,
    input  wire             read_clk,
    input  wire             read_reset,
    input  wire             read_enable,
    output reg  [WIDTH-1:0] read_data,
    output wire             empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [WIDTH-1:0]    memory [0:DEPTH-1];
    reg [ADDR_WIDTH:0] write_binary, write_gray;
    reg [ADDR_WIDTH:0] read_binary, read_gray;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] read_gray_meta, read_gray_seen;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] write_gray_meta, write_gray_seen;

    wire [ADDR_WIDTH:0] write_binary_next = write_binary + (write_enable && !full);
    wire [ADDR_WIDTH:0] read_binary_next  = read_binary + (read_enable && !empty);
    wire [ADDR_WIDTH:0] write_gray_next   = (write_binary_next >> 1) ^ write_binary_next;
    wire [ADDR_WIDTH:0] read_gray_next    = (read_binary_next >> 1) ^ read_binary_next;

    assign full  = write_gray == {~read_gray_seen[ADDR_WIDTH:ADDR_WIDTH-1], read_gray_seen[ADDR_WIDTH-2:0]};
    assign empty = read_gray == write_gray_seen;

    always @(posedge write_clk) begin
        if (write_enable && !full)
            memory[write_binary[ADDR_WIDTH-1:0]] <= write_data;
    end

    always @(posedge write_clk) begin
        if (write_reset) begin
            write_binary    <= 0;
            write_gray      <= 0;
            read_gray_meta  <= 0;
            read_gray_seen  <= 0;
        end else begin
            write_binary    <= write_binary_next;
            write_gray      <= write_gray_next;
            read_gray_meta  <= read_gray;
            read_gray_seen  <= read_gray_meta;
        end
    end

    always @(posedge read_clk) begin
        if (read_enable && !empty)
            read_data <= memory[read_binary[ADDR_WIDTH-1:0]];
    end

    always @(posedge read_clk) begin
        if (read_reset) begin
            read_binary     <= 0;
            read_gray       <= 0;
            write_gray_meta <= 0;
            write_gray_seen <= 0;
        end else begin
            read_binary     <= read_binary_next;
            read_gray       <= read_gray_next;
            write_gray_meta <= write_gray;
            write_gray_seen <= write_gray_meta;
        end
    end

endmodule