module led (
    input   wire            i_sys_clk,          // clk input
    input   wire            i_sys_rst_n,        // reset input
    output  wire     [5:0]   o_led    // 6 LEDS pin
);

// module maze(
//     input   wire    [0:0]   i_clk,
//     input   wire    [0:0]   i_reset_n,
//     input   wire    [7:0]   i_data,
//     output  reg     [7:0]   o_data
// );
wire [7:0] o_output;
maze DUT(
    .i_clk(i_sys_clk),
    .i_reset_n(i_sys_rst_n),
    .i_data('h00),  // Example input data, modify as needed
    .o_data(o_output)   // Connect output to LED pins
);
assign o_led = o_output[5:0]; // Assign the first 6 bits to the LED output    

endmodule
