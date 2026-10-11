`default_nettype none
`timescale 1ns / 1ps

// =============================================================================
// File        : spiral_maze_env.v
// Description : Simulation model emulating physical environment (7x7 inward spiral),
//               boundary walls, and agent kinematics (position/orientation tracking).
// =============================================================================

module spiral_maze_env #(
    parameter GRID_BITS = 6
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   i_step,
    input  wire                   i_turn_right,
    output wire                   o_can_move,
    output reg  [GRID_BITS-1:0]   o_agent_x,
    output reg  [GRID_BITS-1:0]   o_agent_y,
    output reg  [1:0]             o_agent_dir
);

    localparam DIR_N = 2'b00;
    localparam DIR_E = 2'b01;
    localparam DIR_S = 2'b10;
    localparam DIR_W = 2'b11;

    // 7x7 maze grid boolean map (1 = clear / corridor, 0 = wall / boundary)
    reg maze_map [0:6][0:6];

    initial begin
        // Row 0
        maze_map[0][0] = 0; maze_map[0][1] = 0; maze_map[0][2] = 0; maze_map[0][3] = 0; maze_map[0][4] = 0; maze_map[0][5] = 0; maze_map[0][6] = 0;
        // Row 1
        maze_map[1][0] = 0; maze_map[1][1] = 1; maze_map[1][2] = 1; maze_map[1][3] = 1; maze_map[1][4] = 1; maze_map[1][5] = 1; maze_map[1][6] = 0;
        // Row 2
        maze_map[2][0] = 0; maze_map[2][1] = 1; maze_map[2][2] = 0; maze_map[2][3] = 0; maze_map[2][4] = 0; maze_map[2][5] = 1; maze_map[2][6] = 0;
        // Row 3 (Center destination at [3][3])
        maze_map[3][0] = 0; maze_map[3][1] = 1; maze_map[3][2] = 0; maze_map[3][3] = 1; maze_map[3][4] = 0; maze_map[3][5] = 1; maze_map[3][6] = 0;
        // Row 4
        maze_map[4][0] = 0; maze_map[4][1] = 1; maze_map[4][2] = 0; maze_map[4][3] = 1; maze_map[4][4] = 1; maze_map[4][5] = 1; maze_map[4][6] = 0;
        // Row 5
        maze_map[5][0] = 0; maze_map[5][1] = 1; maze_map[5][2] = 0; maze_map[5][3] = 0; maze_map[5][4] = 0; maze_map[5][5] = 0; maze_map[5][6] = 0;
        // Row 6 (South entrance at [6][3])
        maze_map[6][0] = 0; maze_map[6][1] = 1; maze_map[6][2] = 1; maze_map[6][3] = 1; maze_map[6][4] = 0; maze_map[6][5] = 0; maze_map[6][6] = 0;
    end

    // Combinational evaluation of candidate forward cell
    reg [GRID_BITS-1:0] front_x, front_y;

    always @(*) begin
        front_x = o_agent_x;
        front_y = o_agent_y;
        case (o_agent_dir)
            DIR_N: front_y = o_agent_y - 1'b1;
            DIR_E: front_x = o_agent_x + 1'b1;
            DIR_S: front_y = o_agent_y + 1'b1;
            DIR_W: front_x = o_agent_x - 1'b1;
            default: begin
                front_x = o_agent_x;
                front_y = o_agent_y;
            end
        endcase
    end

    // Front sensor reading: 1 if within 7x7 boundaries and clear, 0 if wall or out of range
    assign o_can_move = (front_x <= 6 && front_y <= 6) ? maze_map[front_y[2:0]][front_x[2:0]] : 1'b0;

    // Kinematics and orientation tracking in physical environment
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_agent_x   <= 6'd3;     // South entrance x
            o_agent_y   <= 6'd6;     // South entrance y
            o_agent_dir <= DIR_N;    // Facing North initially
        end else begin
            if (i_turn_right) begin
                o_agent_dir <= o_agent_dir + 2'b01;
            end else if (i_step && o_can_move) begin
                o_agent_x <= front_x;
                o_agent_y <= front_y;
            end
        end
    end

endmodule
