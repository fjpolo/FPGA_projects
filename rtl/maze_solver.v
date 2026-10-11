`default_nettype none
`timescale 1ns / 1ps

// =============================================================================
// File        : maze_solver.v
// Description : Synthesizable FPGA Maze Solver Core using Trémaux's algorithm / DFS.
// =============================================================================

module maze_solver #(
    parameter GRID_BITS   = 6,                         // Up to 64x64 grid
    parameter STACK_DEPTH = 1 << (2 * GRID_BITS)       // Max exploration depth
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   i_start,
    input  wire [GRID_BITS-1:0]   i_start_x,
    input  wire [GRID_BITS-1:0]   i_start_y,
    input  wire [GRID_BITS-1:0]   i_target_x,
    input  wire [GRID_BITS-1:0]   i_target_y,

    // Maze local-sensing interface
    input  wire                   i_can_move,          // 1: go, 0: wall
    output reg                    o_step,              // Single-cycle pulse: advance 1 cell
    output reg                    o_turn_right,        // Single-cycle pulse: rotate +90 deg
    output reg                    o_done,              // Asserted high on reaching target
    output reg                    o_failed             // Asserted high if maze has no solution
);

    // Orientation encoding: 00: North (-y), 01: East (+x), 10: South (+y), 11: West (-x)
    localparam DIR_N = 2'b00;
    localparam DIR_E = 2'b01;
    localparam DIR_S = 2'b10;
    localparam DIR_W = 2'b11;

    // Cell memory states
    localparam CELL_UNVISITED = 2'b00;
    localparam CELL_VISITED   = 2'b01;
    localparam CELL_DEAD_END  = 2'b10;

    // FSM States
    localparam S_IDLE         = 4'd0;
    localparam S_CHECK_GOAL   = 4'd1;
    localparam S_EVAL_DIR     = 4'd2;
    localparam S_READ_MEM     = 4'd3;
    localparam S_DECIDE_DIR   = 4'd4;
    localparam S_ROTATE       = 4'd5;
    localparam S_MOVE_FWD     = 4'd6;
    localparam S_START_BACK   = 4'd7;
    localparam S_ROT_BACK     = 4'd8;
    localparam S_STEP_BACK    = 4'd9;
    localparam S_DONE         = 4'd10;
    localparam S_FAIL         = 4'd11;

    reg [3:0] state;

    // Navigation registers
    reg [GRID_BITS-1:0] curr_x, curr_y;
    reg [1:0]           curr_dir;
    reg [1:0]           turns_tested; // 0 to 3 directions tested for current cell
    reg [1:0]           target_rot;

    // Next cell coordinates based on current orientation
    reg [GRID_BITS-1:0] next_x, next_y;

    always @(*) begin
        next_x = curr_x;
        next_y = curr_y;
        case (curr_dir)
            DIR_N: next_y = curr_y - 1'b1;
            DIR_E: next_x = curr_x + 1'b1;
            DIR_S: next_y = curr_y + 1'b1;
            DIR_W: next_x = curr_x - 1'b1;
            default: begin
                next_x = curr_x;
                next_y = curr_y;
            end
        endcase
    end

    // Grid Memory (BRAM / LUTRAM: 2 bits per coordinate)
    localparam ADDR_BITS = 2 * GRID_BITS;
    localparam MEM_SIZE  = 1 << ADDR_BITS;

    reg [1:0] grid_mem [MEM_SIZE-1:0];

    wire [ADDR_BITS-1:0] curr_addr = {curr_y, curr_x};
    wire [ADDR_BITS-1:0] next_addr = {next_y, next_x};

    reg [1:0] next_cell_val;

    // Initialize grid memory to CELL_UNVISITED for simulation and FPGA BRAM initialization
    integer init_i;
    initial begin
        for (init_i = 0; init_i < MEM_SIZE; init_i = init_i + 1) begin
            grid_mem[init_i] = CELL_UNVISITED;
        end
    end

    // Backtracking stack (stores the 2-bit absolute direction moved)
    localparam PTR_BITS = $clog2(STACK_DEPTH);
    reg [1:0] stack_dir [STACK_DEPTH-1:0];
    reg [PTR_BITS:0] stack_ptr;
    wire [PTR_BITS-1:0] stack_idx      = stack_ptr[PTR_BITS-1:0];
    wire [PTR_BITS-1:0] stack_prev_idx = stack_ptr[PTR_BITS-1:0] - 1'b1;

    // Invert direction vector (dir ^ 2'b10): N <-> S, E <-> W
    function [1:0] invert_dir;
        input [1:0] d;
        begin
            invert_dir = d ^ 2'b10;
        end
    endfunction

    // Synchronous Navigation FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            curr_x       <= {GRID_BITS{1'b0}};
            curr_y       <= {GRID_BITS{1'b0}};
            curr_dir     <= DIR_N;
            turns_tested <= 2'b00;
            target_rot   <= DIR_N;
            next_cell_val<= CELL_UNVISITED;
            stack_ptr    <= {($clog2(STACK_DEPTH)+1){1'b0}};
            o_step       <= 1'b0;
            o_turn_right <= 1'b0;
            o_done       <= 1'b0;
            o_failed     <= 1'b0;
        end else begin
            // Default single-cycle strobe values
            o_step       <= 1'b0;
            o_turn_right <= 1'b0;

            case (state)
                S_IDLE: begin
                    o_done   <= 1'b0;
                    o_failed <= 1'b0;
                    if (i_start) begin
                        curr_x                           <= i_start_x;
                        curr_y                           <= i_start_y;
                        curr_dir                         <= DIR_N;
                        stack_ptr                        <= {($clog2(STACK_DEPTH)+1){1'b0}};
                        grid_mem[{i_start_y, i_start_x}] <= CELL_VISITED;
                        state                            <= S_CHECK_GOAL;
                    end
                end

                S_CHECK_GOAL: begin
                    if (curr_x == i_target_x && curr_y == i_target_y) begin
                        o_done <= 1'b1;
                        state  <= S_DONE;
                    end else begin
                        turns_tested <= 2'b00;
                        state        <= S_EVAL_DIR;
                    end
                end

                S_EVAL_DIR: begin
                    // If wall ahead, discard this direction
                    if (!i_can_move) begin
                        if (turns_tested == 2'd3) begin
                            state <= S_START_BACK; // All 4 directions exhausted
                        end else begin
                            o_turn_right <= 1'b1;
                            curr_dir     <= curr_dir + 2'b01;
                            turns_tested <= turns_tested + 2'b01;
                            state        <= S_ROTATE;
                        end
                    end else begin
                        // Forward is clear: read neighbor cell value from memory
                        state <= S_READ_MEM;
                    end
                end

                S_READ_MEM: begin
                    // Isolate memory read latency to prevent combinational race conditions
                    next_cell_val <= grid_mem[next_addr];
                    state         <= S_DECIDE_DIR;
                end

                S_DECIDE_DIR: begin
                    if (next_cell_val == CELL_UNVISITED) begin
                        // Valid unvisited cell: advance
                        state <= S_MOVE_FWD;
                    end else begin
                        // Cell already visited or dead end
                        if (turns_tested == 2'd3) begin
                            state <= S_START_BACK;
                        end else begin
                            o_turn_right <= 1'b1;
                            curr_dir     <= curr_dir + 2'b01;
                            turns_tested <= turns_tested + 2'b01;
                            state        <= S_ROTATE;
                        end
                    end
                end

                S_ROTATE: begin
                    // 1-cycle settling latency for physical kinematics rotation
                    state <= S_EVAL_DIR;
                end

                S_MOVE_FWD: begin
                    // Execute forward step
                    o_step                 <= 1'b1;
                    curr_x                 <= next_x;
                    curr_y                 <= next_y;
                    grid_mem[next_addr]    <= CELL_VISITED;
                    // Push direction moved onto stack
                    stack_dir[stack_idx]   <= curr_dir;
                    stack_ptr              <= stack_ptr + 1'b1;
                    state                  <= S_CHECK_GOAL;
                end

                S_START_BACK: begin
                    // Exhausted branch: mark current cell as dead end
                    grid_mem[curr_addr] <= CELL_DEAD_END;
                    if (stack_ptr == {(PTR_BITS+1){1'b0}}) begin
                        // Stack empty: no solution exists
                        o_failed <= 1'b1;
                        state    <= S_FAIL;
                    end else begin
                        // Pop previous move and compute reverse direction
                        stack_ptr  <= stack_ptr - 1'b1;
                        target_rot <= invert_dir(stack_dir[stack_prev_idx]);
                        state      <= S_ROT_BACK;
                    end
                end

                S_ROT_BACK: begin
                    if (curr_dir == target_rot) begin
                        state <= S_STEP_BACK;
                    end else begin
                        o_turn_right <= 1'b1;
                        curr_dir     <= curr_dir + 2'b01;
                    end
                end

                S_STEP_BACK: begin
                    o_step <= 1'b1;
                    curr_x <= next_x;
                    curr_y <= next_y;
                    state  <= S_CHECK_GOAL;
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    if (i_start) begin
                        o_done                           <= 1'b0;
                        curr_x                           <= i_start_x;
                        curr_y                           <= i_start_y;
                        curr_dir                         <= DIR_N;
                        stack_ptr                        <= {(PTR_BITS+1){1'b0}};
                        grid_mem[{i_start_y, i_start_x}] <= CELL_VISITED;
                        state                            <= S_CHECK_GOAL;
                    end
                end

                S_FAIL: begin
                    o_failed <= 1'b1;
                    if (i_start) begin
                        o_failed                         <= 1'b0;
                        curr_x                           <= i_start_x;
                        curr_y                           <= i_start_y;
                        curr_dir                         <= DIR_N;
                        stack_ptr                        <= {(PTR_BITS+1){1'b0}};
                        grid_mem[{i_start_y, i_start_x}] <= CELL_VISITED;
                        state                            <= S_CHECK_GOAL;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
