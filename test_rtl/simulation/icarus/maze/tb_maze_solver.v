`timescale 1ns / 1ps

// =============================================================================
// File        : tb_maze_solver.v
// Description : Self-checking testbench for Synthesizable FPGA Maze Solver.
//               Validates navigation through a 7x7 inward spiral maze from
//               South entrance (3, 6) to center target (3, 3) using Trémaux/DFS.
// =============================================================================

module tb_maze_solver;

    // Simulation Parameters
    localparam GRID_BITS   = 6;
    localparam STACK_DEPTH = 256;
    localparam CLK_PERIOD  = 10; // 10 ns = 100 MHz

    // Testbench Stimulus Signals
    reg                   clk;
    reg                   rst_n;
    reg                   i_start;

    wire [GRID_BITS-1:0]  start_x  = 6'd3;
    wire [GRID_BITS-1:0]  start_y  = 6'd6;
    wire [GRID_BITS-1:0]  target_x = 6'd3;
    wire [GRID_BITS-1:0]  target_y = 6'd3;

    // Interconnect Signals
    wire                  can_move;
    wire                  step;
    wire                  turn_right;
    wire                  done;
    wire                  failed;

    wire [GRID_BITS-1:0]  agent_x;
    wire [GRID_BITS-1:0]  agent_y;
    wire [1:0]            agent_dir;

    // Verification Counters and Flags
    integer               check_count = 0;
    integer               error_count = 0;
    integer               step_count  = 0;
    integer               turn_count  = 0;

    reg                   prev_step       = 1'b0;
    reg                   prev_turn_right = 1'b0;

    // Instantiate Unit Under Test: Maze Solver RTL Core
    maze_solver #(
        .GRID_BITS(GRID_BITS),
        .STACK_DEPTH(STACK_DEPTH)
    ) u_solver (
        .clk(clk),
        .rst_n(rst_n),
        .i_start(i_start),
        .i_start_x(start_x),
        .i_start_y(start_y),
        .i_target_x(target_x),
        .i_target_y(target_y),
        .i_can_move(can_move),
        .o_step(step),
        .o_turn_right(turn_right),
        .o_done(done),
        .o_failed(failed)
    );

    // Instantiate Simulation Model: 7x7 Inward Spiral Physical Maze Environment
    spiral_maze_env #(
        .GRID_BITS(GRID_BITS)
    ) u_env (
        .clk(clk),
        .rst_n(rst_n),
        .i_step(step),
        .i_turn_right(turn_right),
        .o_can_move(can_move),
        .o_agent_x(agent_x),
        .o_agent_y(agent_y),
        .o_agent_dir(agent_dir)
    );

    // Clock Generator: 100 MHz (5 ns high / 5 ns low)
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Continuous Protocol & Physical Rule Assertions
    always @(posedge clk) begin
        if (rst_n) begin
            // Rule 1: Collision check - agent must never step into a wall or boundary
            if (step && !can_move) begin
                $display("  [ASSERTION FAIL] Time %0t ns: Collision detected! Agent stepped when can_move=0 at (%0d, %0d) dir=%b",
                         $time, agent_x, agent_y, agent_dir);
                error_count = error_count + 1;
            end

            // Rule 2: Single-cycle synchronous pulse convention for o_step
            if (step && prev_step) begin
                $display("  [ASSERTION FAIL] Time %0t ns: o_step was asserted for multiple consecutive cycles!", $time);
                error_count = error_count + 1;
            end

            // Rule 3: Single-cycle synchronous pulse convention for o_turn_right
            if (turn_right && prev_turn_right) begin
                $display("  [ASSERTION FAIL] Time %0t ns: o_turn_right was asserted for multiple consecutive cycles!", $time);
                error_count = error_count + 1;
            end

            // Rule 4: Mutual exclusion - cannot step and turn in the exact same cycle
            if (step && turn_right) begin
                $display("  [ASSERTION FAIL] Time %0t ns: Simultaneous step and turn_right asserted!", $time);
                error_count = error_count + 1;
            end

            // Rule 5: Coordinates must stay within maze dimensions (7x7: 0..6)
            if (agent_x > 6'd6 || agent_y > 6'd6) begin
                $display("  [ASSERTION FAIL] Time %0t ns: Agent out of bounds at (%0d, %0d)!", $time, agent_x, agent_y);
                error_count = error_count + 1;
            end

            // Track movement statistics
            if (step)       step_count = step_count + 1;
            if (turn_right) turn_count = turn_count + 1;

            prev_step       <= step;
            prev_turn_right <= turn_right;
        end else begin
            prev_step       <= 1'b0;
            prev_turn_right <= 1'b0;
        end
    end

    // Main Test Stimulus and Verification Sequence
    initial begin
        $display("\n========================================================");
        $display("  STARTING SELF-CHECKING TEST: tb_maze_solver");
        $display("  Configuration: %0dx%0d Grid, Stack Depth = %0d", (1 << GRID_BITS), (1 << GRID_BITS), STACK_DEPTH);
        $display("  Entrance: (%0d, %0d) | Target: (%0d, %0d)", start_x, start_y, target_x, target_y);
        $display("========================================================\n");

        // Waveform dumping for debug and visualization
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_maze_solver);

        // Initialize signals
        clk     = 0;
        rst_n   = 0;
        i_start = 0;

        // Apply Reset
        #20;
        // Verify reset behavior
        check_count = check_count + 1;
        if (done !== 1'b0 || failed !== 1'b0 || step !== 1'b0 || turn_right !== 1'b0) begin
            $display("  [ASSERTION FAIL] Time %0t ns: Control outputs not zero during reset!", $time);
            error_count = error_count + 1;
        end else begin
            $display("  [CHECK %0d PASS] Reset state deassertions verified.", check_count);
        end

        // Release Reset
        rst_n = 1;
        #20;

        // Start Maze Solver search
        $display("  [INFO] Asserting i_start strobe...");
        i_start = 1;
        #CLK_PERIOD;
        i_start = 0;

        // Display real-time progress monitor
        $monitor("  [STEP] Time: %0t ns | Agent Pos: (%0d, %0d) | Dir: %b | Step: %b | Turn: %b", 
                 $time, agent_x, agent_y, agent_dir, step, turn_right);

        // Wait for solver completion or watchdog timeout
        fork : wait_for_completion
            begin
                wait (done || failed);
                $display("\n  [INFO] Solver signaled completion: done=%b, failed=%b at %0t ns", done, failed, $time);

                // Stop active monitor before checking assertions
                $monitoroff;

                // Self-Checking Assertion 1: o_done must be high
                check_count = check_count + 1;
                if (done !== 1'b1) begin
                    $display("  [ASSERTION FAIL] Check %0d: Expected o_done=1, got %b", check_count, done);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] o_done correctly asserted high.", check_count);
                end

                // Self-Checking Assertion 2: o_failed must be low
                check_count = check_count + 1;
                if (failed !== 1'b0) begin
                    $display("  [ASSERTION FAIL] Check %0d: Expected o_failed=0, got %b", check_count, failed);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] o_failed correctly low (no failure).", check_count);
                end

                // Self-Checking Assertion 3: Agent X coordinate must match target X
                check_count = check_count + 1;
                if (agent_x !== target_x) begin
                    $display("  [ASSERTION FAIL] Check %0d: Final agent_x=%0d does not match target_x=%0d", 
                             check_count, agent_x, target_x);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] Final agent_x matched target_x (%0d).", check_count, target_x);
                end

                // Self-Checking Assertion 4: Agent Y coordinate must match target Y
                check_count = check_count + 1;
                if (agent_y !== target_y) begin
                    $display("  [ASSERTION FAIL] Check %0d: Final agent_y=%0d does not match target_y=%0d", 
                             check_count, agent_y, target_y);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] Final agent_y matched target_y (%0d).", check_count, target_y);
                end

                // Self-Checking Assertion 5: Solver internal coordinate agreement
                check_count = check_count + 1;
                if (u_solver.curr_x !== target_x || u_solver.curr_y !== target_y) begin
                    $display("  [ASSERTION FAIL] Check %0d: Solver internal curr pos (%0d,%0d) != target (%0d,%0d)",
                             check_count, u_solver.curr_x, u_solver.curr_y, target_x, target_y);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] Solver internal coordinates synchronized with target.", check_count);
                end

                // Post-completion stability verification: wait 10 clock cycles
                # (10 * CLK_PERIOD);
                check_count = check_count + 1;
                if (done !== 1'b1 || step !== 1'b0 || turn_right !== 1'b0) begin
                    $display("  [ASSERTION FAIL] Check %0d: Instability after completion! done=%b, step=%b, turn=%b",
                             check_count, done, step, turn_right);
                    error_count = error_count + 1;
                end else begin
                    $display("  [CHECK %0d PASS] Post-completion state remains stable (done=1, step=0, turn=0).", check_count);
                end

                // Final summary output
                $display("\n========================================================");
                $display("                TEST EXECUTION SUMMARY                  ");
                $display("========================================================");
                $display("  Total Checks Executed : %0d", check_count);
                $display("  Total Errors Found    : %0d", error_count);
                $display("  Total Steps Taken     : %0d", step_count);
                $display("  Total Turns Executed  : %0d", turn_count);
                $display("  Target Cell Reached   : (%0d, %0d)", agent_x, agent_y);
                $display("========================================================");

                if (error_count == 0) begin
                    $display("\n  >>> PASS: tb_maze_solver PASSED ALL CHECKS SUCCESSFULLY! <<<\n");
                    $finish;
                end else begin
                    $display("\n  >>> FAIL: tb_maze_solver FAILED with %0d error(s)! <<<\n", error_count);
                    $fatal(1, "Testbench assertion failure.");
                end
            end

            // Watchdog Timeout (20,000 ns = 2,000 cycles)
            begin
                #20000;
                $display("\n========================================================");
                $display("  [FATAL TIMEOUT] Simulation watchdog timer expired!");
                $display("  Solver failed to reach target within 20,000 ns.");
                $display("  Current Position: (%0d, %0d) | Dir: %b", agent_x, agent_y, agent_dir);
                $display("========================================================\n");
                $fatal(1, "Watchdog timeout reached.");
            end
        join
    end

endmodule
