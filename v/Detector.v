module Detector (
    input clk,             
    input rst_n,           
    input VGA_VS,          
    input VGA_HS,          
    input [12:0] x_coord,   
    input [12:0] y_coord,   
    input pixel_is_black,  
    input iPAUSE,           // KEY[2] freezes boxes, 8x8 grids, and recognized text
    input iALPHA_MODE,      // SW[2] UP = Force Letter Priority (O/I/S/B/Z)
    
    output reg [12:0] X_MIN0, output reg [12:0] X_MAX0, output reg [12:0] Y_MIN0, output reg [12:0] Y_MAX0,
    output reg [12:0] X_MIN1, output reg [12:0] X_MAX1, output reg [12:0] Y_MIN1, output reg [12:0] Y_MAX1,
    output reg [12:0] X_MIN2, output reg [12:0] X_MAX2, output reg [12:0] Y_MIN2, output reg [12:0] Y_MAX2,
    output reg [12:0] X_MIN3, output reg [12:0] X_MAX3, output reg [12:0] Y_MIN3, output reg [12:0] Y_MAX3,
    output reg [12:0] X_MIN4, output reg [12:0] X_MAX4, output reg [12:0] Y_MIN4, output reg [12:0] Y_MAX4,
    output reg [3:0]  oCHAR_COUNT,
    output reg [7:0]  oCHAR0,
    output reg [7:0]  oCHAR1,
    output reg [7:0]  oCHAR2,
    output reg [7:0]  oCHAR3,
    output reg [7:0]  oCHAR4,

    // VGA On-Screen Display (OSD) Outputs for Top-Left Corner & Box Labels
    output wire       oOSD_BG,
    output wire       oOSD_TEXT,
    output wire       oOSD_GRID_ON,
    output wire       oOSD_GRID_OFF
);

    // Safe Detection Window (600x235)
    localparam [12:0] SAFE_X_MIN = 13'd310;
    localparam [12:0] SAFE_X_MAX = 13'd909; // 600 columns (0..599)
    localparam [12:0] SAFE_Y_MIN = 13'd130;
    localparam [12:0] SAFE_Y_MAX = 13'd365;

    // 5-pixel white gap tolerance
    localparam [4:0]  GAP_TOL    = 5'd5;

    wire in_safe_zone = (x_coord >= SAFE_X_MIN && x_coord <= SAFE_X_MAX &&
                         y_coord >= SAFE_Y_MIN && y_coord <= SAFE_Y_MAX);
    wire [9:0] x_rel  = in_safe_zone ? (x_coord - SAFE_X_MIN) : 10'd0;
    wire [5:0] strip_idx = x_rel[9:4]; // 38 vertical strips (16px wide each, 0..37)

    // Horizontal stroke width counter (up to 180px)
    reg [7:0] black_run;
    always @(posedge clk) begin
        if (!rst_n || !in_safe_zone)
            black_run <= 8'd0;
        else if (pixel_is_black)
            black_run <= (black_run == 8'd255) ? 8'd255 : (black_run + 8'd1);
        else
            black_run <= 8'd0;
    end

    wire h_valid_ink = in_safe_zone && (black_run >= 8'd3) && (black_run <= 8'd180);
    wire h_core_ink  = in_safe_zone && (black_run >= 8'd5) && (black_run <= 8'd180);

    // 2-row vertical persistence filter
    reg [599:0] prev_row_ink;
    wire valid_ink = h_valid_ink && prev_row_ink[x_rel];
    wire grid_ink  = h_core_ink  && prev_row_ink[x_rel];

    // 600-bit column projection bitmap
    reg [599:0] col_ink;

    // 38 vertical strip Y-trackers
    reg [12:0] strip_y_min [0:37];
    reg [12:0] strip_y_max [0:37];

    // Live, Previous-Frame, and Latched 8x8 Normalized Grids (bit 63 = Top-Left (0,0), bit 0 = Bottom-Right (7,7))
    reg [63:0] cur_grid0,  cur_grid1,  cur_grid2,  cur_grid3,  cur_grid4;
    reg [63:0] prev_grid0, prev_grid1, prev_grid2, prev_grid3, prev_grid4;
    reg [63:0] sampled_grid0, sampled_grid1, sampled_grid2, sampled_grid3, sampled_grid4;

    // Combine current + previous frame grids to eliminate 60Hz single-frame camera flicker
    wire [63:0] stable_grid0 = cur_grid0 | prev_grid0;
    wire [63:0] stable_grid1 = cur_grid1 | prev_grid1;
    wire [63:0] stable_grid2 = cur_grid2 | prev_grid2;
    wire [63:0] stable_grid3 = cur_grid3 | prev_grid3;
    wire [63:0] stable_grid4 = cur_grid4 | prev_grid4;

    // Frame divider for rock-solid OSD display (updates prediction every 4 frames = ~15Hz)
    reg [1:0] frame_div;

    // Helper function for bounding-box coordinate hysteresis (ignores 1-3px camera jitter)
    function [12:0] smooth_coord(input [12:0] old_c, input [12:0] new_c);
        reg [12:0] diff;
        begin
            diff = (new_c > old_c) ? (new_c - old_c) : (old_c - new_c);
            smooth_coord = (old_c != 13'd0 && diff <= 13'd3) ? old_c : new_c;
        end
    endfunction

    // Exact 8x8 Zone Index Calculator (0..63)
    function [5:0] get_zone8x8(
        input [12:0] x, input [12:0] y,
        input [12:0] xmin, input [12:0] xmax,
        input [12:0] ymin, input [12:0] ymax
    );
        reg [12:0] xc, w, h, dx, dy;
        reg [15:0] dx8, dy8, w1, w2, w3, w4, w5, w6, w7, h1, h2, h3, h4, h5, h6, h7;
        reg [2:0]  c, r, inv_r, inv_c;
        begin
            xc  = (x > xmin + 13'd3) ? (x - 13'd3) : xmin;
            w   = (xmax > xmin) ? (xmax - xmin) : 13'd1;
            h   = (ymax > ymin) ? (ymax - ymin) : 13'd1;
            dx  = (xc > xmin) ? (xc - xmin) : 13'd0;
            dy  = (y  > ymin) ? (y  - ymin) : 13'd0;

            dx8 = {dx, 3'b000};
            dy8 = {dy, 3'b000};

            w1  = {3'b000, w};
            w2  = {2'b00, w, 1'b0};
            w3  = w2 + w1;
            w4  = {1'b0, w, 2'b00};
            w5  = w4 + w1;
            w6  = w4 + w2;
            w7  = {w, 3'b000} - w1;

            h1  = {3'b000, h};
            h2  = {2'b00, h, 1'b0};
            h3  = h2 + h1;
            h4  = {1'b0, h, 2'b00};
            h5  = h4 + h1;
            h6  = h4 + h2;
            h7  = {h, 3'b000} - h1;

            c = (dx8 < w4) ? ((dx8 < w2) ? ((dx8 < w1) ? 3'd0 : 3'd1) : ((dx8 < w3) ? 3'd2 : 3'd3))
                           : ((dx8 < w6) ? ((dx8 < w5) ? 3'd4 : 3'd5) : ((dx8 < w7) ? 3'd6 : 3'd7));

            r = (dy8 < h4) ? ((dy8 < h2) ? ((dy8 < h1) ? 3'd0 : 3'd1) : ((dy8 < h3) ? 3'd2 : 3'd3))
                           : ((dy8 < h6) ? ((dy8 < h5) ? 3'd4 : 3'd5) : ((dy8 < h7) ? 3'd6 : 3'd7));

            inv_r = 3'd7 - r;
            inv_c = 3'd7 - c;
            get_zone8x8 = {inv_r, inv_c};
        end
    endfunction

    function [6:0] popcount64(input [63:0] v);
        integer k;
        reg [6:0] sum;
        begin
            sum = 7'd0;
            for (k = 0; k < 64; k = k + 1) begin
                sum = sum + v[k];
            end
            popcount64 = sum;
        end
    endfunction

    // =========================================================================
    // CALIBRATED 8x8 HIT-OR-MISS + TOPOLOGICAL CLASSIFIER
    // =========================================================================
    // Row 0 (Top):    g[63:56]   (c0=63, c1=62, c2=61, c3=60, c4=59, c5=58, c6=57, c7=56)
    // Row 1:          g[55:48]   (c0=55, c1=54, c2=53, c3=52, c4=51, c5=50, c6=49, c7=48)
    // Row 2:          g[47:40]   (c0=47, c1=46, c2=45, c3=44, c4=43, c5=42, c6=41, c7=40)
    // Row 3 (Mid-Up): g[39:32]   (c0=39, c1=38, c2=37, c3=36, c4=35, c5=34, c6=33, c7=32)
    // Row 4 (Mid-Lo): g[31:24]   (c0=31, c1=30, c2=29, c3=28, c4=27, c5=26, c6=25, c7=24)
    // Row 5:          g[23:16]   (c0=23, c1=22, c2=21, c3=20, c4=19, c5=18, c6=17, c7=16)
    // Row 6:          g[15:8]    (c0=15, c1=14, c2=13, c3=12, c4=11, c5=10, c6=9,  c7=8)
    // Row 7 (Bottom): g[7:0]     (c0=7,  c1=6,  c2=5,  c3=4,  c4=3,  c5=2,  c6=1,  c7=0)
    function [7:0] classify_box8x8(
        input [63:0] g,
        input [12:0] xmin, input [12:0] xmax,
        input [12:0] ymin, input [12:0] ymax
    );
        reg [12:0] w, h;
        reg l_top, m_top, r_top;
        reg l_up,  m_up,  r_up;
        reg l_mid, m_mid, r_mid;
        reg l_lo,  m_lo,  r_lo;
        reg l_bot, m_bot, r_bot;
        reg bot_row_mid_open, top_row_mid_open, bot_cavity_open;
        reg l_spine, r_spine, c_spine;
        reg has_crossbar, center_hollow;
        reg mid_left_empty, mid_right_empty;
        reg [3:0] top_row_cnt;
        reg [6:0] d0, d2, d3, d4, d5, d6, d7, d8, d9;
        reg [6:0] dA, dB, dC, dD, dE, dF, dG, dH, dI, dJ, dK, dL, dM, dN, dP, dQ, dR, dS, dT, dU, dV, dW, dX, dY, dZ;
        reg [6:0] da_lc, db_lc, dd_lc, de_lc, df_lc;
        reg [6:0] min_d;
        reg [7:0] best_c;
        begin
            w = (xmax > xmin) ? (xmax - xmin) : 13'd0;
            h = (ymax > ymin) ? (ymax - ymin) : 13'd0;

            l_top = g[63] | g[62] | g[55] | g[54];
            m_top = g[60] | g[59] | g[52] | g[51];
            r_top = g[57] | g[56] | g[49] | g[48];

            l_up  = g[47] | g[46] | g[45];
            m_up  = g[44] | g[43] | g[52] | g[51];
            r_up  = g[42] | g[41] | g[40];

            // Include column 2 (g[37], g[29]) in l_mid so 'D' with a left serif still registers a left wall
            l_mid = g[39] | g[38] | g[37] | g[31] | g[30] | g[29];
            m_mid = g[36] | g[35] | g[28] | g[27];
            r_mid = g[34] | g[33] | g[32] | g[26] | g[25] | g[24];

            l_lo  = g[23] | g[22] | g[21];
            m_lo  = g[20] | g[19] | g[12] | g[11];
            r_lo  = g[18] | g[17] | g[16];

            l_bot = g[15] | g[14] | g[7]  | g[6];
            m_bot = g[4]  | g[3];
            r_bot = g[9]  | g[8]  | g[1]  | g[0];

            bot_row_mid_open = !(g[4] & g[3]) && !(g[4] & g[5] & g[2]);
            bot_cavity_open  = !g[12] && !g[11] && !g[4] && !g[3];
            top_row_mid_open = !(g[60] & g[59]) && !(g[52] & g[51]);

            l_spine = (l_top | l_up) & (g[39] | g[38] | g[31] | g[30]) & (l_lo | l_bot);
            r_spine = (r_top | r_up) & (g[33] | g[32] | g[25] | g[24]) & (r_lo | r_bot);
            c_spine = (g[52] | g[51] | g[44] | g[43]) & m_mid & (g[20] | g[19] | g[12] | g[11]);

            // Strict empty side-margins across Rows 2..5 so 'I' never hijacks '2'
            mid_left_empty  = !(g[47] | g[46] | g[39] | g[38] | g[31] | g[30] | g[23] | g[22]);
            mid_right_empty = !(g[41] | g[40] | g[33] | g[32] | g[25] | g[24] | g[17] | g[16]);

            has_crossbar  = (g[37] & g[36] & g[35] & g[34]) |
                            (g[29] & g[28] & g[27] & g[26]) |
                            (g[21] & g[20] & g[19] & g[18]) |
                            ((m_mid | g[20] | g[19]) & (g[37] | g[29] | g[21]) & (g[34] | g[26] | g[18]));

            center_hollow = !(g[36] | g[35] | g[28] | g[27]);
            top_row_cnt   = g[63] + g[62] + g[61] + g[60] + g[59] + g[58] + g[57] + g[56];

            if (w == 13'd0 || h == 13'd0) begin
                classify_box8x8 = 8'h2D; // '-'
            end
            // 1. Tall & Narrow Stroke -> '1' (or 'I' via context)
            else if (h > (w << 1) + (w >> 2) || w <= 13'd16) begin
                classify_box8x8 = "1";
            end
            // 2. 'J' vs Serifed 'I' vs 'T' (Requires BOTH mid-left AND mid-right across Rows 2..5 to be empty so '2' is never hijacked!)
            else if (m_top && c_spine && mid_left_empty && mid_right_empty) begin
                if ((l_bot | g[5]) && !g[1] && !g[0] && !g[8])
                    classify_box8x8 = "J";
                else if ((g[5] | g[6] | l_bot) && (g[2] | g[1] | r_bot) && (g[4] | g[3]))
                    classify_box8x8 = "I";
                else if (l_top && r_top && !l_bot && !r_bot)
                    classify_box8x8 = "T";
                else
                    classify_box8x8 = "I";
            end
            // 3. 'J' with hook entering Row 5 left (Top bar, Center/Right stem, Empty Upper-Left & Mid-Left, Bottom-Left Hook, Empty Bottom-Right)
            else if (m_top && !g[47] && !g[46] && !g[39] && !g[38] && !g[31] && !g[30] && (l_bot | g[5]) && (g[12] | g[11] | g[4] | g[3]) && !g[1] && !g[0] && !g[8]) begin
                classify_box8x8 = "J";
            end
            // 4. 'T' (Full Top Bar, Center Stem, Open Lower-Left & Open Lower-Right)
            else if (l_top && m_top && r_top && c_spine && !g[39] && !g[31] && !l_lo && !l_bot && !g[32] && !g[24] && !r_lo && !r_bot) begin
                classify_box8x8 = "T";
            end
            // 5. 'L' (Left Spine, Empty Top-Center/Right, Empty Mid-Center/Right, Bottom Bar)
            else if (l_spine && !m_top && !r_top && !r_up && !m_mid && !g[32] && !g[24] && (m_bot | r_bot)) begin
                classify_box8x8 = "L";
            end
            // 6. 'P' vs 'F' (Left Spine, Top Bar, Middle Bar, Empty Bottom-Center & Empty Bottom-Right)
            else if (l_spine && m_top && (m_mid | g[37] | g[29]) && !g[4] && !g[3] && !g[2] && !r_bot && !r_lo) begin
                if (r_up || r_top || g[41] || g[33])
                    classify_box8x8 = "P";
                else
                    classify_box8x8 = "F";
            end
            // 7. 'V' (Open Top-Center, Upper Left & Right Walls, Converges to Bottom-Center Point with Empty Bottom-Left & Bottom-Right Corners)
            else if (top_row_mid_open && (l_top | l_up) && (r_top | r_up) && !g[7] && !g[6] && !g[1] && !g[0] && (g[4] | g[3] | g[12] | g[11])) begin
                classify_box8x8 = "V";
            end
            // 8. 'U' (Open Top-Center, Left & Right Spines, Hollow Interior Center, Closed Continuous Bottom Bar)
            else if (top_row_mid_open && center_hollow && !g[44] && !g[43] && !g[45] && !g[37] && (l_up | l_mid) && (r_up | r_mid) && (g[4] | g[3]) && (g[5] | g[2])) begin
                classify_box8x8 = "U";
            end
            // 9. 'M' (Full Left & Right Spines, Open Bottom Cavity Rows 6..7, Upper-Inner Diagonals Meeting at Center V)
            else if (l_spine && r_spine && bot_cavity_open && (g[54] | g[53] | g[46] | g[45]) && (g[50] | g[49] | g[42] | g[41]) && m_mid) begin
                classify_box8x8 = "M";
            end
            // 10. 'N' vs 'H' vs 'W' (Left & Right Vertical Spines + Open Top-Center + Open Bottom-Center)
            else if (l_spine && r_spine && top_row_mid_open && bot_row_mid_open) begin
                if (has_crossbar && !g[44] && !g[43] && !g[20] && !g[19] && !g[45] && !g[18])
                    classify_box8x8 = "H";
                else if ((g[21] && g[18]) || (g[13] && g[10]))
                    classify_box8x8 = "W";
                else
                    classify_box8x8 = "N";
            end
            // 11. '2' vs 'Z' (Top Arch/Bar, Upper-Right Curve, Center Diagonal, Lower-Left Leg, Full Bottom Base, Open Upper-Left & Open Lower-Right Cavities)
            else if ((m_top | g[61] | g[53]) && (r_top | r_up) && (m_mid | m_lo) && (l_lo | l_bot) &&
                     (g[4] | g[3] | g[2]) && (g[7] | g[6] | g[5]) &&
                     !g[39] && !g[38] && !g[24] && !g[16]) begin
                if (g[63] && g[56] && !g[48])
                    classify_box8x8 = "Z";
                else
                    classify_box8x8 = "2";
            end
            // 12. '6' (Left Wall, Empty Upper-Right Quadrant, Closed Bottom Loop with Middle Bar & Lower-Right Wall)
            else if ((l_up | g[54] | g[53]) && (g[39] | g[38] | g[31] | g[30]) && (l_lo | l_bot) &&
                     !g[57] && !g[56] && !g[49] && !g[48] && !g[41] && !g[40] &&
                     (m_mid | g[20] | g[19]) && (r_lo | g[9] | g[8]) && (g[4] | g[3] | g[5] | g[2])) begin
                classify_box8x8 = "6";
            end
            // 13. 'A' vs 'R' (Closed Top-Center, Middle Crossbar/Junction, Open Bottom-Center, Left & Right Bottom Legs)
            else if (m_top && (has_crossbar || m_mid || m_lo) && bot_row_mid_open && l_bot && r_bot) begin
                if ((!g[63] && !g[62]) || (!g[63] && !g[55]) || (!g[56] && !g[48]) || (top_row_cnt <= 4'd5 && !g[63]))
                    classify_box8x8 = "A";
                else
                    classify_box8x8 = "R";
            end
            // 14. 'X' vs 'Y' (Open Top-Center, 4 Corners Active, Center Intersection, Open Mid-Left & Open Mid-Right Edges)
            else if (top_row_mid_open && l_top && r_top && m_mid && !g[39] && !g[31] && !g[32] && !g[24]) begin
                if (!l_bot && !r_bot && (g[4] | g[3]))
                    classify_box8x8 = "Y";
                else
                    classify_box8x8 = "X";
            end
            // 15. 'K' (Left Spine, Open Top-Center & Open Bottom-Center, Upper-Right & Lower-Right Arms, Open Mid-Right Edge)
            else if (l_spine && top_row_mid_open && bot_row_mid_open && r_top && r_bot && !g[32] && !g[24]) begin
                classify_box8x8 = "K";
            end
            // 16. 'D' vs '0' / 'O' vs 'Q' (Hollow Center Core, Closed Top & Bottom, Left & Right Walls)
            else if (center_hollow && m_top && (g[4] | g[3] | g[12] | g[11]) && (l_up | l_mid | l_lo) && (r_up | r_mid | r_lo)) begin
                // 'D': Sharp Left Corners (Top-Left g[63:61] & Bottom-Left g[7:5]) with Rounded Right Corners (!g[56] && !g[0])
                if ((g[63] | g[62] | g[55]) && (g[15] | g[7] | g[6] | g[5]) && l_mid && (!g[56] || !g[0]))
                    classify_box8x8 = "D";
                else if (g[0] && (g[1] | g[8]) && (g[18] | g[10]) && !g[63])
                    classify_box8x8 = "Q";
                else
                    classify_box8x8 = "0";
            end
            // 17. 'C' vs 'G' (Hollow Center, Left Wall, Top & Bottom Bars, Open Upper-Right)
            else if (center_hollow && m_top && (g[4] | g[3]) && (l_up | l_mid | l_lo) && !g[40] && !g[41]) begin
                if (r_lo || r_mid || g[26] || g[25] || g[17])
                    classify_box8x8 = "G";
                else
                    classify_box8x8 = "C";
            end
            // 18. 'E' (Left Spine, Top Bar, Mid Bar, Continuous Bottom Bar, Open Upper-Right & Open Lower-Right)
            else if (l_spine && m_top && m_mid && (g[4] | g[3]) && !g[48] && !g[40] && !g[24] && !g[16]) begin
                classify_box8x8 = "E";
            end
            // 19. '7' (Top Bar, Open Lower-Left !l_lo & !l_bot, Open Bottom-Right !r_bot)
            else if (m_top && r_top && !l_up && !l_lo && !l_bot && !r_bot) begin
                classify_box8x8 = "7";
            end
            // 20. '4' (Open Top-Center, Upper Prongs, Middle Bar, Right Stem, Open Bottom-Left)
            else if (top_row_mid_open && (l_up | l_mid) && r_spine && m_mid && !l_bot && !g[4] && !g[3]) begin
                classify_box8x8 = "4";
            end
            // 21. 'S' / '5' vs '3' vs 'B' / '8' vs '9'
            else if (m_top && m_mid && (g[4] | g[3])) begin
                if (l_up && r_up && !l_lo && !l_bot && (r_lo | r_bot))
                    classify_box8x8 = "9";
                else if (!l_up && !l_lo && r_up && r_lo)
                    classify_box8x8 = "3";
                else if (l_up && r_lo && (!g[40] || !g[23])) begin
                    if (g[63] && g[56] && !g[40])
                        classify_box8x8 = "5";
                    else
                        classify_box8x8 = "S";
                end
                else if (l_up && l_lo && r_up && r_lo) begin
                    if (g[63] && g[7])
                        classify_box8x8 = "B";
                    else
                        classify_box8x8 = "8";
                end
                else
                    classify_box8x8 = "S";
            end
            // 22. Full 64-Bit (8x8) Hamming Distance Template Matcher
            else begin
                dA    = popcount64(g ^ 64'h18_3C_66_66_7E_66_66_66);
                dB    = popcount64(g ^ 64'h7C_66_66_7C_66_66_66_7C);
                dC    = popcount64(g ^ 64'h3C_66_60_60_60_60_66_3C);
                dD    = popcount64(g ^ 64'h78_6C_66_66_66_66_6C_78);
                dE    = popcount64(g ^ 64'h7E_60_60_7C_60_60_60_7E);
                dF    = popcount64(g ^ 64'h7E_60_60_7C_60_60_60_60);
                dG    = popcount64(g ^ 64'h3C_66_60_60_6E_66_66_3E);
                dH    = popcount64(g ^ 64'h66_66_66_7E_66_66_66_66);
                dI    = popcount64(g ^ 64'h3C_18_18_18_18_18_18_3C);
                dJ    = popcount64(g ^ 64'h1E_0C_0C_0C_0C_6C_6C_38);
                dK    = popcount64(g ^ 64'h66_6C_78_70_78_6C_66_66);
                dL    = popcount64(g ^ 64'h60_60_60_60_60_60_60_7E);
                dM    = popcount64(g ^ 64'h63_77_7F_6B_63_63_63_63);
                dN    = popcount64(g ^ 64'h66_76_7E_7E_6E_66_66_66);
                d0    = popcount64(g ^ 64'h3C_66_66_66_66_66_66_3C);
                dP    = popcount64(g ^ 64'h7C_66_66_7C_60_60_60_60);
                dQ    = popcount64(g ^ 64'h3C_66_66_66_66_6E_3C_0E);
                dR    = popcount64(g ^ 64'h7C_66_66_7C_78_6C_66_66);
                dS    = popcount64(g ^ 64'h3C_66_60_3C_06_06_66_3C);
                dT    = popcount64(g ^ 64'h7E_18_18_18_18_18_18_18);
                dU    = popcount64(g ^ 64'h66_66_66_66_66_66_66_3C);
                dV    = popcount64(g ^ 64'h66_66_66_66_66_3C_3C_18);
                dW    = popcount64(g ^ 64'h63_63_63_6B_7F_77_63_63);
                dX    = popcount64(g ^ 64'h66_66_3C_18_18_3C_66_66);
                dY    = popcount64(g ^ 64'h66_66_3C_18_18_18_18_18);
                dZ    = popcount64(g ^ 64'h7E_06_0C_18_30_60_60_7E);
                d2    = popcount64(g ^ 64'h3C_66_06_0C_18_30_60_7E);
                d3    = popcount64(g ^ 64'h3C_66_06_1C_06_06_66_3C);
                d4    = popcount64(g ^ 64'h0C_1C_3C_6C_7E_0C_0C_0C);
                d5    = popcount64(g ^ 64'h7E_60_60_7C_06_06_66_3C);
                d6    = popcount64(g ^ 64'h3C_60_60_7C_66_66_66_3C);
                d7    = popcount64(g ^ 64'h7E_06_0C_18_18_18_18_18);
                d8    = popcount64(g ^ 64'h3C_66_66_3C_66_66_66_3C);
                d9    = popcount64(g ^ 64'h3C_66_66_3E_06_06_66_3C);
                da_lc = popcount64(g ^ 64'h3C_06_3E_66_66_66_3F_03);
                db_lc = popcount64(g ^ 64'h60_60_60_7C_66_66_66_7C);
                dd_lc = popcount64(g ^ 64'h06_06_06_3E_66_66_66_3E);
                de_lc = popcount64(g ^ 64'h3C_66_66_7E_60_60_66_3C);
                df_lc = popcount64(g ^ 64'h1E_30_30_7C_30_30_30_30);

                min_d = dA; best_c = "A";
                if (da_lc < min_d) begin min_d = da_lc; best_c = "A"; end
                if (dB    < min_d) begin min_d = dB;    best_c = "B"; end
                if (db_lc < min_d) begin min_d = db_lc; best_c = "B"; end
                if (dC    < min_d) begin min_d = dC;    best_c = "C"; end
                if (dD    < min_d) begin min_d = dD;    best_c = "D"; end
                if (dd_lc < min_d) begin min_d = dd_lc; best_c = "D"; end
                if (dE    < min_d) begin min_d = dE;    best_c = "E"; end
                if (de_lc < min_d) begin min_d = de_lc; best_c = "E"; end
                if (dF    < min_d) begin min_d = dF;    best_c = "F"; end
                if (df_lc < min_d) begin min_d = df_lc; best_c = "F"; end
                if (dG    < min_d) begin min_d = dG;    best_c = "G"; end
                if (dH    < min_d) begin min_d = dH;    best_c = "H"; end
                if (dI    < min_d) begin min_d = dI;    best_c = "I"; end
                if (dJ    < min_d) begin min_d = dJ;    best_c = "J"; end
                if (dK    < min_d) begin min_d = dK;    best_c = "K"; end
                if (dL    < min_d) begin min_d = dL;    best_c = "L"; end
                if (dM    < min_d) begin min_d = dM;    best_c = "M"; end
                if (dN    < min_d) begin min_d = dN;    best_c = "N"; end
                if (d0    < min_d) begin min_d = d0;    best_c = "0"; end
                if (dP    < min_d) begin min_d = dP;    best_c = "P"; end
                if (dQ    < min_d) begin min_d = dQ;    best_c = "Q"; end
                if (dR    < min_d) begin min_d = dR;    best_c = "R"; end
                if (dS    < min_d) begin min_d = dS;    best_c = "S"; end
                if (dT    < min_d) begin min_d = dT;    best_c = "T"; end
                if (dU    < min_d) begin min_d = dU;    best_c = "U"; end
                if (dV    < min_d) begin min_d = dV;    best_c = "V"; end
                if (dW    < min_d) begin min_d = dW;    best_c = "W"; end
                if (dX    < min_d) begin min_d = dX;    best_c = "X"; end
                if (dY    < min_d) begin min_d = dY;    best_c = "Y"; end
                if (dZ    < min_d) begin min_d = dZ;    best_c = "Z"; end
                if (d2    < min_d) begin min_d = d2;    best_c = "2"; end
                if (d3    < min_d) begin min_d = d3;    best_c = "3"; end
                if (d4    < min_d) begin min_d = d4;    best_c = "4"; end
                if (d5    < min_d) begin min_d = d5;    best_c = "5"; end
                if (d6    < min_d) begin min_d = d6;    best_c = "6"; end
                if (d7    < min_d) begin min_d = d7;    best_c = "7"; end
                if (d8    < min_d) begin min_d = d8;    best_c = "8"; end
                if (d9    < min_d) begin min_d = d9;    best_c = "9"; end
                classify_box8x8 = best_c;
            end
        end
    endfunction

    function is_pure_digit(input [7:0] c);
        begin
            is_pure_digit = (c >= "2" && c <= "9");
        end
    endfunction

    function is_pure_letter(input [7:0] c);
        begin
            is_pure_letter = (c >= "A" && c <= "Z" && c != "S" && c != "B" && c != "Z");
        end
    endfunction

    // Keep digits '2'..'9' intact in mixed alphanumeric strings (like "26MVD" or "AB901")
    function [7:0] resolve_context(input [7:0] c, input word_ctx, input num_ctx);
        begin
            if (word_ctx) begin
                case (c)
                    "0": resolve_context = "O";
                    "1": resolve_context = "I";
                    default: resolve_context = c;
                endcase
            end else if (num_ctx) begin
                case (c)
                    "S": resolve_context = "5";
                    "B": resolve_context = "8";
                    "O": resolve_context = "0";
                    "I": resolve_context = "1";
                    default: resolve_context = c;
                endcase
            end else begin
                resolve_context = c;
            end
        end
    endfunction

    wire [7:0] raw_c0 = classify_box8x8(stable_grid0, X_MIN0, X_MAX0, Y_MIN0, Y_MAX0);
    wire [7:0] raw_c1 = classify_box8x8(stable_grid1, X_MIN1, X_MAX1, Y_MIN1, Y_MAX1);
    wire [7:0] raw_c2 = classify_box8x8(stable_grid2, X_MIN2, X_MAX2, Y_MIN2, Y_MAX2);
    wire [7:0] raw_c3 = classify_box8x8(stable_grid3, X_MIN3, X_MAX3, Y_MIN3, Y_MAX3);
    wire [7:0] raw_c4 = classify_box8x8(stable_grid4, X_MIN4, X_MAX4, Y_MIN4, Y_MAX4);

    wire has_digit  = is_pure_digit(raw_c0)  | is_pure_digit(raw_c1)  | is_pure_digit(raw_c2)  |
                      is_pure_digit(raw_c3)  | is_pure_digit(raw_c4);
    wire has_letter = is_pure_letter(raw_c0) | is_pure_letter(raw_c1) | is_pure_letter(raw_c2) |
                      is_pure_letter(raw_c3) | is_pure_letter(raw_c4);
    wire use_word_ctx = iALPHA_MODE || (has_letter && !has_digit);
    wire use_num_ctx  = !iALPHA_MODE && (has_digit && !has_letter);

    // Segment Scanner Registers
    reg [1:0]  state;
    reg [9:0]  scan_x;
    reg        in_char;
    reg [9:0]  seg_start, seg_end;
    reg [12:0] seg_y_min, seg_y_max;
    reg [4:0]  gap_cnt;
    reg [2:0]  box_idx;
    reg        last_VGA_VS;

    reg [12:0] nx_min0, nx_max0, ny_min0, ny_max0;
    reg [12:0] nx_min1, nx_max1, ny_min1, ny_max1;
    reg [12:0] nx_min2, nx_max2, ny_min2, ny_max2;
    reg [12:0] nx_min3, nx_max3, ny_min3, ny_max3;
    reg [12:0] nx_min4, nx_max4, ny_min4, ny_max4;

    wire [9:0]  seg_width  = seg_end - seg_start;
    wire [12:0] seg_height = (seg_y_max > seg_y_min) ? (seg_y_max - seg_y_min) : 13'd0;

    wire is_border_shadow = ((seg_start <= 10'd2) || (seg_end >= 10'd597)) && (seg_height > 13'd170);
    wire is_flat_noise    = ({3'b000, seg_width} > (seg_height << 1)) && (seg_height < 13'd28);
    wire seg_valid        = (seg_width >= 10'd4) && (seg_width <= 10'd220) &&
                            (seg_height >= 13'd16) && (seg_height <= 13'd230) &&
                            !is_border_shadow && !is_flat_noise;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            col_ink      <= 600'd0;
            prev_row_ink <= 600'd0;
            for (i = 0; i < 38; i = i + 1) begin
                strip_y_min[i] <= 13'd2000;
                strip_y_max[i] <= 13'd0;
            end
            X_MIN0 <= 13'd0; X_MAX0 <= 13'd0; Y_MIN0 <= 13'd0; Y_MAX0 <= 13'd0;
            X_MIN1 <= 13'd0; X_MAX1 <= 13'd0; Y_MIN1 <= 13'd0; Y_MAX1 <= 13'd0;
            X_MIN2 <= 13'd0; X_MAX2 <= 13'd0; Y_MIN2 <= 13'd0; Y_MAX2 <= 13'd0;
            X_MIN3 <= 13'd0; X_MAX3 <= 13'd0; Y_MIN3 <= 13'd0; Y_MAX3 <= 13'd0;
            X_MIN4 <= 13'd0; X_MAX4 <= 13'd0; Y_MIN4 <= 13'd0; Y_MAX4 <= 13'd0;
            cur_grid0  <= 64'd0; cur_grid1  <= 64'd0; cur_grid2  <= 64'd0; cur_grid3  <= 64'd0; cur_grid4  <= 64'd0;
            prev_grid0 <= 64'd0; prev_grid1 <= 64'd0; prev_grid2 <= 64'd0; prev_grid3 <= 64'd0; prev_grid4 <= 64'd0;
            sampled_grid0 <= 64'd0; sampled_grid1 <= 64'd0; sampled_grid2 <= 64'd0; sampled_grid3 <= 64'd0; sampled_grid4 <= 64'd0;
            frame_div   <= 2'd0;
            oCHAR_COUNT <= 4'd0;
            oCHAR0 <= 8'h2D; oCHAR1 <= 8'h2D; oCHAR2 <= 8'h2D; oCHAR3 <= 8'h2D; oCHAR4 <= 8'h2D;
            state       <= 2'd0;
            last_VGA_VS <= 1'b1;
        end else begin
            last_VGA_VS <= VGA_VS;

            if (in_safe_zone) begin
                prev_row_ink[x_rel] <= h_valid_ink;
            end

            // 1. Live Pixel Accumulation for Bounding Boxes
            if (valid_ink) begin
                col_ink[x_rel] <= 1'b1;

                if (strip_y_min[strip_idx] == 13'd2000) begin
                    strip_y_min[strip_idx] <= y_coord;
                    strip_y_max[strip_idx] <= y_coord;
                end else if (y_coord <= strip_y_max[strip_idx] + 13'd45) begin
                    strip_y_max[strip_idx] <= y_coord;
                end else if ((strip_y_max[strip_idx] - strip_y_min[strip_idx]) < 13'd6) begin
                    strip_y_min[strip_idx] <= y_coord;
                    strip_y_max[strip_idx] <= y_coord;
                end
            end

            // 2. Eroded Core Sampling into 8x8 (64-bit) Grids
            if (grid_ink) begin
                if (X_MAX0 > X_MIN0 && x_coord >= X_MIN0 && x_coord <= X_MAX0 && y_coord >= Y_MIN0 && y_coord <= Y_MAX0)
                    cur_grid0[get_zone8x8(x_coord, y_coord, X_MIN0, X_MAX0, Y_MIN0, Y_MAX0)] <= 1'b1;
                if (X_MAX1 > X_MIN1 && x_coord >= X_MIN1 && x_coord <= X_MAX1 && y_coord >= Y_MIN1 && y_coord <= Y_MAX1)
                    cur_grid1[get_zone8x8(x_coord, y_coord, X_MIN1, X_MAX1, Y_MIN1, Y_MAX1)] <= 1'b1;
                if (X_MAX2 > X_MIN2 && x_coord >= X_MIN2 && x_coord <= X_MAX2 && y_coord >= Y_MIN2 && y_coord <= Y_MAX2)
                    cur_grid2[get_zone8x8(x_coord, y_coord, X_MIN2, X_MAX2, Y_MIN2, Y_MAX2)] <= 1'b1;
                if (X_MAX3 > X_MIN3 && x_coord >= X_MIN3 && x_coord <= X_MAX3 && y_coord >= Y_MIN3 && y_coord <= Y_MAX3)
                    cur_grid3[get_zone8x8(x_coord, y_coord, X_MIN3, X_MAX3, Y_MIN3, Y_MAX3)] <= 1'b1;
                if (X_MAX4 > X_MIN4 && x_coord >= X_MIN4 && x_coord <= X_MAX4 && y_coord >= Y_MIN4 && y_coord <= Y_MAX4)
                    cur_grid4[get_zone8x8(x_coord, y_coord, X_MIN4, X_MAX4, Y_MIN4, Y_MAX4)] <= 1'b1;
            end

            // 3. End-of-Frame Trigger (Falling Edge of VGA_VS)
            if (last_VGA_VS == 1'b1 && VGA_VS == 1'b0) begin
                if (!iPAUSE) begin
                    frame_div <= frame_div + 2'd1;

                    // Update displayed characters & HUD grids every 4 frames (~15Hz) to prevent 60Hz tearing
                    if (frame_div == 2'd0) begin
                        oCHAR0 <= resolve_context(raw_c0, use_word_ctx, use_num_ctx);
                        oCHAR1 <= resolve_context(raw_c1, use_word_ctx, use_num_ctx);
                        oCHAR2 <= resolve_context(raw_c2, use_word_ctx, use_num_ctx);
                        oCHAR3 <= resolve_context(raw_c3, use_word_ctx, use_num_ctx);
                        oCHAR4 <= resolve_context(raw_c4, use_word_ctx, use_num_ctx);

                        sampled_grid0 <= stable_grid0;
                        sampled_grid1 <= stable_grid1;
                        sampled_grid2 <= stable_grid2;
                        sampled_grid3 <= stable_grid3;
                        sampled_grid4 <= stable_grid4;
                    end

                    prev_grid0 <= cur_grid0;
                    prev_grid1 <= cur_grid1;
                    prev_grid2 <= cur_grid2;
                    prev_grid3 <= cur_grid3;
                    prev_grid4 <= cur_grid4;

                    state     <= 2'd1;
                    scan_x    <= 10'd0;
                    in_char   <= 1'b0;
                    seg_start <= 10'd0;
                    seg_end   <= 10'd0;
                    seg_y_min <= 13'd2000;
                    seg_y_max <= 13'd0;
                    gap_cnt   <= 5'd0;
                    box_idx   <= 3'd0;
                    nx_min0 <= 13'd0; nx_max0 <= 13'd0; ny_min0 <= 13'd0; ny_max0 <= 13'd0;
                    nx_min1 <= 13'd0; nx_max1 <= 13'd0; ny_min1 <= 13'd0; ny_max1 <= 13'd0;
                    nx_min2 <= 13'd0; nx_max2 <= 13'd0; ny_min2 <= 13'd0; ny_max2 <= 13'd0;
                    nx_min3 <= 13'd0; nx_max3 <= 13'd0; ny_min3 <= 13'd0; ny_max3 <= 13'd0;
                    nx_min4 <= 13'd0; nx_max4 <= 13'd0; ny_min4 <= 13'd0; ny_max4 <= 13'd0;
                end
                cur_grid0 <= 64'd0; cur_grid1 <= 64'd0; cur_grid2 <= 64'd0; cur_grid3 <= 64'd0; cur_grid4 <= 64'd0;
                if (iPAUSE) begin
                    col_ink      <= 600'd0;
                    prev_row_ink <= 600'd0;
                    for (i = 0; i < 38; i = i + 1) begin
                        strip_y_min[i] <= 13'd2000;
                        strip_y_max[i] <= 13'd0;
                    end
                end
            end

            // 4. Scan 600 Columns to Extract Up to 5 Complete Character Boxes
            else if (state == 2'd1) begin
                if (scan_x < 10'd600) begin
                    if (col_ink[scan_x]) begin
                        if (!in_char) begin
                            in_char   <= 1'b1;
                            seg_start <= scan_x;
                            seg_end   <= scan_x;
                            gap_cnt   <= 5'd0;
                            if (strip_y_max[scan_x[9:4]] >= strip_y_min[scan_x[9:4]] + 13'd4) begin
                                seg_y_min <= strip_y_min[scan_x[9:4]];
                                seg_y_max <= strip_y_max[scan_x[9:4]];
                            end else begin
                                seg_y_min <= 13'd2000;
                                seg_y_max <= 13'd0;
                            end
                        end else begin
                            seg_end <= scan_x;
                            gap_cnt <= 5'd0;
                            if (strip_y_max[scan_x[9:4]] >= strip_y_min[scan_x[9:4]] + 13'd4) begin
                                if (strip_y_min[scan_x[9:4]] < seg_y_min) seg_y_min <= strip_y_min[scan_x[9:4]];
                                if (strip_y_max[scan_x[9:4]] > seg_y_max) seg_y_max <= strip_y_max[scan_x[9:4]];
                            end
                        end
                    end else if (in_char) begin
                        if (gap_cnt < GAP_TOL) begin
                            gap_cnt <= gap_cnt + 5'd1;
                        end else begin
                            in_char <= 1'b0;
                            if (seg_valid && box_idx < 3'd5) begin
                                case (box_idx)
                                    3'd0: begin nx_min0 <= SAFE_X_MIN + seg_start; nx_max0 <= SAFE_X_MIN + seg_end; ny_min0 <= seg_y_min; ny_max0 <= seg_y_max; end
                                    3'd1: begin nx_min1 <= SAFE_X_MIN + seg_start; nx_max1 <= SAFE_X_MIN + seg_end; ny_min1 <= seg_y_min; ny_max1 <= seg_y_max; end
                                    3'd2: begin nx_min2 <= SAFE_X_MIN + seg_start; nx_max2 <= SAFE_X_MIN + seg_end; ny_min2 <= seg_y_min; ny_max2 <= seg_y_max; end
                                    3'd3: begin nx_min3 <= SAFE_X_MIN + seg_start; nx_max3 <= SAFE_X_MIN + seg_end; ny_min3 <= seg_y_min; ny_max3 <= seg_y_max; end
                                    3'd4: begin nx_min4 <= SAFE_X_MIN + seg_start; nx_max4 <= SAFE_X_MIN + seg_end; ny_min4 <= seg_y_min; ny_max4 <= seg_y_max; end
                                endcase
                                box_idx <= box_idx + 3'd1;
                            end
                        end
                    end
                    scan_x <= scan_x + 10'd1;
                end else begin
                    if (in_char && seg_valid && box_idx < 3'd5) begin
                        case (box_idx)
                            3'd0: begin nx_min0 <= SAFE_X_MIN + seg_start; nx_max0 <= SAFE_X_MIN + seg_end; ny_min0 <= seg_y_min; ny_max0 <= seg_y_max; end
                            3'd1: begin nx_min1 <= SAFE_X_MIN + seg_start; nx_max1 <= SAFE_X_MIN + seg_end; ny_min1 <= seg_y_min; ny_max1 <= seg_y_max; end
                            3'd2: begin nx_min2 <= SAFE_X_MIN + seg_start; nx_max2 <= SAFE_X_MIN + seg_end; ny_min2 <= seg_y_min; ny_max2 <= seg_y_max; end
                            3'd3: begin nx_min3 <= SAFE_X_MIN + seg_start; nx_max3 <= SAFE_X_MIN + seg_end; ny_min3 <= seg_y_min; ny_max3 <= seg_y_max; end
                            3'd4: begin nx_min4 <= SAFE_X_MIN + seg_start; nx_max4 <= SAFE_X_MIN + seg_end; ny_min4 <= seg_y_min; ny_max4 <= seg_y_max; end
                        endcase
                        box_idx <= box_idx + 3'd1;
                    end
                    state <= 2'd2;
                end
            end

            // 5. Latch All Validated Boxes with Anti-Jitter Hysteresis & Reset Strip Trackers
            else if (state == 2'd2) begin
                X_MIN0 <= (nx_max0 == 13'd0) ? 13'd0 : smooth_coord(X_MIN0, nx_min0);
                X_MAX0 <= (nx_max0 == 13'd0) ? 13'd0 : smooth_coord(X_MAX0, nx_max0);
                Y_MIN0 <= (nx_max0 == 13'd0) ? 13'd0 : smooth_coord(Y_MIN0, ny_min0);
                Y_MAX0 <= (nx_max0 == 13'd0) ? 13'd0 : smooth_coord(Y_MAX0, ny_max0);

                X_MIN1 <= (nx_max1 == 13'd0) ? 13'd0 : smooth_coord(X_MIN1, nx_min1);
                X_MAX1 <= (nx_max1 == 13'd0) ? 13'd0 : smooth_coord(X_MAX1, nx_max1);
                Y_MIN1 <= (nx_max1 == 13'd0) ? 13'd0 : smooth_coord(Y_MIN1, ny_min1);
                Y_MAX1 <= (nx_max1 == 13'd0) ? 13'd0 : smooth_coord(Y_MAX1, ny_max1);

                X_MIN2 <= (nx_max2 == 13'd0) ? 13'd0 : smooth_coord(X_MIN2, nx_min2);
                X_MAX2 <= (nx_max2 == 13'd0) ? 13'd0 : smooth_coord(X_MAX2, nx_max2);
                Y_MIN2 <= (nx_max2 == 13'd0) ? 13'd0 : smooth_coord(Y_MIN2, ny_min2);
                Y_MAX2 <= (nx_max2 == 13'd0) ? 13'd0 : smooth_coord(Y_MAX2, ny_max2);

                X_MIN3 <= (nx_max3 == 13'd0) ? 13'd0 : smooth_coord(X_MIN3, nx_min3);
                X_MAX3 <= (nx_max3 == 13'd0) ? 13'd0 : smooth_coord(X_MAX3, nx_max3);
                Y_MIN3 <= (nx_max3 == 13'd0) ? 13'd0 : smooth_coord(Y_MIN3, ny_min3);
                Y_MAX3 <= (nx_max3 == 13'd0) ? 13'd0 : smooth_coord(Y_MAX3, ny_max3);

                X_MIN4 <= (nx_max4 == 13'd0) ? 13'd0 : smooth_coord(X_MIN4, nx_min4);
                X_MAX4 <= (nx_max4 == 13'd0) ? 13'd0 : smooth_coord(X_MAX4, nx_max4);
                Y_MIN4 <= (nx_max4 == 13'd0) ? 13'd0 : smooth_coord(Y_MIN4, ny_min4);
                Y_MAX4 <= (nx_max4 == 13'd0) ? 13'd0 : smooth_coord(Y_MAX4, ny_max4);

                oCHAR_COUNT <= {1'b0, box_idx};

                col_ink      <= 600'd0;
                prev_row_ink <= 600'd0;
                for (i = 0; i < 38; i = i + 1) begin
                    strip_y_min[i] <= 13'd2000;
                    strip_y_max[i] <= 13'd0;
                end
                state <= 2'd0;
            end
        end
    end

    // =========================================================================
    // VGA ON-SCREEN DISPLAY (OSD) FONT ROM & 8x8 DEBUG HUD GENERATOR
    // =========================================================================
    function [4:0] get_font_row(input [7:0] ch, input [2:0] r);
        begin
            case (ch)
                "0": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10011; 3:get_font_row=5'b10101; 4:get_font_row=5'b11001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "1": case(r) 0:get_font_row=5'b00100; 1:get_font_row=5'b01100; 2:get_font_row=5'b00100; 3:get_font_row=5'b00100; 4:get_font_row=5'b00100; 5:get_font_row=5'b00100; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "2": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b00001; 3:get_font_row=5'b00110; 4:get_font_row=5'b01000; 5:get_font_row=5'b10000; 6:get_font_row=5'b11111; default:get_font_row=5'b0; endcase
                "3": case(r) 0:get_font_row=5'b11110; 1:get_font_row=5'b00001; 2:get_font_row=5'b00001; 3:get_font_row=5'b01110; 4:get_font_row=5'b00001; 5:get_font_row=5'b00001; 6:get_font_row=5'b11110; default:get_font_row=5'b0; endcase
                "4": case(r) 0:get_font_row=5'b10010; 1:get_font_row=5'b10010; 2:get_font_row=5'b10010; 3:get_font_row=5'b11111; 4:get_font_row=5'b00010; 5:get_font_row=5'b00010; 6:get_font_row=5'b00010; default:get_font_row=5'b0; endcase
                "5": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b10000; 2:get_font_row=5'b11110; 3:get_font_row=5'b00001; 4:get_font_row=5'b00001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "6": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b11110; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "7": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b00001; 2:get_font_row=5'b00010; 3:get_font_row=5'b00100; 4:get_font_row=5'b01000; 5:get_font_row=5'b01000; 6:get_font_row=5'b01000; default:get_font_row=5'b0; endcase
                "8": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b01110; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "9": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b01111; 4:get_font_row=5'b00001; 5:get_font_row=5'b00001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "A": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b11111; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "B": case(r) 0:get_font_row=5'b11110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b11110; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b11110; default:get_font_row=5'b0; endcase
                "C": case(r) 0:get_font_row=5'b01111; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b10000; 4:get_font_row=5'b10000; 5:get_font_row=5'b10000; 6:get_font_row=5'b01111; default:get_font_row=5'b0; endcase
                "D": case(r) 0:get_font_row=5'b11110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10001; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b11110; default:get_font_row=5'b0; endcase
                "E": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b11110; 4:get_font_row=5'b10000; 5:get_font_row=5'b10000; 6:get_font_row=5'b11111; default:get_font_row=5'b0; endcase
                "F": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b11110; 4:get_font_row=5'b10000; 5:get_font_row=5'b10000; 6:get_font_row=5'b10000; default:get_font_row=5'b0; endcase
                "G": case(r) 0:get_font_row=5'b01111; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b10111; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01111; default:get_font_row=5'b0; endcase
                "H": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b11111; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "I": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b00100; 2:get_font_row=5'b00100; 3:get_font_row=5'b00100; 4:get_font_row=5'b00100; 5:get_font_row=5'b00100; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "J": case(r) 0:get_font_row=5'b00111; 1:get_font_row=5'b00010; 2:get_font_row=5'b00010; 3:get_font_row=5'b00010; 4:get_font_row=5'b10010; 5:get_font_row=5'b10010; 6:get_font_row=5'b01100; default:get_font_row=5'b0; endcase
                "K": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10010; 2:get_font_row=5'b10100; 3:get_font_row=5'b11000; 4:get_font_row=5'b10100; 5:get_font_row=5'b10010; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "L": case(r) 0:get_font_row=5'b10000; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b10000; 4:get_font_row=5'b10000; 5:get_font_row=5'b10000; 6:get_font_row=5'b11111; default:get_font_row=5'b0; endcase
                "M": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b11011; 2:get_font_row=5'b10101; 3:get_font_row=5'b10101; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "N": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b11001; 2:get_font_row=5'b10101; 3:get_font_row=5'b10011; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "O": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10001; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "P": case(r) 0:get_font_row=5'b11110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b11110; 4:get_font_row=5'b10000; 5:get_font_row=5'b10000; 6:get_font_row=5'b10000; default:get_font_row=5'b0; endcase
                "Q": case(r) 0:get_font_row=5'b01110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10001; 4:get_font_row=5'b10101; 5:get_font_row=5'b10010; 6:get_font_row=5'b01101; default:get_font_row=5'b0; endcase
                "R": case(r) 0:get_font_row=5'b11110; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b11110; 4:get_font_row=5'b10100; 5:get_font_row=5'b10010; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "S": case(r) 0:get_font_row=5'b01111; 1:get_font_row=5'b10000; 2:get_font_row=5'b10000; 3:get_font_row=5'b01110; 4:get_font_row=5'b00001; 5:get_font_row=5'b00001; 6:get_font_row=5'b11110; default:get_font_row=5'b0; endcase
                "T": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b00100; 2:get_font_row=5'b00100; 3:get_font_row=5'b00100; 4:get_font_row=5'b00100; 5:get_font_row=5'b00100; 6:get_font_row=5'b00100; default:get_font_row=5'b0; endcase
                "U": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10001; 4:get_font_row=5'b10001; 5:get_font_row=5'b10001; 6:get_font_row=5'b01110; default:get_font_row=5'b0; endcase
                "V": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10001; 4:get_font_row=5'b01010; 5:get_font_row=5'b01010; 6:get_font_row=5'b00100; default:get_font_row=5'b0; endcase
                "W": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b10001; 3:get_font_row=5'b10101; 4:get_font_row=5'b10101; 5:get_font_row=5'b11011; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "X": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b01010; 3:get_font_row=5'b00100; 4:get_font_row=5'b01010; 5:get_font_row=5'b10001; 6:get_font_row=5'b10001; default:get_font_row=5'b0; endcase
                "Y": case(r) 0:get_font_row=5'b10001; 1:get_font_row=5'b10001; 2:get_font_row=5'b01010; 3:get_font_row=5'b00100; 4:get_font_row=5'b00100; 5:get_font_row=5'b00100; 6:get_font_row=5'b00100; default:get_font_row=5'b0; endcase
                "Z": case(r) 0:get_font_row=5'b11111; 1:get_font_row=5'b00001; 2:get_font_row=5'b00010; 3:get_font_row=5'b00100; 4:get_font_row=5'b01000; 5:get_font_row=5'b10000; 6:get_font_row=5'b11111; default:get_font_row=5'b0; endcase
                "-": case(r) 3:get_font_row=5'b11111; default:get_font_row=5'b00000; endcase
                default: get_font_row = 5'b00000;
            endcase
        end
    endfunction

    // 1. Top-Left Corner HUD Box (X: 228..516, Y: 30..128)
    assign oOSD_BG = (x_coord >= 13'd228 && x_coord <= 13'd516 &&
                      y_coord >= 13'd30  && y_coord <= 13'd128);

    wire hud_border = oOSD_BG && (x_coord == 13'd228 || x_coord == 13'd516 ||
                                  y_coord == 13'd30  || y_coord == 13'd128);

    // 2. Giant 8x Font Characters in HUD Row 1 (40x56 pixels each at Y: 34..89)
    wire in_hud_char_y = (y_coord >= 13'd34 && y_coord < 13'd90);
    wire [12:0] hud_dy = y_coord - 13'd34;
    wire [2:0]  hud_r  = hud_dy[5:3];

    wire s0_x = (x_coord >= 13'd240 && x_coord < 13'd280);
    wire s1_x = (x_coord >= 13'd296 && x_coord < 13'd336);
    wire s2_x = (x_coord >= 13'd352 && x_coord < 13'd392);
    wire s3_x = (x_coord >= 13'd408 && x_coord < 13'd448);
    wire s4_x = (x_coord >= 13'd464 && x_coord < 13'd504);

    wire [12:0] hud_dx = s0_x ? (x_coord - 13'd240) :
                         s1_x ? (x_coord - 13'd296) :
                         s2_x ? (x_coord - 13'd352) :
                         s3_x ? (x_coord - 13'd408) :
                                (x_coord - 13'd464);

    wire [7:0] hud_ch  = s0_x ? ((oCHAR_COUNT >= 4'd1) ? oCHAR0 : 8'h2D) :
                         s1_x ? ((oCHAR_COUNT >= 4'd2) ? oCHAR1 : 8'h2D) :
                         s2_x ? ((oCHAR_COUNT >= 4'd3) ? oCHAR2 : 8'h2D) :
                         s3_x ? ((oCHAR_COUNT >= 4'd4) ? oCHAR3 : 8'h2D) :
                                ((oCHAR_COUNT >= 4'd5) ? oCHAR4 : 8'h2D);

    wire [4:0] hud_font_bits = get_font_row(hud_ch, hud_r);
    wire hud_char_pix = in_hud_char_y && (s0_x | s1_x | s2_x | s3_x | s4_x) &&
                        hud_font_bits[3'd4 - hud_dx[5:3]];

    // 3. 8x8 Sampled Debug Grids in HUD Row 2 (32x32 pixels each at Y: 92..123, 4x4 pixels per cell)
    wire in_hud_grid_y = (y_coord >= 13'd92 && y_coord < 13'd124);
    wire [12:0] grid_dy = y_coord - 13'd92;

    wire g0_x = (x_coord >= 13'd244 && x_coord < 13'd276);
    wire g1_x = (x_coord >= 13'd300 && x_coord < 13'd332);
    wire g2_x = (x_coord >= 13'd356 && x_coord < 13'd388);
    wire g3_x = (x_coord >= 13'd412 && x_coord < 13'd444);
    wire g4_x = (x_coord >= 13'd468 && x_coord < 13'd500);

    wire [12:0] grid_dx = g0_x ? (x_coord - 13'd244) :
                          g1_x ? (x_coord - 13'd300) :
                          g2_x ? (x_coord - 13'd356) :
                          g3_x ? (x_coord - 13'd412) :
                                 (x_coord - 13'd468);

    wire [2:0] g_inv_r   = 3'd7 - grid_dy[4:2];
    wire [2:0] g_inv_c   = 3'd7 - grid_dx[4:2];
    wire [5:0] g_bit_idx = {g_inv_r, g_inv_c};

    wire [63:0] active_grid = g0_x ? sampled_grid0 :
                              g1_x ? sampled_grid1 :
                              g2_x ? sampled_grid2 :
                              g3_x ? sampled_grid3 : sampled_grid4;

    wire in_grid_cell = in_hud_grid_y && (g0_x | g1_x | g2_x | g3_x | g4_x) &&
                        (grid_dx[1:0] != 2'd3) && (grid_dy[1:0] != 2'd3);

    assign oOSD_GRID_ON  = in_grid_cell && active_grid[g_bit_idx];
    assign oOSD_GRID_OFF = in_grid_cell && !active_grid[g_bit_idx];

    // 4. Floating 4x Character Labels Directly Above Each Green Box (20x28 pixels)
    wire lbl0 = (X_MAX0 > X_MIN0) && (Y_MIN0 > 13'd34) &&
                (x_coord >= X_MIN0 && x_coord < X_MIN0 + 13'd20) &&
                (y_coord + 13'd32 >= Y_MIN0 && y_coord + 13'd4 < Y_MIN0);
    wire [12:0] lbl0_dx = x_coord - X_MIN0;
    wire [12:0] lbl0_dy = y_coord - (Y_MIN0 - 13'd32);
    wire [4:0]  lbl0_bits = get_font_row(oCHAR0, lbl0_dy[4:2]);
    wire lbl0_pix = lbl0 && lbl0_bits[3'd4 - lbl0_dx[4:2]];

    wire lbl1 = (X_MAX1 > X_MIN1) && (Y_MIN1 > 13'd34) &&
                (x_coord >= X_MIN1 && x_coord < X_MIN1 + 13'd20) &&
                (y_coord + 13'd32 >= Y_MIN1 && y_coord + 13'd4 < Y_MIN1);
    wire [12:0] lbl1_dx = x_coord - X_MIN1;
    wire [12:0] lbl1_dy = y_coord - (Y_MIN1 - 13'd32);
    wire [4:0]  lbl1_bits = get_font_row(oCHAR1, lbl1_dy[4:2]);
    wire lbl1_pix = lbl1 && lbl1_bits[3'd4 - lbl1_dx[4:2]];

    wire lbl2 = (X_MAX2 > X_MIN2) && (Y_MIN2 > 13'd34) &&
                (x_coord >= X_MIN2 && x_coord < X_MIN2 + 13'd20) &&
                (y_coord + 13'd32 >= Y_MIN2 && y_coord + 13'd4 < Y_MIN2);
    wire [12:0] lbl2_dx = x_coord - X_MIN2;
    wire [12:0] lbl2_dy = y_coord - (Y_MIN2 - 13'd32);
    wire [4:0]  lbl2_bits = get_font_row(oCHAR2, lbl2_dy[4:2]);
    wire lbl2_pix = lbl2 && lbl2_bits[3'd4 - lbl2_dx[4:2]];

    wire lbl3 = (X_MAX3 > X_MIN3) && (Y_MIN3 > 13'd34) &&
                (x_coord >= X_MIN3 && x_coord < X_MIN3 + 13'd20) &&
                (y_coord + 13'd32 >= Y_MIN3 && y_coord + 13'd4 < Y_MIN3);
    wire [12:0] lbl3_dx = x_coord - X_MIN3;
    wire [12:0] lbl3_dy = y_coord - (Y_MIN3 - 13'd32);
    wire [4:0]  lbl3_bits = get_font_row(oCHAR3, lbl3_dy[4:2]);
    wire lbl3_pix = lbl3 && lbl3_bits[3'd4 - lbl3_dx[4:2]];

    wire lbl4 = (X_MAX4 > X_MIN4) && (Y_MIN4 > 13'd34) &&
                (x_coord >= X_MIN4 && x_coord < X_MIN4 + 13'd20) &&
                (y_coord + 13'd32 >= Y_MIN4 && y_coord + 13'd4 < Y_MIN4);
    wire [12:0] lbl4_dx = x_coord - X_MIN4;
    wire [12:0] lbl4_dy = y_coord - (Y_MIN4 - 13'd32);
    wire [4:0]  lbl4_bits = get_font_row(oCHAR4, lbl4_dy[4:2]);
    wire lbl4_pix = lbl4 && lbl4_bits[3'd4 - lbl4_dx[4:2]];

    assign oOSD_TEXT = hud_border | hud_char_pix | lbl0_pix | lbl1_pix | lbl2_pix | lbl3_pix | lbl4_pix;

endmodule