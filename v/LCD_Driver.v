module LCD_Driver (
    input            iCLK,
    input            iRST_N,
    input      [3:0] iDIGIT,   // Number of detected boxes (0..5)
    input      [7:0] iCHAR0,   // Recognized ASCII Character 0
    input      [7:0] iCHAR1,   // Recognized ASCII Character 1
    input      [7:0] iCHAR2,   // Recognized ASCII Character 2
    input      [7:0] iCHAR3,   // Recognized ASCII Character 3
    input      [7:0] iCHAR4,   // Recognized ASCII Character 4
    
    output           LCD_ON,
    output           LCD_BLON,
    output           LCD_RW,
    output reg       LCD_EN,
    output reg       LCD_RS,
    output reg [7:0] LCD_DATA
);

    assign LCD_ON   = 1'b1;
    assign LCD_BLON = 1'b1;
    assign LCD_RW   = 1'b0;

    reg [19:0] clk_div;
    reg [5:0]  step;

    always @(posedge iCLK or negedge iRST_N) begin
        if (!iRST_N) begin
            clk_div  <= 20'd0;
            step     <= 6'd0;
            LCD_EN   <= 1'b0;
            LCD_RS   <= 1'b0;
            LCD_DATA <= 8'h00;
        end else begin
            if (clk_div < 20'd100_000) begin // ~2ms per LCD step at 50MHz
                clk_div <= clk_div + 20'd1;
                if (clk_div == 20'd20_000)
                    LCD_EN <= 1'b1;
                else if (clk_div == 20'd80_000)
                    LCD_EN <= 1'b0;
            end else begin
                clk_div <= 20'd0;
                step    <= (step == 6'd28) ? 6'd4 : (step + 6'd1);

                case (step)
                    // LCD Hardware Initialization
                    6'd0:  begin LCD_RS <= 1'b0; LCD_DATA <= 8'h38; end // 8-bit, 2 lines
                    6'd1:  begin LCD_RS <= 1'b0; LCD_DATA <= 8'h0C; end // Display ON, Cursor OFF
                    6'd2:  begin LCD_RS <= 1'b0; LCD_DATA <= 8'h01; end // Clear Display
                    6'd3:  begin LCD_RS <= 1'b0; LCD_DATA <= 8'h06; end // Entry Mode

                    // Line 1: "DETECTED: X"
                    6'd4:  begin LCD_RS <= 1'b0; LCD_DATA <= 8'h80; end // Line 1 Start
                    6'd5:  begin LCD_RS <= 1'b1; LCD_DATA <= "D";   end
                    6'd6:  begin LCD_RS <= 1'b1; LCD_DATA <= "E";   end
                    6'd7:  begin LCD_RS <= 1'b1; LCD_DATA <= "T";   end
                    6'd8:  begin LCD_RS <= 1'b1; LCD_DATA <= "E";   end
                    6'd9:  begin LCD_RS <= 1'b1; LCD_DATA <= "C";   end
                    6'd10: begin LCD_RS <= 1'b1; LCD_DATA <= "T";   end
                    6'd11: begin LCD_RS <= 1'b1; LCD_DATA <= "E";   end
                    6'd12: begin LCD_RS <= 1'b1; LCD_DATA <= "D";   end
                    6'd13: begin LCD_RS <= 1'b1; LCD_DATA <= ":";   end
                    6'd14: begin LCD_RS <= 1'b1; LCD_DATA <= " ";   end
                    6'd15: begin LCD_RS <= 1'b1; LCD_DATA <= 8'h30 + {4'b0000, iDIGIT}; end

                    // Line 2: "TEXT: XXXXX"
                    6'd16: begin LCD_RS <= 1'b0; LCD_DATA <= 8'hC0; end // Line 2 Start
                    6'd17: begin LCD_RS <= 1'b1; LCD_DATA <= "T";   end
                    6'd18: begin LCD_RS <= 1'b1; LCD_DATA <= "E";   end
                    6'd19: begin LCD_RS <= 1'b1; LCD_DATA <= "X";   end
                    6'd20: begin LCD_RS <= 1'b1; LCD_DATA <= "T";   end
                    6'd21: begin LCD_RS <= 1'b1; LCD_DATA <= ":";   end
                    6'd22: begin LCD_RS <= 1'b1; LCD_DATA <= " ";   end
                    6'd23: begin LCD_RS <= 1'b1; LCD_DATA <= (iDIGIT >= 4'd1) ? iCHAR0 : 8'h20; end
                    6'd24: begin LCD_RS <= 1'b1; LCD_DATA <= (iDIGIT >= 4'd2) ? iCHAR1 : 8'h20; end
                    6'd25: begin LCD_RS <= 1'b1; LCD_DATA <= (iDIGIT >= 4'd3) ? iCHAR2 : 8'h20; end
                    6'd26: begin LCD_RS <= 1'b1; LCD_DATA <= (iDIGIT >= 4'd4) ? iCHAR3 : 8'h20; end
                    6'd27: begin LCD_RS <= 1'b1; LCD_DATA <= (iDIGIT >= 4'd5) ? iCHAR4 : 8'h20; end
                    6'd28: begin LCD_RS <= 1'b1; LCD_DATA <= " ";   end
                    default: begin LCD_RS <= 1'b0; LCD_DATA <= 8'h80; end
                endcase
            end
        end
    end
endmodule