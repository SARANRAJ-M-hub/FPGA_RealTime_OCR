module text_binarizer (
    input        iCLK,       // Connected to D5M_PIXLCLK
    input [11:0] iRed,       // Incoming 12-bit Red
    input [11:0] iGreen,     // Incoming 12-bit Green
    input [11:0] iBlue,      // Incoming 12-bit Blue
    input        iSW_active, // Slide switch to toggle the effect (e.g., SW)
    output [11:0] oRed,      // Filtering output Red
    output [11:0] oGreen,    // Filtering output Green
    output [11:0] oBlue      // Filtering output Blue
);

    reg [13:0] grayscale;
    reg        is_white;

    always @(posedge iCLK) begin
        // Standard Grayscale weighting calculation: (R*2 + G*4 + B*2) / 8 for fast shifts
        grayscale <= (iRed >> 2) + (iGreen >> 1) + (iBlue >> 2);
        
        // FIXED LOGIC: If dark pixel, make it black (0). If bright pixel, make it white (1).
        if (grayscale < 12'h400) 
            is_white <= 1'b0; // Text stroke (Black)
        else
            is_white <= 1'b1; // Background paper (White)
    end

    // Mux logic: If switch is ON, show Black & White text. If OFF, show normal color.
    assign oRed   = (iSW_active) ? (is_white ? 12'hFFF : 12'h000) : iRed;
    assign oGreen = (iSW_active) ? (is_white ? 12'hFFF : 12'h000) : iGreen;
    assign oBlue  = (iSW_active) ? (is_white ? 12'hFFF : 12'h000) : iBlue;

endmodule
