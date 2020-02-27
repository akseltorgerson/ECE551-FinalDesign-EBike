module mtr_drv(clk, rst_n, duty, selGrn, selYlw, selBlu, highGrn, lowGrn, highYlw, lowYlw, highBlu, lowBlu);

	input logic clk, rst_n;
	input logic [1:0] selBlu, selYlw, selGrn;
	input logic [10:0] duty;
	output logic highGrn, lowGrn, highYlw, lowYlw, highBlu, lowBlu;
	logic PWM_sig;
	logic o_mux_grn, o_mux_ylw, o_mux_blu;
	logic o2_mux_grn, o2_mux_ylw, o2_mux_blu;

	PWM PWM11(.clk(clk), .rst_n(rst_n), .duty(duty), .PWM_sig(PWM_sig));
	nonoverlap iGrn(.clk(clk), .rst_n(rst_n), .highIn(o_mux_grn), .lowIn(o2_mux_grn), .highOut(highGrn), .lowOut(lowGrn));
	nonoverlap iYlw(.clk(clk), .rst_n(rst_n), .highIn(o_mux_ylw), .lowIn(o2_mux_ylw), .highOut(highYlw), .lowOut(lowYlw));
	nonoverlap iBlu(.clk(clk), .rst_n(rst_n), .highIn(o_mux_blu), .lowIn(o2_mux_blu), .highOut(highBlu), .lowOut(lowBlu));
	
	assign o_mux_grn = (^selGrn) ? (selGrn[0] ? ~PWM_sig : PWM_sig) : 1'b0;
	assign o_mux_ylw = (^selYlw) ? (selYlw[0] ? ~PWM_sig : PWM_sig) : 1'b0;
	assign o_mux_blu = (^selBlu) ? (selBlu[0] ? ~PWM_sig : PWM_sig) : 1'b0;
	
	assign o2_mux_grn = (selGrn[0]) ? (PWM_sig) : (selGrn[1] ? ~PWM_sig : 1'b0);
	assign o2_mux_ylw = (selYlw[0]) ? (PWM_sig) : (selYlw[1] ? ~PWM_sig : 1'b0);
	assign o2_mux_blu = (selBlu[0]) ? (PWM_sig) : (selBlu[1] ? ~PWM_sig : 1'b0);
	
endmodule
