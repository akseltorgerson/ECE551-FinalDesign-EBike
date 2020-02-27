module rst_synch(RST_n, clk, rst_n);

	input RST_n, clk;
	output logic rst_n;
	logic intermediate;

	// first flippity floppity
	always @(negedge clk, negedge RST_n) begin
		if(~RST_n)
			intermediate <= 1'b0;
		else
			intermediate <= 1'b1;
	end
	
	// second flippity floppity
	always @(negedge clk, negedge RST_n) begin
		if(~RST_n)
			rst_n <= 1'b0;
		else
			rst_n <= intermediate;
	end
	
endmodule
			

