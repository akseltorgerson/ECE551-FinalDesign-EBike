module cadence_filt(clk, rst_n, cadence, cadence_filt);
	input logic clk, rst_n, cadence;
	output logic cadence_filt;
	logic q1,q2,q3; 
	logic cnt_in, stable, filt_in;
	logic [15:0] stbl_cnt;
	
	parameter FAST_SIM = 0;

	// Triple flop the input cadence
	always @(posedge clk) begin
		q1 <= cadence;
		q2 <= q1;
		q3 <= q2;
	end
	
	//cadence flop
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			stbl_cnt <= 1'b0;
		else if (q2 == q3)
			stbl_cnt <= stbl_cnt + 1;
		else
			stbl_cnt <= 15'b0;
	end


	
	/***********
	* FAST_SIM *
	***********/
	generate if (FAST_SIM)
		assign stable = &stbl_cnt[8:0];
	else
		assign stable = &stbl_cnt;
	endgenerate
	
	assign filt_in = stable ? q3 : cadence_filt; // Serves as the mux to choose only when the signal is stable

	always @(posedge clk) begin
		cadence_filt <= filt_in;
	end

endmodule


