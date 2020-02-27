module PWM(clk, rst_n, duty, PWM_sig);
	input clk, rst_n; //Clock and reset
	input [10:0] duty;
	output logic PWM_sig;
	logic [10:0] cnt; //unsigned counter
	
	//counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			cnt <= 11'b0;
		else
		    cnt <= cnt + 1'b1;
	end
	
	//compare
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			PWM_sig <= 1'b0;
		else if (cnt < duty)
			PWM_sig <= 1'b1;
		else if (cnt >= duty)
		     PWM_sig <= 1'b0;
	end
endmodule
	
	
	
		
		
