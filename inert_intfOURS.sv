module inert_intf(clk, rst_n, incline, vld, INT, SS_n, SCLK, MOSI, MISO);
	input clk, rst_n, INT, MISO;
	output SS_n, MOSI, SCLK, vld;
	output [12:0] incline;
	logic INT_ff1, INT_ff2, init_done;
	logic capture, done, RR_sig_en, capture_f;
	logic [15:0] cnt;
	logic [2:0] RR_sig_init;
	logic [2:0] RR_sig;
	logic [15:0] rd_data, cmd; 
	logic [15:0] addr_data_init, addr_data;
	logic [7:0] rollL, rollH, yawL, yawH, AYL, AYH, AZL, AZH;
	logic [15:0] roll_rt, yaw_rt, AY, AZ;
	logic wrt, vld, truth;
	wire rollL_en, rollH_en, yawL_en, yawH_en, AYL_en, AYH_en, AZL_en, AZH_en;
	SPI_mstr i_spy(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .wrt(wrt), .cmd(cmd), .MISO(MISO), .rd_data(rd_data), .done(done));
    inertial_integrator calculus(.clk(clk),.rst_n(rst_n),.vld(vld),.roll_rt(roll_rt),.yaw_rt(yaw_rt),.AY(AY),.AZ(AZ),.incline(incline));
	
	always @(posedge clk) 
		capture_f <= capture;
	 //logic ENABLE;
	// The data registers :)
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			rollL <= 8'h00;
		else if(rollL_en) begin 
			rollL <= rd_data[7:0];
			
		end
	
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			rollH <= 8'h00;
		else if(rollH_en) begin
			rollH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			yawL <= 8'h00;
		else if(yawL_en) begin
			yawL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			yawH <= 8'h00;
		else if(yawH_en) begin
			yawH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AYL <= 8'h00;
		else if(AYL_en) begin
			AYL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AYH <= 8'h00;
		else if(AYH_en) begin
			AYH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AZL <= 8'h00;
		else if(AZL_en) begin
			AZL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AZH <= 8'h00;
		else if(AZH_en) begin
			AZH <= rd_data[7:0];
			
		end
	end
	 // Assign the enable signals :)
	 
	 assign rollL_en = capture_f ? ((RR_sig == 4'h0) ? 1'b1 : 1'b0) : 1'b0;
	 assign rollH_en = capture_f ? ((RR_sig == 4'h1) ? 1'b1 : 1'b0) : 1'b0;
	 assign yawL_en = capture_f ? ((RR_sig == 4'h2) ? 1'b1 : 1'b0) : 1'b0;
	 assign yawH_en = capture_f ? ((RR_sig == 4'h3) ? 1'b1 : 1'b0) : 1'b0;
	 assign AYL_en = capture_f ? ((RR_sig == 4'h4) ? 1'b1 : 1'b0) : 1'b0;
	 assign AYH_en = capture_f ? ((RR_sig == 4'h5) ? 1'b1 : 1'b0) : 1'b0;
	 assign AZL_en = capture_f ? ((RR_sig == 4'h6) ? 1'b1 : 1'b0) : 1'b0;
	 assign AZH_en = capture_f ? ((RR_sig == 4'h7) ? 1'b1 : 1'b0) : 1'b0;

	

	 // Assign the inputs to inertial_integrator
	 
	 assign roll_rt = {rollH, rollL};
	 assign yaw_rt = {yawH, yawL};
	 assign AY = {AYH, AYL};
	 assign AZ = {AZH, AZL};
	 
	 

	 
	 // Count
	 always @(posedge clk, negedge rst_n) begin
	    if(!rst_n)
		   cnt <= 16'b0;
		else 
		    cnt <= cnt + 1;
	end
	// RR initial counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) begin
			RR_sig_init <= 4'b0;
			
		end
		else if(done) 
			RR_sig_init <= RR_sig_init + 1;
		
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			init_done <= 1'h0;
		else if(RR_sig_init == 4'h3) begin
			init_done <= 1'h1;
		end
	
	end
	
	assign truth = (rollL_en || rollH_en || yawL_en || yawH_en || AYL_en || AYH_en || AZL_en || AZH_en);

	always@(posedge clk, negedge rst_n) begin
		if(!rst_n)
			RR_sig_en <= 0;
		else if(RR_sig_en)
			RR_sig_en <= 0;
		else
			RR_sig_en <= truth;
	end

	assign addr_data_init = RR_sig_init[1] ? (RR_sig_init[0] ? 16'h1460: 16'h1150) : (RR_sig_init[0] ? 16'h1053 : 16'h0D02);
	
	// RR General Counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) 
			RR_sig <= 3'b0;
		else if(RR_sig_en && init_done)
			RR_sig <= RR_sig + 1;
	end
	
	assign addr_data = RR_sig[2] ? (RR_sig[1] ? (RR_sig[0] ? 16'hAD00 : 16'hAC00) : (RR_sig[0] ? 16'hAB00 : 16'hAA00)) : RR_sig[1] ? (RR_sig[0] ? 16'hA700 : 16'hA600) : RR_sig[0] ? 16'hA500 : 16'hA400; 
	// Double flop INT
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) begin
			INT_ff1 <= 1'b0;
			INT_ff2 <= 1'b0;
		end
		else begin
			INT_ff1 <= INT;
			INT_ff2 <= INT_ff1;
		end
	end
	assign cmd = init_done ? addr_data : addr_data_init;
	
	typedef enum logic [2:0] {RESET, INITIALIZE, WAIT_INITIALIZE, START_READ,WAIT_INIT, SET_VALID, WAIT_READ} state_t;
	state_t state, nxt_state;
	
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			state <= RESET;
		else
			state <= nxt_state;
	end
	
	
	always_comb begin 
		wrt = 0;
		capture = 0;
		nxt_state = state;
		vld = 0;
		case (state) 
			RESET: 
				if(&cnt)
					nxt_state = INITIALIZE;
			
			INITIALIZE: begin
				//if (&cnt)
				//	nxt_state = WAIT_INIT;
				 if(!init_done) begin
					wrt = 1;
					nxt_state = WAIT_INITIALIZE;
				end
				else 
						nxt_state = WAIT_INIT;
				
			end
			WAIT_INITIALIZE: begin
				if (done)
					nxt_state = INITIALIZE;
			
			end
			WAIT_INIT: begin
				if(INT_ff2) begin
					nxt_state = START_READ;
				end
			end
			START_READ: begin
					wrt = 1;
					nxt_state = WAIT_READ;
				
			end
			WAIT_READ: begin
				if(done) begin
					capture = 1;
					if(RR_sig == 8'h7) 
						nxt_state = SET_VALID;
					else
						nxt_state = START_READ;
				end
			end
			SET_VALID: begin
				vld = 1;
				nxt_state = WAIT_INIT;
			
			end
			
			default: nxt_state = RESET;
			
			endcase
		end
endmodule
