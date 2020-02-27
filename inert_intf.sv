module inert_intf(clk, rst_n, incline, vld, INT, SS_n, SCLK, MOSI, MISO);
	input clk, rst_n, INT, MISO;
	output SS_n, MOSI, SCLK; 
	output [12:0] incline;
	logic INT_ff1, INT_ff2, init_done;
	logic done;
        logic done_init;	
	logic [15:0] cnt;
	logic [2:0] RR_sig_init;
	logic [2:0] RR_sig;
	logic RR_sig_inc, RR_init_sig_inc, reg_en;
	logic [15:0] rd_data, cmd; 
	logic [15:0] addr_data_init, addr_data;
	//logic [7:0] addr_data_byte;
	logic [7:0] rollL, rollH, yawL, yawH, AYL, AYH, AZL, AZH;
	logic [15:0] roll_rt, yaw_rt, AY, AZ;
	logic wrt;
	output reg vld;
	SPI_mstr mine(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .wrt(wrt), .cmd(cmd), .MISO(MISO), .rd_data(rd_data), .done(done));
    inertial_integrator me(.clk(clk),.rst_n(rst_n),.vld(vld),.roll_rt(roll_rt),.yaw_rt(yaw_rt),.AY(AY),.AZ(AZ),.incline(incline));
	
	 //logic ENABLE;
	// The data registers :)
	
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
	
	 // Count
	 always @(posedge clk, negedge rst_n) begin
	    if(!rst_n)
		   cnt <= 16'b0;
		else 
		    cnt <= cnt + 1;
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			rollL <= 8'h00;
		else if(RR_sig == 3'h0 && reg_en) begin 
			rollL <= rd_data[7:0];
			
		end
	
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			rollH <= 8'h00;
		else if(RR_sig == 3'h1 && reg_en) begin
			rollH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			yawL <= 8'h00;
		else if(RR_sig == 3'h2 && reg_en) begin
			yawL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			yawH <= 8'h00;
		else if(RR_sig == 3'h3 && reg_en) begin
			yawH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AYL <= 8'h00;
		else if(RR_sig == 3'h4 && reg_en) begin
			AYL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AYH <= 8'h00;
		else if(RR_sig == 3'h5 && reg_en) begin
			AYH <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AZL <= 8'h00;
		else if(RR_sig == 3'h6 && reg_en) begin
			AZL <= rd_data[7:0];
			
		end
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			AZH <= 8'h00;
		else if(RR_sig == 3'h7 && reg_en) begin
			AZH <= rd_data[7:0];
			
		end
	end
	 
	 
	

	

	 // Assign the inputs to inertial_integrator
	 
	 assign roll_rt = {rollH, rollL};
	 assign yaw_rt = {yawH, yawL};
	 assign AY = {AYH, AYL};
	 assign AZ = {AZH, AZL};
	 
	 

	 
	
	// RR initial counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) begin
			RR_sig_init <= 4'b0;
			//init_done <= 1'b0;
		end
		else if(done && !init_done) 
			RR_sig_init <= RR_sig_init + 1;
			//init_done <= 1'b1;
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			init_done <= 1'h0;
		else if(done_init) begin
			init_done <= 1'h1;
		end
	
	end
	//assign init_done = (RR_sig_init ==4 ) ? 1 : 0; 
	assign addr_data_init = RR_sig_init[1] ? (RR_sig_init[0] ? 16'h1460: 16'h1150) : (RR_sig_init[0] ? 16'h1053 : 16'h0D02);
	//assign RR_sig_en = (RR_sig_en || ~rst_n) ? 0 : (rollL_en || rollH_en || yawL_en || yawH_en || AYL_en || AYH_en || AZL_en || AZH_en) ? 1 : 0;
	// RR General Counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) 
			RR_sig <= 3'b0;
		else if(RR_sig_inc)
			RR_sig <= RR_sig + 1;
	end
	
	assign addr_data = RR_sig[2] ? (RR_sig[1] ? (RR_sig[0] ? 16'hAD00 : 16'hAC00) : (RR_sig[0] ? 16'hAB00 : 16'hAA00)) : RR_sig[1] ? (RR_sig[0] ? 16'hA700 : 16'hA600) : RR_sig[0] ? 16'hA500 : 16'hA400; 
	
	assign cmd = init_done ? addr_data : addr_data_init;
	// TODO : How to cycle through the different read signals
	
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
		//capture = 0;
		nxt_state = state;
		vld = 0;
		done_init= 0;
		RR_sig_inc = 0;
		RR_init_sig_inc = 0;
		reg_en = 0;
		case (state) 
			RESET: if(&cnt)
				nxt_state = INITIALIZE;
			
			INITIALIZE: begin
				wrt = 1;
				nxt_state = WAIT_INITIALIZE;
				
			end
			WAIT_INITIALIZE: begin
				if (done) begin
					if(RR_sig_init == 3'h3) begin
						done_init = 1;
						nxt_state = WAIT_INIT;
					end
					else begin
						RR_init_sig_inc = 1;
						nxt_state = INITIALIZE;
					end
				end
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
					reg_en = 1;
					RR_sig_inc = 1;
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
