module SPI_mstr(clk, rst_n, SS_n, SCLK, MOSI, wrt, cmd, MISO, rd_data, done);
	input clk, wrt, rst_n, MISO;
	input [15:0] cmd;
	output reg SS_n, SCLK, MOSI, done;
	output logic [15:0] rd_data;
	reg [5:0] sclk_div;
	reg MISO_smpl, shft, smpl, ld_SCLK, init, set_done;
	wire done15;
	reg [15:0] shft_reg;
	reg [15:0] shft_reg_shifted;
	reg [3:0] bit_cntr;
  wire sclk_rise_eminent, sclk_fall_eminent;
	//SCLK Thing
	always @(posedge clk) begin
		if (ld_SCLK)
			sclk_div <= 6'b110000;
		else 
			sclk_div <= sclk_div + 1;
	end
	assign SCLK = sclk_div[5];
	assign sclk_rise_eminent = ~sclk_div[5] & (&sclk_div[4:0]);
	assign sclk_fall_eminent = &sclk_div;
	//MISO Shift Reg
	
	always @(posedge clk) begin
		if (smpl)
			MISO_smpl <= MISO;
	end
	
	assign shft_reg_shifted = {shft_reg[14:0], MISO_smpl};
	//MISO Shift Reg Round 2
	always @(posedge clk) begin
		if (init)
			shft_reg <= cmd;
		else if(shft)
			shft_reg <= shft_reg_shifted;
	end
	assign MOSI = shft_reg[15];
	//bit counter
	always @(posedge clk) begin
		if (init)
			bit_cntr <= 4'b0;
		else begin
			if (shft)
				bit_cntr <= bit_cntr + 1;
			else 
				bit_cntr <= bit_cntr;
		end
	end	
	assign done15 = &bit_cntr; // Changed from nor to and
	
	///STATE Machine
	typedef enum reg[2:0] {IDLE, FRONTPORCH, SMPL, SHFT, BACKPORCH} state_t;
	state_t state, nxt_state;
	
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) 
			state <= IDLE;
		else
			state <= nxt_state;
	end
	
	always_comb begin
		//DEFAULT Stuff
		ld_SCLK = 0; // HERE changed from 0 to 1
		init = 0;
       shft = 0;
      smpl = 0;
		set_done = 0;
		nxt_state = state; 
		case(state)
			IDLE: if(wrt) begin
					init = 1;
					ld_SCLK = 1;
					nxt_state = FRONTPORCH;
			end else ld_SCLK = 1;
			FRONTPORCH: begin
				ld_SCLK = 0; // HERE added this 
                if (sclk_fall_eminent) 
                  nxt_state = SMPL;
		end
			SMPL: begin	
                shft = sclk_fall_eminent;
                smpl = sclk_rise_eminent;	
				if(done15) begin
					nxt_state = BACKPORCH;
				end
			end
			BACKPORCH: begin 
                shft = sclk_fall_eminent;
                smpl = sclk_rise_eminent;	
               if (sclk_fall_eminent) begin
				  ld_SCLK = 1;
				  set_done = 1;
				  nxt_state = IDLE;
				end
			end
			default: 
				nxt_state = IDLE;
			endcase
		end
		assign rd_data = shft_reg;
		//OUTPUT FLOP
		always @(posedge clk, negedge rst_n) begin
			if (!rst_n )
				done <= 1'b0;
			else if(init || !set_done)
				done <= 1'b0;
			else if(set_done) begin
				done <= 1'b1;
				end
			end

		always @(posedge clk, negedge rst_n) begin
			if (!rst_n )
				SS_n <= 1'b1;
			else if(init )
				SS_n  <= 1'b0;
			else if(set_done) begin
				SS_n <= 1'b1;
				end
			end
	
endmodule
	
	
	
