module A2D_intf(clk, rst_n, MISO, batt, curr, brake,torque, SS_n, SCLK, MOSI);
	input clk, rst_n, MISO;
	output reg[11:0] batt, curr, brake, torque;
	output SS_n, SCLK, MOSI;
	
	logic [13:0] count;
	logic count_full;

	// flopped version
	logic truth;	

	logic test;
	logic wrt;
	logic [15:0] cmd;
	logic [15:0] rd_data;
	logic done;
	logic cnv_cmplt;
	logic [1:0] RR_cnt, RR_sig;
	logic [2:0] channel;
	wire torque_en, curr_en, batt_en, brake_en;
	SPI_mstr mstr(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .wrt(wrt), .cmd(cmd), .MISO(MISO),.rd_data(rd_data), .done(done));
	
	
	// 14 bit Counter 
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			count <= 0;
		else
		 count <= count + 1;
	end
	assign count_full = ~|count;
	// Go to the next RR_cnt when cnv_complete

	//RR Counter
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			RR_sig <= 2'b0;
		else if(test) begin
			RR_sig <= RR_sig + 1;
			//test <= 0;
		end
	end
	

	assign truth = torque_en || curr_en || brake_en || batt_en;

	always@(posedge clk, negedge rst_n) begin
		if(!rst_n)
			test <= 0;
		else if(test)
			test <= 0;
		else 
			test <= truth;
	end

		
	
	//assign cmd
	//The channel is assigned based off the RR_cnt *See the table*
	assign channel = RR_sig[1] ? (RR_sig[0] ? 3'b100: 3'b011) : RR_sig[0] ? 3'b001: 3'b000; 
	assign cmd = {2'b0, channel, 11'h000};
	
	
	///////////////////////////////////////////////
	//////////////STATE Machine////////////////////
	///////////////////////////////////////////////
	typedef enum logic [1:0] {IDLE, Transaction1, Wait, ReadData} state_t;
	state_t state, nxt_state;
	
	// State flop
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			state <= IDLE;
		else
			state <= nxt_state;
	end
	
	
	always_comb begin
		wrt = 0;
		cnv_cmplt = 0;
		nxt_state = state;
		case(state) 
			IDLE: if(count_full) begin	
			  nxt_state = Transaction1;
			  wrt = 1;
			end
			Transaction1: if(done) begin
				nxt_state = Wait;
			end
			Wait: begin
				nxt_state = ReadData;
				wrt = 1;
				end
			ReadData: if (done) begin
				nxt_state = IDLE;
				cnv_cmplt = 1;
				end
			default: nxt_state = IDLE;
		endcase
	end
	// These enables are only true when cnv_cmplt and the respective channel was just used, checked with RR count
	assign torque_en = cnv_cmplt ? &RR_sig ? 1: 0 : 0;
	assign curr_en = cnv_cmplt ? ~RR_sig[1] && RR_sig[0] ? 1 : 0 : 0;
	assign batt_en = cnv_cmplt ? ~|RR_sig ? 1 : 0 : 0;
	assign brake_en = cnv_cmplt ? RR_sig[1] && ~RR_sig[0] ? 1 : 0 : 0;
	// The output flops
	
	always @(posedge clk, negedge rst_n) begin
		if (!rst_n) begin
			batt <= 12'b0;
			torque <= 12'b0;
			curr <= 12'b0;
			brake <= 12'b0;
			
		end
		else if(truth) begin
			
			if (torque_en) begin
				torque <= {rd_data[11:0]};
				
			end
			if(batt_en) begin 
				batt <= {rd_data[11:0]};
				
			end
			if (curr_en)begin 
				curr <= {rd_data[11:0]};
				
			end
			if(brake_en)begin
				brake <= {rd_data[11:0]};
				
			end
			
	end
	end
endmodule
	
	
