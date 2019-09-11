module TopModule(hw_ram_rasn, hw_ram_casn, hw_ram_wen, hw_ram_ba, hw_ram_udqs_p, 
	hw_ram_udqs_n, hw_ram_ldqs_p, hw_ram_ldqs_n, hw_ram_udm, hw_ram_ldm, hw_ram_ck, 
	hw_ram_ckn, hw_ram_cke, hw_ram_odt, hw_ram_ad, hw_ram_dq, hw_rzq_pin, hw_zio_pin,
	/*CLK,*/ reset, leds, switches, dip_switches, status,
	
	OSC_100MHz, AUD_ADCLRCK,AUD_ADCDAT,AUD_DACLRCK,AUD_DACDAT,AUD_XCK,AUD_BCLK,
	AUD_I2C_SCLK,AUD_I2C_SDAT,AUD_MUTE,PLL_LOCKED,
	/*debugLED,*/write,rdy,resetFSM,DEBUGLED,DEBUGLEDTWO/*,
	
	
	fnumber*//*,data_in,data_out,command*/,
	
	rs232_tx, rs232_rx, reset1, clk,
	cclk, rst, kypd_col, kypd_row, kypd_ddata
	);
	
	input resetFSM;
	
	
	
	
	
	
	//ram below
	
	output reg 	DEBUGLED;
	output	DEBUGLEDTWO;
	output 	hw_ram_rasn;
	output 	hw_ram_casn;
	output 	hw_ram_wen;
	output[2:0] hw_ram_ba;
	inout 	hw_ram_udqs_p;
	inout 	hw_ram_udqs_n;
	inout 	hw_ram_ldqs_p;
	inout 	hw_ram_ldqs_n;
	output 	hw_ram_udm;
	output 	hw_ram_ldm;
	output 	hw_ram_ck;
	output 	hw_ram_ckn;
	output 	hw_ram_cke;
	output 	hw_ram_odt;
	output[12:0] hw_ram_ad;
	inout [15:0] hw_ram_dq;
	inout 	hw_rzq_pin;
	inout 	hw_zio_pin;
	
	//input 	CLK, reset;
	input 	reset;
	input		write;
	output 	status;	
	
	input  [7:0]	switches; 		// address
	input	 [7:0]	dip_switches; 	// data to be written into RAM
	output [7:0]	leds;			// data read out of RAM
	
	wire system_clk;	
	
	//Audio Below
	input  OSC_100MHz;

	inout  AUD_ADCLRCK;
	input  AUD_ADCDAT;

	inout  AUD_DACLRCK;
	output AUD_DACDAT;

	output AUD_XCK;
	inout  AUD_BCLK;

	output AUD_I2C_SCLK;
	inout  AUD_I2C_SDAT;

	output AUD_MUTE;
	output PLL_LOCKED;
	 
	 
	/*wire  [3:0] KEY;
	wire  [3:0] SW;
	wire [3:0] LED;*/
//	assign KEY = switches[3:0];
//	assign SW = switches[7:4];
//	assign LED = leds[3:0];
	 
//	input  [3:0] KEY;
//	input  [3:0] SW;
//	output [3:0] LED;
	
	
	//wire reset = !KEY[0];
	wire main_clk;
	wire audio_clk;

	wire [1:0] sample_end;
	wire [1:0] sample_req;
	wire [15:0] audio_output;
	wire [15:0] audio_input;

	wire CLK_OUT1;
	wire CLK_OUT2;
	
	//these are only for the state table!#@!@#!@#   probably delete later
		//wire rdy, 	dataPresent;
	//wire [25:0]	max_ram_address;
	
	/*reg [3:0]	state=4'b0000;
	
	parameter stInit = 4'b0000;
	parameter stReadFromPorts = 4'b0001;
	parameter stMemWrite = 4'b0010;
	parameter stMemReadReq  = 4'b0011;
	parameter stMemReadData = 4'b0100;*/
	
	//reg 	[25:0] address = 0;
	//reg 	[7:0]	ram_data_in;
	//wire 	[7:0]	ram_data_out;
	//reg	[7:0] dataOut; 
	//reg 			reqRead;
	//reg 			enableWrite;
	//reg 			ackRead = 0;
	
	reg 		[2:0] fnumber;
	wire 		[3:0] command;
	/*input 		[7:0] data_in;
	output reg [7:0] data_out;*/
	
	input	reset1;
	input	clk;
	input rs232_rx;
	output rs232_tx;
	
	input cclk;
	input rst; 
	input [3:0] kypd_col;
	input [3:0] kypd_row;
	output [3:0] kypd_ddata;
	
	parameter MESSAGE_BITS = 3;
	parameter MEM_ADDRESS_BITS = 26;

	reg [MESSAGE_BITS-1:0] fnumber_addr;
	reg [MEM_ADDRESS_BITS-MESSAGE_BITS-1:0] offset;
	wire [MEM_ADDRESS_BITS-1:0] full_ram_addr;
	assign full_ram_addr = {fnumber_addr, offset};

	reg [MEM_ADDRESS_BITS-MESSAGE_BITS-1:0] fsizes [0:7];  // 8 long array of 23 bit numbers

	wire [MEM_ADDRESS_BITS-1:0] max_ram_address;
	reg [15:0] ram_data_in;//
	wire [15:0] ram_data_out;
	reg reqRead;
	reg ackRead = 0;
	output reg rdy;	
	wire dataPresent;
	reg enableWrite;

	wire ram_rdy;
	reg mem_full;
	
	reg [15:0] dat;
	
	
	
	reg [10:0] delay_count = 0;

	
	
	
	
	
	
	
	
	
	
	
	wire endOfAudio;
		
	parameter c_play = 4'h1;
	parameter c_pause = 4'h2;
	parameter c_skip = 4'h3;
	parameter c_record = 4'h4;
	parameter c_delete = 4'h5;
	parameter c_delete_all = 4'h6;
	parameter c_volume_up = 4'h7;
	parameter c_volume_down = 4'h8;
	parameter c_message_a = 4'ha;
	parameter c_message_b = 4'hb;
	parameter c_message_c = 4'hc;
	parameter c_message_d = 4'hd;
	parameter c_message_e = 4'he;
	
	parameter s_wait_command  = 5'b00000;
	parameter s_finish  = 5'b00001;
	parameter s_delete  = 5'b00010;
	parameter s_delete_all  = 5'b00011;
	parameter s_play   = 5'b00100;
	parameter s_play2  = 5'b00101;
	parameter s_play3  = 5'b00110;
	parameter s_record  = 5'b00111;
	parameter s_record2  = 5'b01000;
	parameter s_record3 = 5'b01001;
	parameter s_skip = 5'b01010;
	parameter s_volume_up = 5'b01011;
	parameter s_volume_down = 5'b01100;
	parameter s_message_a = 5'b01101;
	parameter s_message_b = 5'b01110;
	parameter s_message_c = 5'b01111;
	parameter s_message_d = 5'b10000;
	parameter s_message_e = 5'b10001;
	
	parameter s_delayPlay = 5'b10010;
	parameter s_delayRecord = 5'b10011;
	
  mainClocks instance_name
   (// Clock in ports	
    .OSC_100MHz(system_clk),      // IN				//was osc_100mhz not system clock
    // Clock out ports
    .CLK_OUT1(CLK_OUT1));


ram_interface_wrapper myram (
    .address(full_ram_addr), 
    .data_in(ram_data_in), 
    .write_enable(enableWrite), 
    .read_request(reqRead), 
    .read_ack(ackRead), 
    .data_out(ram_data_out), 
    .reset(reset), 
    .clk(OSC_100MHz), 
    .hw_ram_rasn(hw_ram_rasn), 
    .hw_ram_casn(hw_ram_casn), 
    .hw_ram_wen(hw_ram_wen), 
    .hw_ram_ba(hw_ram_ba), 
    .hw_ram_udqs_p(hw_ram_udqs_p), 
    .hw_ram_udqs_n(hw_ram_udqs_n), 
    .hw_ram_ldqs_p(hw_ram_ldqs_p), 
    .hw_ram_ldqs_n(hw_ram_ldqs_n), 
    .hw_ram_udm(hw_ram_udm), 
    .hw_ram_ldm(hw_ram_ldm), 
    .hw_ram_ck(hw_ram_ck), 
    .hw_ram_ckn(hw_ram_ckn), 
    .hw_ram_cke(hw_ram_cke), 
    .hw_ram_odt(hw_ram_odt), 
    .hw_ram_ad(hw_ram_ad), 
    .hw_ram_dq(hw_ram_dq), 
    .hw_rzq_pin(hw_rzq_pin), 
    .hw_zio_pin(hw_zio_pin), 
    .clkout(system_clk), 
    .sys_clk(system_clk), 
    .rdy(ram_rdy), 
    .rd_data_pres(dataPresent), 
    .max_ram_address(max_ram_address)/*, 
    .ledRAM(leds)*/
    );
	 
 sockit_top myaudio (
    .OSC_100MHz(CLK_OUT1),
    .AUD_ADCLRCK(AUD_ADCLRCK), 
    .AUD_ADCDAT(AUD_ADCDAT), 
    .AUD_DACLRCK(AUD_DACLRCK), 
    .AUD_DACDAT(AUD_DACDAT), 
    .AUD_XCK(AUD_XCK), 
    .AUD_BCLK(AUD_BCLK), 
    .AUD_I2C_SCLK(AUD_I2C_SCLK), 
    .AUD_I2C_SDAT(AUD_I2C_SDAT), 
    .AUD_MUTE(AUD_MUTE), 
    .PLL_LOCKED(PLL_LOCKED),
	 .SWITCHES(switches),
    .LED(led), 
    .endOfAudio(endOfAudio),
	 .audio_output(audio_output), 
    .audio_input(audio_input)
    );
	 
	 
	 
	loopback test(
    .switches(switches), 
    .rs232_tx(rs232_tx), 
    .rs232_rx(rs232_rx), 
    .reset1(reset1), 
    .clk(CLK_OUT1)
    );
	 
	 keypad kypd (
	 .cclk(cclk),
	 .rst(rst),
	 .kypd_col(kypd_col),
	 .kypd_row(kypd_row),
	 .kypd_ddata(kypd_ddata)
	 );

//assign leds = switches;
//
//assign status = 1'b1;


(* FSM_ENCODING="SEQUENTIAL", SAFE_IMPLEMENTATION="NO" *) reg [4:0] state = s_finish;

always@(posedge system_clk)

      if (resetFSM) begin
         state <= s_wait_command;
         reqRead <= 0;
         ackRead <= 0;
         enableWrite <= 0;
         fsizes[0] <= 0;
         fsizes[1] <= 0;
         fsizes[2] <= 0;
         fsizes[3] <= 0;
         fsizes[4] <= 0;
			fnumber <= 3'b000;
			DEBUGLED = 1'b0;
		end
      else
         case (state)	//commented out "(* FULL_CASE, PARALLEL_CASE *)"
            s_wait_command : begin
               if (command == c_play) begin 
                  state <= s_play;
                  offset <= 0;
                  fnumber_addr <= fnumber;
                  rdy <= 0;
               end
               
               else if (command == c_record) begin  
                  state <= s_record;
                  fnumber_addr <= fnumber;
                  offset <= 0;
                  rdy <= 0;
               end
               
               else if (command == c_delete) begin
                  state <= s_delete;
                  fnumber_addr <= fnumber;
                  rdy <= 0;
               end 
               
               else if (command == c_delete_all) begin
                  state <= s_delete_all;
                  fnumber_addr <= fnumber;
                  rdy <= 0;
               end 
               
               else begin
                    rdy <= 1;
                    reqRead <= 0;
                    ackRead <= 0;
               end
               
            end
            
            s_finish : begin
                if (!command) begin
                    mem_full <= 0;
                    rdy <= 0;
                    state <= s_wait_command;
                end
            end
                    
            s_delete : begin
                fsizes[fnumber_addr] <= 0;
                state <= s_finish;
            end
            
            s_delete_all : begin
                fsizes[0] <= 0;
                fsizes[1] <= 0;
                fsizes[2] <= 0;
                fsizes[3] <= 0;
                fsizes[4] <= 0;
                fsizes[5] <= 0;
                state <= s_finish;
            end
            
            s_play : begin 
					DEBUGLED = 1'b0;
                ackRead <= 0;
                if (offset == fsizes[fnumber_addr])  // Are we at the end of of the message?
                    state <= s_finish;
          
                else if (!endOfAudio)  // wait for endOfAudio to drop low again
                    state <= s_delayPlay;
            end
            
            s_play2 : begin
					
                if (/*endOfAudio && */(command == c_play)) begin  // Wait for the frame endOfAudio to get the next byte
                    state <= s_play3;
                    reqRead <= 1'b1;
                end
                else if (command == c_pause)  // If pause, just stay in this block
                    state <= s_play2;
                else if (command != c_play)  // Something other than play means finish up and return to function select
                    state <= s_finish;
                else
                    state <= s_play2;
            end
            
            s_play3 : begin
					 reqRead <= 1'b0;
                if (!dataPresent) begin // Wait for RAM to present the data
                    state <= s_play3;
					 end
                else begin
						  DEBUGLED = 1'b1;	//just for debugging, remove later
                    dat <= ram_data_out;		//brooks did it like this 
                    offset <= offset + 1;																	// this might need to happen on a diff clock cycle
                    //reqRead <= 0;  // Do I need to deassert this more quickly? 
                    ackRead <= 1'b1; 
                    state <= s_play;
                end
            end
            
            
            s_record : begin
                if (command != c_record) begin  // When we finish recording, write out our file size
                    state <= s_finish;
                    fsizes[fnumber_addr] <= offset;
                end
                    
                else if (endOfAudio) begin  // wait for frame endOfAudio to grab data and write
                    ram_data_in <= audio_input;
						  dat <= audio_input;
                    enableWrite <= 1;
                    state <= s_delayRecord;
                end
            end
            
            s_record2 : begin
                enableWrite <= 0;
                offset <= offset + 1;
                state <= s_record3;
            end
            
            s_record3 : begin
                if (offset == 0) begin  // praying for rollover - max message length
                    mem_full <= 1;
                    fsizes[fnumber_addr] <= -1;  // please please wrap around
                    state <= s_finish;
                end
                else if (!endOfAudio)  // wait for endOfAudio to drop
                    state <= s_record;
            end
				
				s_delayRecord: begin
					if(delay_count < 800) begin	//brooks said 350
						delay_count <= delay_count + 1;
					end else begin
						delay_count <= 0;
						state <= s_record2;
					end
				end
				
				s_delayPlay: begin
					if(delay_count < 800) begin	//brooks said 350
						delay_count <= delay_count + 1;
					end else begin
						delay_count <= 0;
						state <= s_play2;
					end
				end
				
			endcase

			
//assign audio_output = audio_input;	//junk
assign status = rdy; // rdy signal from ram
//assign leds = dataOut; // output data read to leds
//assign leds = kypd_ddata[3:0];
//assign leds = audio_input[7:0];
//assign audio_output = audio_input;
assign audio_output = dat;



//assign leds[7:3] = state;
//assign leds[2:0] = command;
//assign leds = delay_count[7:0];

assign leds = offset[7:0];
//assign leds = dat;

assign DEBUGLEDTWO = endOfAudio;
assign command = switches[3:0];


//assign DEBUGLED = switches[3];

//assign DEBUGLEDTHREE = dataPresent;
endmodule
