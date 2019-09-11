module debouncer(	
	 input clk, //this is a 50MHz clock provided on FPGA pin PIN_Y2
    input noisy,  //this is the input to be debounced
    output reg clean  //this is the debounced switch
);
/*This module debounces the pushbutton PB.
 *It can be added to your project files and called as is:
 *DO NOT EDIT THIS MODULE
 */

// Synchronize the switch input to the clock
reg PB_sync_0;
always @(posedge clk) PB_sync_0 <= noisy;
reg PB_sync_1;
always @(posedge clk) PB_sync_1 <= PB_sync_0;

// Debounce the switch
reg [15:0] PB_cnt;
always @(posedge clk)
if(clean==PB_sync_1)
    PB_cnt <= 0;
else
begin
    PB_cnt <= PB_cnt + 1'b1; 
    if(PB_cnt == 16'hffff) clean <= ~clean; 
end
endmodule
