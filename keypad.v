module keypad (
    input cclk,
    input rst,  
    input [3:0] kypd_col,
    input [3:0] kypd_row,
    output reg [3:0] kypd_ddata
);

always@(posedge cclk) begin
	 if (rst == 1) begin
		  kypd_ddata <= 4'b0;
	  end
	  else begin
			if (kypd_col == 4'b1110) begin
				 if (kypd_row == 4'b1110)
					 kypd_ddata <= 4'h1;
				 else if (kypd_row == 4'b1101)  
					 kypd_ddata <= 4'h4;
				 else if (kypd_row == 4'b1011)  
					 kypd_ddata <= 4'h7;
				 else if (kypd_row == 4'b0111)  
					 kypd_ddata <= 4'h0;
			end
			else if (kypd_col == 4'b1101) begin
				 if (kypd_row == 4'b1110)
					 kypd_ddata <= 4'h2;
				 else if (kypd_row == 4'b1101)
					 kypd_ddata <= 4'h5;
				 else if (kypd_row == 4'b1011)
					 kypd_ddata <= 4'h8;
				 else if (kypd_row == 4'b0111)
					 kypd_ddata <= 4'hf;
			end
			else if (kypd_col == 4'b1011) begin
				 if (kypd_row == 4'b1110)
					 kypd_ddata <= 4'h3;
				 else if (kypd_row == 4'b1101) 
					 kypd_ddata <= 4'h6;
				 else if (kypd_row == 4'b1011) 
					 kypd_ddata <= 4'h9;
				 else if (kypd_row == 4'b0111) 
					 kypd_ddata <= 4'he;
			end
			else if (kypd_col == 4'b0111) begin
				 if (kypd_row == 4'b1110)
					 kypd_ddata <= 4'ha;
				 else if (kypd_row == 4'b1101)
					 kypd_ddata <= 4'hb;
				 else if (kypd_row == 4'b1011) 
					 kypd_ddata <= 4'hc;
				 else if (kypd_row == 4'b0111) 
					 kypd_ddata <= 4'hd;
			end
	  end
end 
endmodule 
  