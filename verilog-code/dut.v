module MyDesign(
    //Control signals
      output reg        dut_busy                   ,
      input  wire       dut_run                    , 
      input  wire       reset_b                    ,  
      input  wire       clk                        ,
     
    //---------------------------------------------------------------------------
    //Input SRAM interface
      output            input_sram_write_enable    ,
      output reg [11:0] input_sram_write_addresss  ,
      output reg [15:0] input_sram_write_data      ,
      output     [11:0] input_sram_read_address    ,
      input wire [15:0] input_sram_read_data       ,
    
    //---------------------------------------------------------------------------
    //Output SRAM interface
      output reg        output_sram_write_enable    ,
      output reg [11:0] output_sram_write_addresss  ,
      output reg [15:0] output_sram_write_data      ,
      output reg [11:0] output_sram_read_address    ,
      input wire [15:0] output_sram_read_data       ,

    //---------------------------------------------------------------------------
    //scratchpad SRAM interface
      output            scratchpad_sram_write_enable    ,
      output reg [11:0] scratchpad_sram_write_addresss  ,
      output reg [15:0] scratchpad_sram_write_data      ,
      output reg [11:0] scratchpad_sram_read_address    ,
      input wire [15:0] scratchpad_sram_read_data       ,
    
    //---------------------------------------------------------------------------
    //Weights SRAM interface                                                       
      output reg        weights_sram_write_enable    ,
      output reg [11:0] weights_sram_write_addresss  ,
      output reg [15:0] weights_sram_write_data      ,
      output reg [11:0] weights_sram_read_address    ,
      input wire [15:0] weights_sram_read_data       

);

  //using local parameters to set names to fsm states--------------------------

  localparam reset_state              = 'd0;
  localparam pre_data_fetch_state     = 'd1;
  localparam initial_data_fetch_state = 'd2;
  localparam inter_conv1_state        = 'd3;
  localparam inter_conv2_state        = 'd4;
  localparam inter_conv3_state        = 'd5;
  localparam inter_conv4_state        = 'd6;
  localparam inter_conv5_state        = 'd7;
  localparam inter_conv6_state        = 'd8;
  localparam conv_compare_state       = 'd9;
  localparam max_pool1_state          = 'd10;
  localparam max_pool2_state          = 'd11;
  localparam write_to_output_sram     = 'd12;
  localparam next_data_fetch_state    = 'd13;
  localparam stop_state               = 'd14;

  //reg declarations-----------------------------------------------------------

  reg               ip_end_reg, 
                    even_max_pool_valid_bit, 
                    odd_max_pool_valid_bit;

  reg        [3:0]  current_state;
  reg        [3:0]  kernel_counter;

  reg        [7:0]  current_size_reg;
  reg        [5:0]  input_row_index;
  reg        [5:0]  input_col_index;
  reg        [5:0]  conv_row_index;
  reg        [5:0]  conv_even_col_index;
  reg        [5:0]  conv_odd_col_index;
  reg        [5:0]  max_pool_reg_write_ind;
  reg        [5:0]  max_pool_reg_read_ind;

  reg signed [7:0]  temp_even_max_pool_reg;
  reg signed [7:0]  temp_odd_max_pool_reg;
  reg signed [7:0]  k_reg_array             [8:0]; // 9 kernel elements of 1 byte each
  reg signed [7:0]  temp_max_pool_reg_array [30:0]; // 31 intermediate feature map registers for max_pooling stage

  reg        [11:0] static_base_addr;
  reg        [11:0] output_sram_addr;
  reg        [11:0] dynamic_base_addr;

  reg signed [15:0] multiplier_1_reg;
  reg signed [15:0] multiplier_2_reg;
  reg signed [15:0] multiplier_3_reg;
  reg signed [15:0] multiplier_4_reg;

  reg signed [19:0] mac0_reg;
  reg signed [19:0] mac1_reg;
  reg signed [19:0] temp_conv_mac0_output;
  reg signed [19:0] temp_conv_mac1_output;

  //wire declarations-----------------------------------------------------------------

  wire status_k_fetch_complete, status_last_index_arrived;

  wire signed [7:0]  temp_even_max_pool_out;
  wire signed [7:0]  temp_odd_max_pool_out;
  wire signed [7:0]  max_pool_out;

  wire signed [7:0]  ip_sram_read_msb_out;
  wire signed [7:0]  ip_sram_read_lsb_out;
  wire signed [7:0]  relued_max_out;
  wire signed [7:0]  temp_max_out;

  wire signed [19:0] max_feature_out;

  //wire definitions in data flow modelling

  //setting sram writes to 0
  assign input_sram_write_enable = 1'd0;
  assign scratchpad_sram_write_enable = 1'd0;

  //for reading from ip sram, calc address
  assign input_sram_read_address = ( (current_state == inter_conv6_state) && 
                                    (conv_row_index == current_size_reg - 3) && 
                                    (conv_odd_col_index == current_size_reg - 3) ) ? static_base_addr : dynamic_base_addr ;

  //2 data packets arrive in same address line
  assign ip_sram_read_lsb_out = input_sram_read_data[7:0];
  assign ip_sram_read_msb_out = input_sram_read_data[15:8];

  //kernel counter counts from 0 to n. when n reaches 'b100 kernel fetch is complete
  assign status_k_fetch_complete = kernel_counter[3];

  //calc end of conv states to calc next addr
  assign status_last_index_arrived = (input_row_index == current_size_reg - 1 && input_col_index    == current_size_reg - 2);

  //max pooling
  assign max_feature_out   = (mac0_reg > mac1_reg)      ? mac0_reg : mac1_reg;

  //relu
  assign max_pool_out      = temp_max_pool_reg_array[max_pool_reg_read_ind];

  assign temp_max_out      = (max_feature_out < 0)      ? 'd0      : max_feature_out;
  assign relued_max_out    = (max_feature_out > 'sd127) ? 'd127    : temp_max_out;

  assign temp_odd_max_pool_out  = (max_pool_out > relued_max_out) ? max_pool_out : relued_max_out;
  assign temp_even_max_pool_out = (max_pool_out > relued_max_out) ? max_pool_out : relued_max_out;

  /////////sequential logic modelling

  //when should we change states?
  always @(posedge clk) begin
    if(!reset_b) begin
      current_state <= 'd0;
    end
    else begin
      casex(current_state)
        reset_state: begin
          if(dut_run == 1)
            current_state <=  pre_data_fetch_state;
          else
            current_state <=  reset_state;
        end
        pre_data_fetch_state: begin
          current_state <=  initial_data_fetch_state;
        end
        initial_data_fetch_state: begin
          if(status_k_fetch_complete == 1'd1)
            current_state <=  inter_conv1_state;
          else
            current_state <=  initial_data_fetch_state;
        end
        inter_conv1_state: begin
          if(current_size_reg != 'hff)
            current_state <=  inter_conv2_state;
          else
            current_state <=  stop_state;
        end
        inter_conv2_state: begin
          current_state <=  inter_conv3_state;
        end
        inter_conv3_state: begin
          current_state <=  inter_conv4_state;
        end
        inter_conv4_state: begin
          current_state <=  inter_conv5_state;
        end
        inter_conv5_state: begin
          current_state <=  inter_conv6_state;
        end
        inter_conv6_state: begin
          if(conv_row_index[0] == 0)
            current_state <=  conv_compare_state;
          else if(even_max_pool_valid_bit == 1'd0)
            current_state <=  max_pool1_state;
          else if(even_max_pool_valid_bit == 1'd1) 
            current_state <=  max_pool2_state;
        end
        conv_compare_state: begin
          current_state <=  inter_conv1_state;
        end
        max_pool1_state: begin
          if(conv_row_index == current_size_reg - 3 && conv_odd_col_index == current_size_reg - 3)
            current_state <=  write_to_output_sram;
          else
            current_state <=  inter_conv1_state;
        end
        max_pool2_state: begin
          current_state <=  write_to_output_sram;
        end
        write_to_output_sram: begin
          if(ip_end_reg == 1'd1)
            current_state <=  next_data_fetch_state;
          else
            current_state <=  inter_conv1_state;
        end
        next_data_fetch_state: begin
          if(current_size_reg != 'd63)//h3f
            current_state <=  inter_conv1_state;
          else
            current_state <=  stop_state;
        end
        stop_state: begin
          if(dut_run == 1'd0)
            current_state <=  reset_state;
          else
            current_state <=  stop_state;

        end
        default: begin
            current_state <=  reset_state;
        end
      endcase
    end
  end

  //when we are in a current_state, what must happen for every clock cycle?
  always @(posedge clk) begin
    casex(current_state)

      reset_state: begin
        weights_sram_read_address   <= 'd0;
        kernel_counter              <= 'd0;
        mac0_reg                    <= 'd0;
        mac1_reg                    <= 'd0;
        static_base_addr            <= 'd0;
        conv_row_index              <= 'd0;
        conv_even_col_index         <= 'd0;
        conv_odd_col_index          <= 'd1;
        max_pool_reg_write_ind      <= 'd0;
        max_pool_reg_read_ind       <= 'd0;
        output_sram_addr            <= 'd0;
        dut_busy                    <= 'd0;
        even_max_pool_valid_bit     <= 'd0;
        odd_max_pool_valid_bit      <= 'd0;
        output_sram_write_enable    <= 'd0;
      end

      pre_data_fetch_state: begin
        weights_sram_read_address <= weights_sram_read_address + 1'd1;
      end

      initial_data_fetch_state: begin
        k_reg_array[kernel_counter     ] <= weights_sram_read_data[15:8];
        k_reg_array[kernel_counter + 1 ] <= weights_sram_read_data[7:0];
        kernel_counter <= kernel_counter + 2;
        current_size_reg <= input_sram_read_data[7:0];
        dut_busy <= 'd1;

        if (kernel_counter <= 8) begin
          weights_sram_read_address <= weights_sram_read_address + 1'd1;
        end
      end

      inter_conv1_state,
      inter_conv2_state,
      inter_conv3_state,
      inter_conv4_state,
      inter_conv6_state: begin 
        mac1_reg <= temp_conv_mac1_output;
        mac0_reg <= temp_conv_mac0_output;
      end

      //inorder to start calculating next conv elems from conv_compare_state we need the addr to be "available" in conv6 state.
      inter_conv5_state: begin
        mac1_reg <= temp_conv_mac1_output;
        mac0_reg <= temp_conv_mac0_output;
        if(status_last_index_arrived == 1) begin
          static_base_addr <= dynamic_base_addr + 1'd1;
        end
      end

      conv_compare_state: begin
        temp_max_pool_reg_array[max_pool_reg_write_ind] <= relued_max_out; 

        if(max_pool_reg_write_ind == 'd30) begin
          max_pool_reg_write_ind <= 'd0;
        end
        else begin
          max_pool_reg_write_ind <= max_pool_reg_write_ind + 1'd1;
        end
        if (conv_odd_col_index == current_size_reg - 3) begin
          conv_row_index <= conv_row_index + 1'd1;
          conv_even_col_index <= 'd0;
          conv_odd_col_index <= 'd1;
        end
        else begin
          conv_row_index <= conv_row_index;
          conv_even_col_index <= conv_even_col_index + 'd2;
          conv_odd_col_index <= conv_odd_col_index + 'd2;
        end
      end

      max_pool1_state: begin

        temp_even_max_pool_reg <= temp_even_max_pool_out;
        even_max_pool_valid_bit <= 1'd1;
        ip_end_reg <= (conv_row_index  == current_size_reg - 3 && conv_odd_col_index == current_size_reg - 3); //input ended
        
        if(max_pool_reg_read_ind == 'd30) begin
          max_pool_reg_read_ind <= 'd0;
        end
        else begin
          max_pool_reg_read_ind <= max_pool_reg_read_ind + 1'd1;
        end

        if(conv_row_index == current_size_reg - 3 && conv_odd_col_index == current_size_reg -3) begin
          current_size_reg <= input_sram_read_data[07:00];
        end
        if(conv_row_index != current_size_reg - 2 && conv_odd_col_index != current_size_reg - 2) begin
          if (conv_odd_col_index == current_size_reg - 3) begin
            conv_row_index <= conv_row_index + 1'd1;
            conv_even_col_index <= 'd0;
            conv_odd_col_index <= 'd1;
          end
          else begin
            conv_row_index <= conv_row_index;
            conv_even_col_index <= conv_even_col_index + 'd2;
            conv_odd_col_index <= conv_odd_col_index + 'd2;
          end
        end
      end

      max_pool2_state: begin
        temp_odd_max_pool_reg <= temp_odd_max_pool_out;
        odd_max_pool_valid_bit <= 1'd1;
        output_sram_write_enable  <= 1'd1;
        if(max_pool_reg_read_ind == 'd30) begin //check
          max_pool_reg_read_ind <= 'd0;
        end
        else begin
          max_pool_reg_read_ind <= max_pool_reg_read_ind + 1'd1;
        end
      end

      write_to_output_sram: begin
        output_sram_write_addresss <= output_sram_addr;
        output_sram_addr <= output_sram_addr + 1'd1;

        //grouping data to write two data in single line
        output_sram_write_data <= { 
                                    (even_max_pool_valid_bit ? (temp_even_max_pool_reg) : 8'd0), 
                                    (odd_max_pool_valid_bit  ? (temp_odd_max_pool_reg ) : 8'd0) 
                                  };

        if(conv_row_index != current_size_reg - 2 && conv_odd_col_index != current_size_reg - 2) begin
          if(conv_odd_col_index == current_size_reg - 3) begin
            conv_row_index <= conv_row_index + 1'd1;
            conv_even_col_index <= 'd0;
            conv_odd_col_index <= 'd1;
          end
          else begin
            conv_row_index <= conv_row_index;
            conv_even_col_index <= conv_even_col_index + 'd2;
            conv_odd_col_index <= conv_odd_col_index + 'd2;
          end
        end
        if(ip_end_reg == 1'd1) begin
          conv_row_index <= 1'd0;
          conv_even_col_index <= 1'd0;
          conv_odd_col_index <= 1'd1;
        end

        even_max_pool_valid_bit <= 'd0;
        odd_max_pool_valid_bit <= 'd0;

      end

      next_data_fetch_state: begin
        max_pool_reg_write_ind <= 'd0;
        max_pool_reg_read_ind <= 'd0;
        ip_end_reg <= 0;
      end

      stop_state: begin
        dut_busy <= 'd0;
      end
      
    endcase
  end

  //combinational logic modelling--------------------------------------------------

  //how does input row and column change ( to be used for data retrival ) when current state changes? 
  always@(*) begin
    casex(current_state)

      inter_conv1_state: begin
        input_row_index = conv_row_index;
        input_col_index = conv_even_col_index + 2;
      end

      inter_conv2_state: begin
        input_row_index = conv_row_index + 1;
        input_col_index = conv_even_col_index;
      end

      inter_conv3_state: begin
        input_row_index = conv_row_index + 1;
        input_col_index = conv_even_col_index + 2;
      end

      inter_conv4_state: begin
        input_row_index = conv_row_index + 2;
        input_col_index = conv_even_col_index;
      end

      inter_conv5_state: begin
        input_row_index = conv_row_index  + 2;
        input_col_index = conv_even_col_index + 2;    
      end

      inter_conv6_state: begin
        if (conv_odd_col_index == current_size_reg - 3)
          begin
            input_row_index = conv_row_index + 1; 
            input_col_index = 0;
          end
        else
          begin
            input_row_index = conv_row_index;
            input_col_index = conv_odd_col_index + 1;
          end
      end

      conv_compare_state: begin
        if (conv_odd_col_index == current_size_reg - 3)
          begin
            input_row_index = conv_row_index + 1; 
            input_col_index = 0;
          end
        else
          begin
            input_row_index = conv_row_index;
            input_col_index = conv_odd_col_index + 1;
          end
      end

      default: begin
        input_row_index = conv_row_index;
        input_col_index = conv_even_col_index;
      end
    endcase
  end

  //how does dynamic base address for fetching from input change for every change in state?
  always @(*) begin
    casex(current_state)
      reset_state: begin
        dynamic_base_addr = 'd0;
      end
      pre_data_fetch_state: begin
        dynamic_base_addr = 'd0;
      end
      initial_data_fetch_state: begin
        if(status_k_fetch_complete)
          dynamic_base_addr = 'd1;
        else
          dynamic_base_addr = 'd0;
      end
      
      inter_conv1_state,
      inter_conv2_state,
      inter_conv3_state,
      inter_conv4_state,
      inter_conv5_state: begin
        dynamic_base_addr   = static_base_addr + (((input_row_index*current_size_reg) + input_col_index ) >> 1 ) + 1;
      end
      
      inter_conv6_state,
      conv_compare_state,
      max_pool2_state,
      write_to_output_sram: begin 
        if(conv_odd_col_index == current_size_reg - 3)
          dynamic_base_addr = static_base_addr + (((conv_row_index+1)*current_size_reg)>>1) + 1'd1;
        else
          dynamic_base_addr = static_base_addr + ((conv_row_index*current_size_reg + conv_odd_col_index + 1'd1)>>1) + 1'd1;
      end
      max_pool1_state: begin
        if(conv_row_index == current_size_reg - 3 && conv_odd_col_index == current_size_reg - 3)
          dynamic_base_addr = static_base_addr + 1'd1;
        else begin
          if(conv_odd_col_index == current_size_reg - 3)
            dynamic_base_addr = static_base_addr + (((conv_row_index+1)*current_size_reg)>>1) + 1'd1;
          else
            dynamic_base_addr = static_base_addr + ((conv_row_index*current_size_reg + conv_odd_col_index + 1'd1)>>1) + 1'd1;
        end
      end
      next_data_fetch_state: begin
        dynamic_base_addr = static_base_addr + 1'd1;
      end
      stop_state: begin
        dynamic_base_addr = 'd0;
      end
      default: begin
        dynamic_base_addr = 'd0;
      end
    endcase
  end

  //conv is made of mult and accum. mult has a pattern of kernel values feeding into it for every conv state which is modelled here
  always@(*) begin
    casex(current_state)

    inter_conv2_state: begin
      multiplier_1_reg = (k_reg_array[2] * ip_sram_read_msb_out);
      multiplier_2_reg = 0;
      multiplier_3_reg = (k_reg_array[1] * ip_sram_read_msb_out);
      multiplier_4_reg = (k_reg_array[2] * ip_sram_read_lsb_out);
    end

    inter_conv3_state: begin
      multiplier_1_reg = (k_reg_array[3] * ip_sram_read_msb_out);
      multiplier_2_reg = (k_reg_array[4] * ip_sram_read_lsb_out);
      multiplier_3_reg = 0;
      multiplier_4_reg = (k_reg_array[3] * ip_sram_read_lsb_out);
    end

    inter_conv4_state: begin
      multiplier_1_reg = (k_reg_array[5] * ip_sram_read_msb_out);
      multiplier_2_reg = 0;
      multiplier_3_reg = (k_reg_array[4] * ip_sram_read_msb_out);
      multiplier_4_reg = (k_reg_array[5] * ip_sram_read_lsb_out);
    end

    inter_conv5_state: begin
      multiplier_1_reg = (k_reg_array[6] * ip_sram_read_msb_out);
      multiplier_2_reg = (k_reg_array[7] * ip_sram_read_lsb_out);
      multiplier_3_reg = 0;
      multiplier_4_reg = (k_reg_array[6] * ip_sram_read_lsb_out);
    end

    inter_conv6_state: begin
      multiplier_1_reg = (k_reg_array[8] * ip_sram_read_msb_out);
      multiplier_2_reg = 0;
      multiplier_3_reg = (k_reg_array[7] * ip_sram_read_msb_out);
      multiplier_4_reg = (k_reg_array[8] * ip_sram_read_lsb_out);
    end

    default: begin
        multiplier_1_reg = (k_reg_array[0] * ip_sram_read_msb_out);
        multiplier_2_reg = (k_reg_array[1] * ip_sram_read_lsb_out);
        multiplier_3_reg = 0;
        multiplier_4_reg = (k_reg_array[0] * ip_sram_read_lsb_out);
    end
    endcase
  end

  //conv is made of mult and accum. mult is taken care in seperate procedural block
  always @(*) begin

    if((current_state == reset_state) ||
      (current_state == pre_data_fetch_state) ||
      (current_state == initial_data_fetch_state))  begin
      temp_conv_mac0_output = 0;
      temp_conv_mac1_output = 0;
    end

    else if (current_state == inter_conv1_state)  begin
      temp_conv_mac0_output = multiplier_1_reg + multiplier_2_reg;
      temp_conv_mac1_output = multiplier_3_reg + multiplier_4_reg;
    end
    
    else begin
      temp_conv_mac0_output = mac0_reg + multiplier_1_reg + multiplier_2_reg;
      temp_conv_mac1_output = mac1_reg + multiplier_3_reg + multiplier_4_reg;
    end
  end

endmodule
