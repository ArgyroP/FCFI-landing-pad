// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 26.10.2018sim:/ariane_tb/dut/i_ariane/i_frontend/icache_ex_valid_q

// Description: Instruction Queue, separates instruction front-end from processor
//              back-end.
//
// This is an optimized instruction queue which supports the handling of
// compressed instructions (16 bit instructions). Internally it is organized as
// FETCH_ENTRY x 32 bit queues which are filled in a consecutive manner. Two pointers
// point into (`idx_is_q` and `idx_ds_q`) the fill port and the read port. The read port
// is designed so that it will easily allow for multiple issue implementation.
// The input supports arbitrary power of two instruction fetch widths.
//
// The queue supports handling of branch prediction and will take care of
// only saving a valid instruction stream.
//
// Furthermore it contains a replay interface in case the instruction queue
// is already full. As instructions are in general easily replayed this should
// increase the efficiency as I$ misses are potentially hidden. This stands in
// contrast to pessimistic actions (early stalling) or credit based approaches.
// Credit based systems might be difficult to implement with the current system
// as we do not exactly know how much space we are going to need in the fifos
// as each instruction can take either one or two slots.
//
// So the consumed/valid interface degenerates to a `information` interface. If the
// upstream circuits keeps pushing the queue will discard the information
// and start replaying from the point were it could last manage to accept instructions.
//
// The instruction front-end will stop issuing instructions as soon as the
// fifo is full. This will gate the logic if the processor is e.g.: halted
//
// TODO(zarubaf): The instruction queues can be reduced to 16 bit. Potentially
// the replay mechanism gets more complicated as it can be that a 32 bit instruction
// can not be pushed at once.

module instr_queue import ariane_pkg::*; (
  input  logic                                               clk_i,
  input  logic                                               rst_ni,
  input  logic                                               flush_i,
  input  logic [ariane_pkg::INSTR_PER_FETCH-1:0][31:0]       instr_i,
  input  logic [ariane_pkg::INSTR_PER_FETCH-1:0][riscv::VLEN-1:0] addr_i,
  input  logic [ariane_pkg::INSTR_PER_FETCH-1:0]             valid_i,
  output logic                                               ready_o,
  output logic [ariane_pkg::INSTR_PER_FETCH-1:0]             consumed_o,
  // we've encountered an exception, at this point the only possible exceptions are page-table faults
  input  ariane_pkg::frontend_exception_t                    exception_i,
  input  logic [riscv::VLEN-1:0]                             exception_addr_i,
  // branch predict
  input  logic [riscv::VLEN-1:0]                             predict_address_i,
  input  ariane_pkg::cf_t  [ariane_pkg::INSTR_PER_FETCH-1:0] cf_type_i,
  // replay instruction because one of the FIFO was already full
  output logic                                               replay_o,
  output logic [riscv::VLEN-1:0]                             replay_addr_o, // address at which to replay this instruction
  // to processor backend
  output ariane_pkg::fetch_entry_t                           fetch_entry_o,
  output logic                                               fetch_entry_valid_o,
  input  logic                                               fetch_entry_ready_i,

  input  logic                                               move,
  output riscv::elp                                               elp_o,
  input logic  [1:0]                                               complete_cfi,
  //input  riscv::label                                        label_i,
  //input  logic                                               external_interrupt,
  //input  riscv::lp_expected                                  lp_exp,
  input  logic                                               xLPAD_i,
  input  logic                                               ex_valid_i
  //input  logic                                               eret_i
);

  typedef struct packed {
    logic [31:0]     instr; // instruction word
    ariane_pkg::cf_t cf;    // branch was taken
    ariane_pkg::frontend_exception_t ex;    // exception happened
    logic [riscv::VLEN-1:0] ex_vaddr;       // lower VLEN bits of tval for exception
  } instr_data_t;

  logic [ariane_pkg::LOG2_INSTR_PER_FETCH-1:0] branch_index;
  // instruction queues
  logic [ariane_pkg::INSTR_PER_FETCH-1:0]
        [$clog2(ariane_pkg::FETCH_FIFO_DEPTH)-1:0] instr_queue_usage;
  instr_data_t [ariane_pkg::INSTR_PER_FETCH-1:0]   instr_data_in, instr_data_out;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0]          push_instr, push_instr_fifo;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0]          pop_instr;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0]          instr_queue_full;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0]          instr_queue_empty;
  logic instr_overflow;
  // address queue
  logic [$clog2(ariane_pkg::FETCH_FIFO_DEPTH)-1:0] address_queue_usage;
  logic [riscv::VLEN-1:0] address_out;
  logic pop_address;
  logic push_address;
  logic full_address;
  logic empty_address;
  logic address_overflow;
  // input stream counter
  logic [ariane_pkg::LOG2_INSTR_PER_FETCH-1:0] idx_is_d, idx_is_q;
  // Registers
  // output FIFO select, one-hot
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] idx_ds_d, idx_ds_q;
  logic [riscv::VLEN-1:0] pc_d, pc_q; // current PC
  logic reset_address_d, reset_address_q; // we need to re-set the address because of a flush

  logic [ariane_pkg::INSTR_PER_FETCH*2-2:0] branch_mask_extended;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] branch_mask;
  logic branch_empty;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] taken;
  // shift amount, e.g.: instructions we want to retire
  logic [ariane_pkg::LOG2_INSTR_PER_FETCH:0] popcount;
  logic [ariane_pkg::LOG2_INSTR_PER_FETCH-1:0] shamt;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] valid;
  logic [ariane_pkg::INSTR_PER_FETCH*2-1:0] consumed_extended;
  // FIFO mask
  logic [ariane_pkg::INSTR_PER_FETCH*2-1:0] fifo_pos_extended;
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] fifo_pos;
  logic [ariane_pkg::INSTR_PER_FETCH*2-1:0][31:0] instr;
  ariane_pkg::cf_t [ariane_pkg::INSTR_PER_FETCH*2-1:0] cf;
  // replay interface
  logic [ariane_pkg::INSTR_PER_FETCH-1:0] instr_overflow_fifo;

  assign ready_o = ~(|instr_queue_full) & ~full_address;
  
  if (ariane_pkg::RVC) begin : gen_multiple_instr_per_fetch_with_C
  
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_unpack_taken
      assign taken[i] = cf_type_i[i] != ariane_pkg::NoCF;
    end
   
    // calculate a branch mask, e.g.: get the first taken branch
    lzc #(
      .WIDTH   ( ariane_pkg::INSTR_PER_FETCH ),
      .MODE    ( 0                           ) // count trailing zeros
    ) i_lzc_branch_index (
      .in_i    ( taken          ), // we want to count trailing zeros
      .cnt_o   ( branch_index   ), // first branch on branch_index
      .empty_o ( branch_empty   )
    );
  
 
    // the first index is for sure valid
    // for example (64 bit fetch):
    // taken mask: 0 1 1 0
    // leading zero count = 1
    // 0 0 0 1, 1 1 1 << 1 = 0 0 1 1, 1 1 0
    // take the upper 4 bits: 0 0 1 1
    assign branch_mask_extended = {{{ariane_pkg::INSTR_PER_FETCH-1}{1'b0}}, {{ariane_pkg::INSTR_PER_FETCH}{1'b1}}} << branch_index;
    assign branch_mask = branch_mask_extended[ariane_pkg::INSTR_PER_FETCH * 2 - 2:ariane_pkg::INSTR_PER_FETCH - 1];

    // mask with taken branches to get the actual amount of instructions we want to push
    assign valid = valid_i & branch_mask;
    // rotate right again
    assign consumed_extended = {push_instr_fifo, push_instr_fifo} >> idx_is_q;
    assign consumed_o = consumed_extended[ariane_pkg::INSTR_PER_FETCH-1:0];
    // count the numbers of valid instructions we've pushed from this package
    popcount #(
      .INPUT_WIDTH   ( ariane_pkg::INSTR_PER_FETCH )
    ) i_popcount (
      .data_i     ( push_instr_fifo ),
      .popcount_o ( popcount        )
    );
    assign shamt = popcount[$bits(shamt)-1:0];

    // save the shift amount for next cycle
    assign idx_is_d = idx_is_q + shamt;
    //assign elp_init=elp;
    // ----------------------
    // Input interface
    // ----------------------
    // rotate left by the current position
    assign fifo_pos_extended = { valid, valid } << idx_is_q;
    // we just care about the upper bits
    assign fifo_pos = fifo_pos_extended[ariane_pkg::INSTR_PER_FETCH*2-1:ariane_pkg::INSTR_PER_FETCH];
    // the fifo_position signal can directly be used to guide the push signal of each FIFO
    // make sure it is not full
    assign push_instr = fifo_pos & ~instr_queue_full;
  
    // duplicate the entries for easier selection e.g.: 3 2 1 0 3 2 1 0
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_duplicate_instr_input
      assign instr[i] = instr_i[i];
      assign instr[i + ariane_pkg::INSTR_PER_FETCH] = instr_i[i];
      assign cf[i] = cf_type_i[i];
      assign cf[i + ariane_pkg::INSTR_PER_FETCH] = cf_type_i[i];
    end

    // shift the inputs
    for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_fifo_input_select
      /* verilator lint_off WIDTH */
      assign instr_data_in[i].instr = instr[i + idx_is_q];
      assign instr_data_in[i].cf = cf[i + idx_is_q];
      assign instr_data_in[i].ex = exception_i; // exceptions hold for the whole fetch packet
      assign instr_data_in[i].ex_vaddr = exception_addr_i;
      /* verilator lint_on WIDTH */
    end
  end else begin : gen_multiple_instr_per_fetch_without_C
    
    assign taken = '0;
    assign branch_empty = '0;
    assign branch_index = '0;
    assign branch_mask_extended = '0;
    assign branch_mask = '0;
    assign consumed_extended = '0;
    assign fifo_pos_extended = '0;
    assign fifo_pos = '0;
    assign instr = '0;
    assign popcount = '0;
    assign shamt = '0;
    assign valid = '0;
    
    
    assign consumed_o = push_instr_fifo[0];
    // ----------------------
    // Input interface
    // ----------------------
    assign push_instr = valid_i & ~instr_queue_full;    
    
    /* verilator lint_off WIDTH */
    assign instr_data_in[0].instr = instr_i[0];
    assign instr_data_in[0].cf = cf_type_i[0];
    assign instr_data_in[0].ex = exception_i; // exceptions hold for the whole fetch packet
    assign instr_data_in[0].ex_vaddr = exception_addr_i;
    /* verilator lint_on WIDTH */
  end

  // ----------------------
  // Replay Logic
  // ----------------------
  // We need to replay a instruction fetch iff:
  // 1. One of the instruction data FIFOs was full and we needed it
  // (e.g.: we pushed and it was full)
  // 2. The address/branch predict FIFO was full
  // if one of the FIFOs was full we need to replay the faulting instruction
  if (ariane_pkg::RVC == 1'b1) begin : gen_instr_overflow_fifo_with_C
    assign instr_overflow_fifo = instr_queue_full & fifo_pos;
  end else begin : gen_instr_overflow_fifo_without_C
    assign instr_overflow_fifo = instr_queue_full & valid_i;
  end
  assign instr_overflow = |instr_overflow_fifo; // at least one instruction overflowed
  assign address_overflow = full_address & push_address;
  assign replay_o = instr_overflow | address_overflow;

  if (ariane_pkg::RVC) begin : gen_replay_addr_o_with_c
    // select the address, in the case of an address fifo overflow just
    // use the base of this package
    // if we successfully pushed some instructions we can output the next instruction
    // which we didn't manage to push
    assign replay_addr_o = (address_overflow) ? addr_i[0] : addr_i[shamt];
  end else begin : gen_replay_addr_o_without_C
    assign replay_addr_o = addr_i[0];
  end
  
  // ----------------------
  // Downstream interface
  // ----------------------
  // as long as there is at least one queue which can take the value we have a valid instruction
  assign fetch_entry_valid_o = ~(&instr_queue_empty);


  //FSM states
    enum logic [2:0]{
        NO_LP_EXPECTED_STATE,         //0
        WAIT_STATE,             //1 wait for the branch to be resolved
        LP_EXPECTED_STATE_LPAD,          //2
        LP_EXPECTED_STATE_MATCH, //3
        ILLEGAL_STATE     //4
    }curr_state,next_state;
    logic is_lp_expected_jalr, is_lp_expected_cmpr_jump;
    logic cfi_on_q,cfi_on_d;
    //riscv::label cur_label;
    //logic cfi_on_q,cfi_on_d,elp,wait_for_fetch_valid_d,wait_for_fetch_valid_q,cfi_illegal,checked,checked_ll,checked_ml,checked_ul;
  if (ariane_pkg::RVC) begin : gen_downstream_itf_with_c
    always_comb begin
      idx_ds_d = idx_ds_q;

      pop_instr = '0;
      // assemble fetch entry
      fetch_entry_o.instruction = '0;
      fetch_entry_o.address = pc_q;
      fetch_entry_o.ex.valid = 1'b0;
      fetch_entry_o.ex.cause = '0;
      elp_o = riscv::NO_LP_EXPECTED;
      fetch_entry_o.ex.tval = '0;
      fetch_entry_o.branch_predict.predict_address = address_out;
      fetch_entry_o.branch_predict.cf = ariane_pkg::NoCF;
      // output mux select
      for (int unsigned i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin
        if (idx_ds_q[i]) begin
          if (instr_data_out[i].ex == ariane_pkg::FE_INSTR_ACCESS_FAULT) begin
            fetch_entry_o.ex.cause = riscv::INSTR_ACCESS_FAULT;
          end else begin
            fetch_entry_o.ex.cause = riscv::INSTR_PAGE_FAULT;
          end

        if(xLPAD_i) begin
          case (curr_state)

            NO_LP_EXPECTED_STATE: begin
              elp_o = riscv::NO_LP_EXPECTED;
              if(instr_data_out[i].instr[1:0]==riscv::OpcodeC1 && instr_data_out[i].instr[15:13] == riscv::OpcodeC1LuiAddi16sp && instr_data_out[i].instr[11:7] == 5'b00111) begin
               cfi_on_d =1'b1;
               
              end
              if(cfi_on_q) begin
                is_lp_expected_jalr = ((instr_data_out[i].instr[6:0] == riscv::OpcodeJalr) && (instr_data_out[i].instr[19:15] != 5'b00001) && (instr_data_out[i].instr[19:15] != 5'b00101) && (instr_data_out[i].instr[19:15] != 5'b00111) ) ? 1 : 0;
                is_lp_expected_cmpr_jump = ( ((instr_data_out[i].instr[1:0]==riscv::OpcodeC2 && instr_data_out[i].instr[15:13] == riscv::OpcodeC2JalrMvAdd && instr_data_out[i].instr[6:2] == 5'b0 &&  instr_data_out[i].instr[12]== 1'b0) || 
                ( instr_data_out[i].instr[1:0]==riscv::OpcodeC2 && instr_data_out[i].instr[15:13] == riscv::OpcodeC2JalrMvAdd && instr_data_out[i].instr[12] != 1'b0 && instr_data_out[i].instr[11:7] != 5'b0 && instr_data_out[i].instr[6:2] == 5'b0)) && 
                (instr_data_out[i].instr[11:7] != 5'b00001) && (instr_data_out[i].instr[11:7] != 5'b00101) && (instr_data_out[i].instr[11:7] != 5'b00111) ) ? 1 : 0;
            
              end
               if(is_lp_expected_jalr || is_lp_expected_cmpr_jump) next_state = WAIT_STATE;
               else next_state = NO_LP_EXPECTED_STATE;
              
            end

            WAIT_STATE: begin

              elp_o = riscv::LP_EXPECTED;
              if(instr_data_out[i].ex != ariane_pkg::FE_NONE || ex_valid_i) begin
                  next_state = NO_LP_EXPECTED_STATE; //interrupt
                  elp_o = riscv::NO_LP_EXPECTED;
                  cfi_on_d = 1'b0;
                end else begin
                   if (move) begin // brach is resolved
                     next_state = LP_EXPECTED_STATE_LPAD;
                   end else next_state = WAIT_STATE;
                end
            end
            LP_EXPECTED_STATE_LPAD: begin
                elp_o = riscv::LP_EXPECTED;
                is_lp_expected_jalr = 1'b0;
                is_lp_expected_cmpr_jump = 1'b0;
                if(instr_data_out[i].ex != ariane_pkg::FE_NONE || ex_valid_i) begin
                  next_state = NO_LP_EXPECTED_STATE; //interrupt
                  elp_o = riscv::NO_LP_EXPECTED;
                  cfi_on_d = 1'b0;
                end
                if(fetch_entry_valid_o) begin
                  if(instr_data_out[i].instr[6:0] == riscv::OpcodeAuipc && instr_data_out[i].instr[11:7] == 5'b0000) begin
                    next_state = LP_EXPECTED_STATE_MATCH;
                    cfi_on_d=1'b0;
                  end else next_state = ILLEGAL_STATE;
                end 

                //if(!cfi_on_q && complete_cfi) next_state = NO_LP_EXPECTED_STATE;
            end
            LP_EXPECTED_STATE_MATCH: begin
                    elp_o = riscv::LP_EXPECTED;
                    if(complete_cfi == 2'b11) next_state = NO_LP_EXPECTED_STATE;
                    else if(complete_cfi == 2'b00) next_state = ILLEGAL_STATE;
                    else next_state = LP_EXPECTED_STATE_MATCH;
            end
            ILLEGAL_STATE: begin
              //fetch_entry_o.ex.valid = 1'b1;
              //fetch_entry_o.ex.tval  = 'h0002;
              elp_o = riscv::LP_EXPECTED;
              
            end
            
          endcase
        end
         /* if(xLPAD_i) begin
            next_state = curr_state;
            elp=1'b0;
            case (curr_state)
              NO_LP_EXPECTED: begin
                cfi_illegal=1'b0;
                checked=1'b0;
                checked_ll=1'b0;
                checked_ml=1'b0;
                checked_ul=1'b0;
                if (instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7]== 8'b10000000 
                   && instr_data_out[i].instr[31:24]== 8'b10000010)//lpsll 
                   if(fetch_entry_valid_o)
                      cfi_on_d=1'b1;
                if(cfi_on_q) begin

                  if (instr_data_out[i].instr[6:0] == riscv::OpcodeJalr && (instr_data_out[i].instr[11:7]== 5'd1 
                    || instr_data_out[i].instr[11:7]== 5'd5) && (instr_data_out[i].instr[19:15]== 5'd1 
                    || instr_data_out[i].instr[19:17]== 5'd5)) begin //jalr
                     $display("from jalr");
                    next_state = WAIT_STATE;
                    end
                  if(instr_data_out[i].instr[6:0] == riscv::OpcodeC2)begin
                    if(instr_data_out[i].instr[11:7] != 5'b0 && instr_data_out[i].instr[6:2] == 5'b0)begin //c.jalr
                     $display("from c.jalr");
                    next_state = WAIT_STATE;
                    end

                    if(instr_data_out[i].instr[15:13]==riscv::OpcodeC2JalrMvAdd && instr_data_out[i].instr[6:2] == 5'b0)begin //c.jalr
                    $display("from c.jr");
                    next_state = WAIT_STATE;
                    end
                  end

                  if ((instr_data_out[i].instr[6:0] == riscv::OpcodeJalr && (instr_data_out[i].instr[11:7]== 5'd1 
                    || instr_data_out[i].instr[11:7]== 5'd5)) ||
                     (instr_data_out[i].instr[6:0] == riscv::OpcodeJalr && instr_data_out[i].instr[14:7]=='0 && instr_data_out[i].instr[31:20]=='0
                     ) || (instr_data_out[i].instr[6:0] == riscv::OpcodeC2 && (instr_i[11:7] != 5'b0 && instr_i[6:2] == 5'b0)) || (instr_data_out[i].instr[6:0] == riscv::OpcodeC2  && instr_i[6:2] == 5'b0)) begin //jalr || c.jalr || c.jr
                      if(instr_data_out[i].instr[6:0] == riscv::OpcodeJalr) $display("from jalr");
                      if(instr_data_out[i].instr[6:0] == riscv::OpcodeC2) $display("from c.jalr or jr");
                      next_state = WAIT_STATE;
                    end
                end
              end
              WAIT_STATE: begin
                cfi_illegal=1'b0;
                if(instr_data_out[i].ex != ariane_pkg::FE_NONE /*|| external_interrupt */ /*|| ex_valid_i) begin
                  next_state = NO_LP_EXPECTED; //interrupt
                  cfi_on_d = 1'b0;
                end else begin
                   //if (move) begin
                    if ((instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:24] == 8'b10000011) || (instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:23] == 9'b100001101) || (instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:23] == 9'b100001111) ) begin 
                       checked = 1'b1;
                      if(label_i.LL == instr_data_out[i].instr[23:15] ) begin
                         $display("l label match check");
                         checked_ll=1'b1;
                         //next_state = M_LP_EXPECTED; //branch resolved
                      end else if( label_i.ML == instr_data_out[i].instr[22:15]) begin
                        checked_ml=1'b1;
                        $display("m label match check");
                       // next_state = U_LP_EXPECTED; //branch resolved
                      end else if( label_i.UL == instr_data_out[i].instr[22:15]) begin
                        checked_ul=1'b1;
                       // next_state = U_LP_EXPECTED; //branch resolved
                        $display("u label match check");
                      end       
                    end// else
                    // next_state = L_LP_EXPECTED; //branch resolved
                    
                   //end
                  if (move) begin
                    if(checked_ll && !checked_ml && !checked_ul) next_state = M_LP_EXPECTED;
                    else if(checked_ml && !checked_ul) next_state = U_LP_EXPECTED;
                    else if(checked_ul) next_state = U_LP_EXPECTED;
                    else next_state = L_LP_EXPECTED; 
                  end
                    
                end
              end
              L_LP_EXPECTED: begin 
                cfi_illegal=1'b0;
                elp = 1'b1;
                
                if(instr_data_out[i].ex != ariane_pkg::FE_NONE /*|| external_interrupt*/ /* ) begin
                  next_state = NO_LP_EXPECTED; //interrupt
                  cfi_on_d = 1'b0;
                end else if(fetch_entry_valid_o) begin
                if(instr_data_out[i].ex == ariane_pkg::FE_NONE /*&& !external_interrupt*/ /*&& !checked) begin
                  if(!(instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:24] == 8'b10000011)) begin//no lpcll
                      fetch_entry_o.ex.cause = riscv::ILLEGAL_INSTR;
                      cfi_illegal=1'b1;
                      next_state = ILLEGAL_INSTR_STATE;
                      $display("illegal instr: %d",instr_data_out[i].instr);
                    end
                  else if ((instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                          && instr_data_out[i].instr[31:24] == 8'b10000011)) begin
                            //cfi_on_d=1'b0;
                            if(label_i.LL == instr_data_out[i].instr[23:15]) begin
                                next_state = M_LP_EXPECTED; //completed cfi  
                                $display("l label match");   
                            end  else begin
                               fetch_entry_o.ex.cause = riscv::ILLEGAL_INSTR; //labels don't match
                               $display(" l label DONT match, fetch_valid_o %0b",fetch_entry_o.ex.valid); 
                               cfi_illegal=1'b1;
                               next_state = ILLEGAL_INSTR_STATE;
                            end
                          end 
                end else if(checked) begin 
                  next_state = NO_LP_EXPECTED; //completed cfi  
                   cfi_on_d=1'b0;
                   if(label_i.LL == instr_data_out[i].instr[23:15] || label_i.ML == instr_data_out[i].instr[22:15] || label_i.UL == instr_data_out[i].instr[22:15]) begin
                                next_state = NO_LP_EXPECTED; //completed cfi  
                                $display("Checked label match");                             
                    end
                end    
              end 

              end
              M_LP_EXPECTED:begin
                cfi_illegal=1'b0;
                elp = 1'b1;
                if(instr_data_out[i].ex != ariane_pkg::FE_NONE /*|| external_interrupt*/ /* ) begin
                  next_state = NO_LP_EXPECTED; //interrupt
                  cfi_on_d = 1'b0;
                end else if(fetch_entry_valid_o) begin
                    if(!(instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:25] == 7'b1000011 && instr_data_out[i].instr[24:23] == 2'b01 )) begin //no lpcml only lower label exist -> cfi complete
                     next_state = NO_LP_EXPECTED; 
                     cfi_on_d=1'b0;

                     $display("label match, only lower label exists");
                    end else if (instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:25] == 7'b1000011 && instr_data_out[i].instr[24:23] == 2'b01) begin //lpcml exists
                       if(label_i.ML == instr_data_out[i].instr[22:15]) begin
                                next_state = U_LP_EXPECTED; //completed cfi  
                                $display("M label match");                             
                       end else if(label_i.ML==8'b0)
                            next_state = M_LP_EXPECTED;
                            else begin
                               fetch_entry_o.ex.cause = riscv::ILLEGAL_INSTR; //labels don't match
                               $display("m label DONT match, instr %0b, label %0b",instr_data_out[i].instr,label_i.ML); 
                               cfi_illegal=1'b1;
                               next_state = ILLEGAL_INSTR_STATE;
                            end
                    end
                end
              end
              U_LP_EXPECTED:begin
                cfi_illegal=1'b0;
                elp = 1'b1;
                if(instr_data_out[i].ex != ariane_pkg::FE_NONE /*|| external_interrupt*/ /* ) begin
                  next_state = NO_LP_EXPECTED; //interrupt
                  cfi_on_d = 1'b0;
                end else if(fetch_entry_valid_o) begin
                    if(!(instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:25] == 7'b1000011 && instr_data_out[i].instr[24:23] == 2'b11 )) begin //no lpcUl only lower label exist -> cfi complete
                     next_state = NO_LP_EXPECTED; 
                     cfi_on_d=1'b0;
                     $display("label match, only MIDLE label exists OR checked in u label");
                    end else if (instr_data_out[i].instr[6:0] == riscv::OpcodeSystem && instr_data_out[i].instr[14:7] == 8'b10000000 
                    && instr_data_out[i].instr[31:25] == 7'b1000011 && instr_data_out[i].instr[24:23] == 2'b11) begin //lpcUl exists
                       if(label_i.UL == instr_data_out[i].instr[22:15]) begin
                              cfi_on_d=1'b0;
                                next_state = NO_LP_EXPECTED; //completed cfi  
                                $display("U label match"); 
                            end else begin
                               fetch_entry_o.ex.cause = riscv::ILLEGAL_INSTR; //labels don't match
                               $display("u label DONT match, fetch_valid_o %0b",fetch_entry_o.ex.valid); 
                               cfi_illegal=1'b1;
                               next_state = ILLEGAL_INSTR_STATE;
                            end
                    end
                end
              end
              ILLEGAL_INSTR_STATE: begin
                cfi_illegal=1'b0;
                cfi_on_d=1'b0;
                elp = 1'b0;
                if(eret_i && lp_exp == riscv::LP_EXPECTED) next_state = NO_LP_EXPECTED; //the only way this happens is when an interrupt throuws as here and after the return an lpcll is executed
            end

            endcase 
          end */
          
          fetch_entry_o.instruction = instr_data_out[i].instr;
          fetch_entry_o.ex.valid = (instr_data_out[i].ex != ariane_pkg::FE_NONE) || (next_state == ILLEGAL_STATE);
          if(next_state == ILLEGAL_STATE) fetch_entry_o.ex.tval = 'h00000002;
          else  fetch_entry_o.ex.tval  = {{64-riscv::VLEN{1'b0}}, instr_data_out[i].ex_vaddr};
          fetch_entry_o.branch_predict.cf = instr_data_out[i].cf;
          pop_instr[i] = fetch_entry_valid_o & fetch_entry_ready_i;
        end
      end
      // rotate the pointer left
      if (fetch_entry_ready_i) begin
        idx_ds_d = {idx_ds_q[ariane_pkg::INSTR_PER_FETCH-2:0], idx_ds_q[ariane_pkg::INSTR_PER_FETCH-1]};
      end
    end
  end else begin : gen_downstream_itf_without_c
    always_comb begin
      idx_ds_d = '0;
      idx_is_d = '0;
      fetch_entry_o.instruction = instr_data_out[0].instr;
      fetch_entry_o.address = pc_q;
    
      fetch_entry_o.ex.valid = instr_data_out[0].ex != ariane_pkg::FE_NONE;
      if (instr_data_out[0].ex == ariane_pkg::FE_INSTR_ACCESS_FAULT) begin
        fetch_entry_o.ex.cause = riscv::INSTR_ACCESS_FAULT;
      end else begin
        fetch_entry_o.ex.cause = riscv::INSTR_PAGE_FAULT;
      end
      fetch_entry_o.ex.tval = {{64-riscv::VLEN{1'b0}}, instr_data_out[0].ex_vaddr};
    
      fetch_entry_o.branch_predict.predict_address = address_out;
      fetch_entry_o.branch_predict.cf = instr_data_out[0].cf;
    
      pop_instr[0] = fetch_entry_valid_o & fetch_entry_ready_i;
    end
  end

  // TODO(zarubaf): This needs to change for dual-issue
  // if the handshaking is successful and we had a prediction pop one address entry
  assign pop_address = ((fetch_entry_o.branch_predict.cf != ariane_pkg::NoCF) & |pop_instr);

  // ----------------------
  // Calculate (Next) PC
  // ----------------------
  always_comb begin
    pc_d = pc_q;
    reset_address_d = flush_i ? 1'b1 : reset_address_q;

    if (fetch_entry_ready_i) begin
      // TODO(zarubaf): This needs to change for a dual issue implementation
      // advance the PC
      if (ariane_pkg::RVC == 1'b1) begin : gen_pc_with_c_extension
        pc_d =  pc_q + ((fetch_entry_o.instruction[1:0] != 2'b11) ? 'd2 : 'd4);
      end else begin : gen_pc_without_c_extension
        pc_d =  pc_q + 'd4;
      end
    end

    if (pop_address) pc_d = address_out;

      // we previously flushed so we need to reset the address
    if (valid_i[0] && reset_address_q) begin
      // this is the base of the first instruction
      pc_d = addr_i[0];
      reset_address_d = 1'b0;
    end
  end

  // FIFOs
  for (genvar i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin : gen_instr_fifo
    // Make sure we don't save any instructions if we couldn't save the address
    assign push_instr_fifo[i] = push_instr[i] & ~address_overflow;
    fifo_v3 #(
      .DEPTH      ( ariane_pkg::FETCH_FIFO_DEPTH ),
      .dtype      ( instr_data_t                 )
    ) i_fifo_instr_data (
      .clk_i      ( clk_i                ),
      .rst_ni     ( rst_ni               ),
      .flush_i    ( flush_i              ),
      .testmode_i ( 1'b0                 ),
      .full_o     ( instr_queue_full[i]  ),
      .empty_o    ( instr_queue_empty[i] ),
      .usage_o    ( instr_queue_usage[i] ),
      .data_i     ( instr_data_in[i]     ),
      .push_i     ( push_instr_fifo[i]   ),
      .data_o     ( instr_data_out[i]    ),
      .pop_i      ( pop_instr[i]         )
    );
  end
  // or reduce and check whether we are retiring a taken branch (might be that the corresponding)
  // fifo is full.
  always_comb begin
    push_address = 1'b0;
    // check if we are pushing a ctrl flow change, if so save the address
    for (int i = 0; i < ariane_pkg::INSTR_PER_FETCH; i++) begin
      push_address |= push_instr[i] & (instr_data_in[i].cf != ariane_pkg::NoCF);
    end
  end

  fifo_v3 #(
    .DEPTH      ( ariane_pkg::FETCH_FIFO_DEPTH ), // TODO(zarubaf): Fork out to separate param
    .DATA_WIDTH ( riscv::VLEN                  )
  ) i_fifo_address (
    .clk_i      ( clk_i                        ),
    .rst_ni     ( rst_ni                       ),
    .flush_i    ( flush_i                      ),
    .testmode_i ( 1'b0                         ),
    .full_o     ( full_address                 ),
    .empty_o    ( empty_address                ),
    .usage_o    ( address_queue_usage          ),
    .data_i     ( predict_address_i            ),
    .push_i     ( push_address & ~full_address ),
    .data_o     ( address_out                  ),
    .pop_i      ( pop_address                  )
  );

  unread i_unread_address_fifo (.d_i(|{empty_address, address_queue_usage}));
  unread i_unread_branch_mask (.d_i(|branch_mask_extended));
  unread i_unread_lzc (.d_i(|{branch_empty}));
  unread i_unread_fifo_pos (.d_i(|fifo_pos_extended)); // we don't care about the lower signals
  unread i_unread_instr_fifo (.d_i(|instr_queue_usage));

  if (ariane_pkg::RVC) begin : gen_pc_q_with_c
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        idx_ds_q        <= 'b1;
        idx_is_q        <= '0;
        pc_q            <= '0;
        reset_address_q <= 1'b1;
         //cfi process signals
        //cfi_illegal=1'b0;
        cfi_on_q          <=1'b0;
         curr_state <= NO_LP_EXPECTED_STATE; // Initial state
        //count=0;
      end else begin
        pc_q            <= pc_d;
        reset_address_q <= reset_address_d;
        //cfi process signals
        cfi_on_q          <= cfi_on_d;
        curr_state <= next_state;
        if (flush_i) begin
          // one-hot encoded
          idx_ds_q        <= 'b1;
          // binary encoded
          idx_is_q        <= '0;
          reset_address_q <= 1'b1;
        end else begin
          idx_ds_q        <= idx_ds_d;
          idx_is_q        <= idx_is_d;
        end
      end
    end
  end else begin : gen_pc_q_without_C
    assign idx_ds_q = '0;
    assign idx_is_q = '0;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        pc_q            <= '0;
        reset_address_q <= 1'b1;
      end else begin
        pc_q            <= pc_d;
        reset_address_q <= reset_address_d;
        if (flush_i) begin
          reset_address_q <= 1'b1;
        end
      end
    end
  end

  // pragma translate_off
  `ifndef VERILATOR
      replay_address_fifo: assert property (
        @(posedge clk_i) disable iff (!rst_ni) replay_o |-> !i_fifo_address.push_i
      ) else $fatal(1,"[instr_queue] Pushing address although replay asserted");

      output_select_onehot: assert property (
        @(posedge clk_i) $onehot0(idx_ds_q)
      ) else begin $error("Output select should be one-hot encoded"); $stop(); end
  `endif
  // pragma translate_on
endmodule
