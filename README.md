# FCFI-landing-pad

Downolad CVA6 from https://github.com/openhwgroup/cva6 , build it and replace the cva6/core folder with this core.

*Verilator v4.110

*cva6 version 71e7019

*Make sure that the $RISCV variable points to the path where riscv tools are installed

Updates in CVA6 core (following riscv-cfi spec):
1. instr_tracer_pkg : Declare lpad instruction format.
2. ariane_pkg	    : Include lpad instruction in fu_op enumaration.
3. riscv_pkg	    : Declare elp enumaration.
		                Declare cfi_t struct.
		                Include cfi_t in instruction_t union.
		                Include menvcfg,menvcfgh,senvcfg,mseccfg,henvcfg,henvcfgh csrs in csr_reg_t enum.
		                Declare env_cfg,msec_cfg structs.
		                Update status_rv_t,dcsr_t structs.
4. instr_queue    : CFI FSM.
5. decoder        : Add lpad instruction in decoding proccess. (UIMM to align with lui instruction immed)
6. alu            : Add lpad instruction in branch_resolve process:   LPAD:     alu_branch_res_o = ~|(fu_data_i.operand_a ^ fu_data_i.operand_b); // if 1 match else mismatch
7. branch_unit    : Add exception in case of mismatch
8. csr_regfile    : Include menvcfg,menvcfgh,senvcfg,mseccfg,henvcfg,henvcfgh csrs in read/write processes and in reset/sync process.
                    CFI update logic ( write process ).
                    Preserve and update on traps.
