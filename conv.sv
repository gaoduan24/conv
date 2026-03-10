module conv #(
    parameter int unsigned P_ICH      = 4,
    parameter int unsigned P_OCH      = 4,
    parameter int unsigned N_ICH      = 16,
    parameter int unsigned N_OCH      = 16,
    parameter int unsigned K          = 3,
    parameter int unsigned A_BIT      = 8,
    parameter int unsigned W_BIT      = 8,
    parameter int unsigned B_BIT      = 32,
    parameter int unsigned N_HW       = 64,
    parameter string       W_FILE     = "",
    parameter              W_ROM_TYPE = "block"
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic [P_ICH*A_BIT-1:0] in_data,
    input  logic                   in_valid,
    output logic                   in_ready,
    output logic [P_OCH*B_BIT-1:0] out_data,
    output logic                   out_valid,
    input  logic                   out_ready
);

    localparam int unsigned FOLD_I = N_ICH / P_ICH;
    localparam int unsigned FOLD_O = N_OCH / P_OCH;
    localparam int unsigned KK = K * K;
    localparam int unsigned WEIGHT_DEPTH = FOLD_O * FOLD_I * KK;
    localparam int unsigned LB_DEPTH = FOLD_I * KK;
    localparam int unsigned LB_AWIDTH = $clog2(LB_DEPTH);

    logic signed [               B_BIT-1:0] acc                   [  P_OCH];
    logic        [      $clog2(N_HW+1)-1:0] cntr_hw;
    logic        [    $clog2(FOLD_O+1)-1:0] cntr_fo;
    logic        [    $clog2(FOLD_I+1)-1:0] cntr_fi;
    logic        [        $clog2(KK+1)-1:0] cntr_kk;
    logic                                   pipe_en_in;
    logic                                   pipe_en_out;
    logic                                   pipe_en;
    logic                                   is_fst_fo;
    logic                                   mac_array_data_vld;
    logic        [         P_ICH*A_BIT-1:0] in_buf;
    logic                                   is_fst_kk_fi;
    logic                                   is_lst_kk_fi;
    logic                                   out_valid_r;
    logic                                   out_push;
    logic                                   mac_en;
    logic                                   flush_consume;
    logic                                   line_buffer_we;
    logic        [           LB_AWIDTH-1:0] line_buffer_waddr;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_wdata;
    logic                                   line_buffer_re;
    logic        [           LB_AWIDTH-1:0] line_buffer_raddr;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_rdata;
    logic        [$clog2(WEIGHT_DEPTH)-1:0] weight_addr;
    logic        [   P_OCH*P_ICH*W_BIT-1:0] weight_data;

    // ---------------------------------------------------------------
    // 延迟寄存器：补偿 ROM / RAM 各 1 拍同步读延迟
    // ---------------------------------------------------------------
    logic                                   is_fst_fo_d1;
    logic                                   is_fst_kk_fi_d1;
    logic                                   is_lst_kk_fi_d1;
    logic                                   pipe_en_in_d1;
    logic        [         P_ICH*A_BIT-1:0] in_data_d1;

    // ---------------------------------------------------------------
    // Weight ROM
    // !! ce0 必须用 pipe_en，而非 pipe_en_out !!
    //
    // 原因：当 fo=0, in_valid=0, out_ready=1 时（输入端 stall）：
    //   pipe_en = 0，计数器停止，weight_addr 不推进
    //   若 ce0 = pipe_en_out = 1，ROM 仍采样同一 addr，
    //   下一拍 weight_data 不变（看似无害）；
    //   但当 in_valid 变 1 后计数器推进，ROM 需要在"原 addr"再保留一拍，
    //   若 ce0 = pipe_en_out，ROM 会在 stall 的下一拍立刻采样新 addr，
    //   导致 weight_data 与 _d1（用 pipe_en 使能）错开一拍！
    //   使用 ce0 = pipe_en 可保证 ROM 推进与计数器完全同步。
    // ---------------------------------------------------------------
    rom #(
        .DWIDTH(P_OCH * P_ICH * W_BIT),
        .AWIDTH($clog2(WEIGHT_DEPTH)),
        .MEM_SIZE(WEIGHT_DEPTH),
        .INIT_FILE(W_FILE),
        .ROM_TYPE(W_ROM_TYPE)
    ) u_weight_rom (
        .clk  (clk),
        .ce0  (pipe_en),        // ← pipe_en，而非 pipe_en_out
        .addr0(weight_addr),
        .q0   (weight_data)
    );

    ram #(
        .DWIDTH(P_ICH * A_BIT),
        .AWIDTH(LB_AWIDTH),
        .MEM_SIZE(LB_DEPTH),
        .RAM_STYLE("ultra")
    ) u_line_buffer (
        .clk  (clk),
        .we   (line_buffer_we),
        .waddr(line_buffer_waddr),
        .wdata(line_buffer_wdata),
        .re   (line_buffer_re),
        .raddr(line_buffer_raddr),
        .rdata(line_buffer_rdata)
    );

    // ---------------------------------------------------------------
    // 组合逻辑
    // ---------------------------------------------------------------
    assign is_fst_fo    = (cntr_fo == 0);
    assign is_fst_kk_fi = (cntr_kk == 0) && (cntr_fi == 0);
    assign is_lst_kk_fi = (cntr_kk == KK - 1) && (cntr_fi == FOLD_I - 1);

    // fo=0：等待 in_valid；fo>0：数据来自 line_buffer，可自由推进
    assign pipe_en_in  = is_fst_fo ? in_valid : 1'b1;
    assign pipe_en_out = out_ready;
    assign pipe_en     = pipe_en_in && pipe_en_out;

    // Retire one pending fo>0 tail result when fetch side stalls at fo=0.
    assign flush_consume = pipe_en_out && !pipe_en && !is_fst_fo_d1 && pipe_en_in_d1 && is_lst_kk_fi_d1;
    assign mac_en        = pipe_en || flush_consume;

    // 握手：fo=0 且下游 ready 时接受输入（in_ready && in_valid 同为 1 才完成事务）
    assign in_ready = is_fst_fo && pipe_en_out;

    // Weight ROM 地址（T 拍发出，T+1 拍数据有效）
    assign weight_addr = (cntr_fo * KK * FOLD_I) + cntr_fi * KK + cntr_kk;

    // Line buffer 写：fo=0 时缓存输入供后续 fo 复用
    assign line_buffer_we    = is_fst_fo && pipe_en;
    assign line_buffer_waddr = cntr_fi * KK + cntr_kk;
    assign line_buffer_wdata = in_data;

    // Line buffer 读：仅在流水线推进时读取，避免 stall 后 rdata 与 _d1 控制错拍
    assign line_buffer_re    = (~is_fst_fo) && pipe_en;
    assign line_buffer_raddr = cntr_fi * KK + cntr_kk;

    // ---------------------------------------------------------------
    // 计数器：最内层 kk → fi → fo → hw
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cntr_hw <= 0;
            cntr_fo <= 0;
            cntr_fi <= 0;
            cntr_kk <= 0;
        end else if (pipe_en) begin
            if (cntr_kk == KK - 1) begin
                cntr_kk <= 0;
                if (cntr_fi == FOLD_I - 1) begin
                    cntr_fi <= 0;
                    if (cntr_fo == FOLD_O - 1) begin
                        cntr_fo <= 0;
                        if (cntr_hw == N_HW - 1) begin
                            cntr_hw <= 0;
                        end else begin
                            cntr_hw <= cntr_hw + 1;
                        end
                    end else begin
                        cntr_fo <= cntr_fo + 1;
                    end
                end else begin
                    cntr_fi <= cntr_fi + 1;
                end
            end else begin
                cntr_kk <= cntr_kk + 1;
            end
        end
    end

    // ---------------------------------------------------------------
    // 延迟寄存器（使能 = pipe_en，与计数器和 ROM 完全同步）
    //
    // T 拍 posedge 同时发生：
    //   - 计数器推进（当前拍状态 → 下一拍状态）
    //   - _d1 捕获 T 拍（推进前）的组合信号
    //   - ROM 采样 T 拍 addr
    // T+1 拍：weight_data 和 _d1 均反映 T 拍状态，完全对齐
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_fst_fo_d1    <= 1'b0;
            is_fst_kk_fi_d1 <= 1'b0;
            is_lst_kk_fi_d1 <= 1'b0;
            pipe_en_in_d1   <= 1'b0;
            in_data_d1      <= '0;
        end else begin
            if (pipe_en) begin          // ← pipe_en，非 pipe_en_out
                is_fst_fo_d1    <= is_fst_fo;
                is_fst_kk_fi_d1 <= is_fst_kk_fi;
                is_lst_kk_fi_d1 <= is_lst_kk_fi;
                pipe_en_in_d1   <= pipe_en_in;
                in_data_d1      <= in_data;
            end else if (flush_consume) begin
                // Consume the pending tail exactly once without advancing fetch counters.
                is_fst_kk_fi_d1 <= 1'b0;
                is_lst_kk_fi_d1 <= 1'b0;
                pipe_en_in_d1   <= 1'b0;
            end
        end
    end

    // ---------------------------------------------------------------
    // MAC 阵列输入组装（T+1 拍：ROM/RAM 数据与 _d1 完全对齐）
    // ---------------------------------------------------------------
    assign mac_array_data_vld = pipe_en_in_d1;
    assign in_buf             = is_fst_fo_d1 ? in_data_d1 : line_buffer_rdata;

    logic        [A_BIT-1:0] x_vec[P_ICH];
    logic signed [W_BIT-1:0] w_vec[P_OCH][P_ICH];

    always_comb begin
        for (int i = 0; i < P_ICH; i++) begin
            x_vec[i] = in_buf[i*A_BIT+:A_BIT];
        end
    end

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            for (int i = 0; i < P_ICH; i++) begin
                w_vec[o][i] = weight_data[(P_ICH*o+i)*W_BIT+:W_BIT];
            end
        end
    end

    generate
        for (genvar o = 0; o < P_OCH; o++) begin : gen_mac_array
            conv_mac_array #(
                .P_ICH(P_ICH),
                .A_BIT(A_BIT),
                .W_BIT(W_BIT),
                .B_BIT(B_BIT)
            ) u_mac_array (
                .clk    (clk),
                .rst_n  (rst_n),
                // !! en 必须用 pipe_en，而非 pipe_en_out !!
                //
                // 若 en = pipe_en_out：当 in_valid=0 造成 stall 时，
                // pipe_en_out=1 但 pipe_en=0，MAC 仍会执行一次"幽灵计算"，
                // 重复累加上一拍的 in_buf * weight_data，导致结果错误。
                // en = pipe_en 保证只有流水线真正推进时 MAC 才累加。
                .en     (mac_en),
                .dat_vld(mac_array_data_vld),
                .clr    (is_fst_kk_fi_d1),
                .x_vec  (x_vec),
                .w_vec  (w_vec[o]),
                .acc    (acc[o])
            );
        end
    endgenerate

    assign out_push  = mac_en && is_lst_kk_fi_d1;
    assign out_valid = out_valid_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid_r <= 1'b0;
        end else begin
            // Keep valid asserted until downstream accepts, and set again when a new result is produced.
            out_valid_r <= (out_valid_r && !out_ready) || out_push;
        end
    end

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            out_data[o*B_BIT+:B_BIT] = acc[o];
        end
    end
endmodule