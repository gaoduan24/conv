module mac_body #(
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic        [A_BIT-1:0] x,
    input  logic signed [W_BIT-1:0] w,
    input  logic signed [B_BIT-1:0] cas_in,
    output logic signed [B_BIT-1:0] cas_out
);
    logic signed [B_BIT-1:0] prod;
    assign prod    = x * w; 
    assign cas_out = prod + cas_in;
endmodule

module mac_tail #(
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    en,
    input  logic                    dat_vld,
    input  logic                    clr,
    input  logic        [A_BIT-1:0] x,
    input  logic signed [W_BIT-1:0] w,
    input  logic signed [B_BIT-1:0] cas_in,
    output logic signed [B_BIT-1:0] acc
);
    logic signed [B_BIT-1:0] prod;
    logic signed [B_BIT-1:0] sum;
    logic signed [B_BIT-1:0] acc_r;
    assign prod = x * w;
    assign sum  = prod + cas_in;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_r <= '0;
        end else if (en && dat_vld) begin
            if (clr) begin
                acc_r <= sum;
            end else begin
                acc_r <= acc_r + sum;
            end
        end
    end

    assign acc = acc_r;
endmodule

module conv_mac_array #(
    parameter int unsigned P_ICH = 4,
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    en,
    input  logic                    dat_vld,
    input  logic                    clr,
    input  logic        [A_BIT-1:0] x_vec  [P_ICH],
    input  logic signed [W_BIT-1:0] w_vec  [P_ICH],
    output logic signed [B_BIT-1:0] acc
);

    logic signed [B_BIT-1:0] mac_cascade[P_ICH];

    assign mac_cascade[0] = '0;

    generate
        for (genvar i = 0; i < P_ICH - 1; i++) begin : gen_mac_body
            mac_body #(
                .A_BIT(A_BIT),
                .W_BIT(W_BIT),
                .B_BIT(B_BIT)
            ) u_mac_body (
                .x      (x_vec[i]),
                .w      (w_vec[i]),
                .cas_in (mac_cascade[i]),
                .cas_out(mac_cascade[i+1])
            );
        end
    endgenerate

    mac_tail #(
        .A_BIT(A_BIT),
        .W_BIT(W_BIT),
        .B_BIT(B_BIT)
    ) u_mac_tail (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (en),
        .dat_vld(dat_vld),
        .clr    (clr),
        .x      (x_vec[P_ICH-1]),
        .w      (w_vec[P_ICH-1]),
        .cas_in (mac_cascade[P_ICH-1]),
        .acc    (acc)
    );

endmodule
