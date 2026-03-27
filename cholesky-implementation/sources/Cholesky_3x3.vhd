library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cholesky_3x3 is
    generic (
        DATA_WIDTH : integer := 32;
        FRAC_BITS  : integer := 12
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        a11_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a21_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a22_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a31_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a32_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a33_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        data_valid  : in  std_logic;
        input_ready : out std_logic;

        l11_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l21_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l22_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l31_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l32_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l33_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);

        output_valid : out std_logic;
        done        : out std_logic;
        error_flag  : out std_logic
    );
end cholesky_3x3;

architecture Behavioral of cholesky_3x3 is
    type state_type is (IDLE,
                       CALC_L11, WAIT_L11,
                       CALC_L21_L31,
                       CALC_L22, WAIT_L22,
                       CALC_L32,
                       CALC_L33, WAIT_L33,
                       FINISH);
    signal state : state_type := IDLE;

    signal a11, a21, a22, a31, a32, a33 : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal l11, l21, l22, l31, l32, l33 : signed(DATA_WIDTH-1 downto 0) := (others => '0');

    signal sqrt_start    : std_logic := '0';
    signal sqrt_x_in     : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal sqrt_result   : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal sqrt_done     : std_logic := '0';
    signal sqrt_busy     : std_logic := '0';

begin
   sqrt_inst: entity work.sqrt_newton
        port map (
            clk => clk,
            rst => rst,           -- <-- Added the reset port mapping here
            start_rt => sqrt_start,
            x_in => sqrt_x_in,
            x_out => sqrt_result,
            done => sqrt_done
        );
    process(clk)
        -- Deconstructed variables to prevent Vivado type-resolution errors
        variable v_mult     : signed(63 downto 0);
        variable v_mult2    : signed(63 downto 0);
        variable v_div      : signed(63 downto 0);
        variable v_slice    : signed(31 downto 0);
        variable v_slice2   : signed(31 downto 0);
        variable v_op1_64   : signed(63 downto 0);
        variable v_op2_64   : signed(63 downto 0);
        variable v_sub_64   : signed(63 downto 0);
        variable v_shift_64 : signed(63 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state <= IDLE;
                output_valid <= '0';
                done <= '0';
                error_flag <= '0';
                sqrt_start <= '0';
                sqrt_busy <= '0';
            else
                output_valid <= '0';
                done <= '0';
                sqrt_start <= '0';

                case state is
                    when IDLE =>
                        if data_valid = '1' then
                            a11 <= signed(a11_in);
                            a21 <= signed(a21_in);
                            a22 <= signed(a22_in);
                            a31 <= signed(a31_in);
                            a32 <= signed(a32_in);
                            a33 <= signed(a33_in);
                            state <= CALC_L11;
                        end if;

                    when CALC_L11 =>
                        sqrt_x_in <= a11;
                        sqrt_start <= '1';
                        sqrt_busy <= '1';
                        state <= WAIT_L11;

                    when WAIT_L11 =>
                        sqrt_start <= '0';
                        if sqrt_done = '1' then
                            l11 <= sqrt_result;
                            if sqrt_result <= 0 then
                                error_flag <= '1';
                                state <= FINISH;
                            else
                                state <= CALC_L21_L31;
                            end if;
                            sqrt_busy <= '0';
                        end if;

                    when CALC_L21_L31 =>
                        -- L21 Calculation mapped explicitly step-by-step
                        v_op1_64 := resize(a21, 64);
                        v_op2_64 := resize(l11, 64);
                        v_shift_64 := shift_left(v_op1_64, 12);
                        v_div := v_shift_64 / v_op2_64;
                        l21 <= v_div(31 downto 0);

                        -- L31 Calculation
                        v_op1_64 := resize(a31, 64);
                        v_shift_64 := shift_left(v_op1_64, 12);
                        v_div := v_shift_64 / v_op2_64;
                        l31 <= v_div(31 downto 0);

                        state <= CALC_L22;

                    when CALC_L22 =>
                        v_mult := l21 * l21;
                        v_slice := v_mult(43 downto 12);
                        sqrt_x_in <= a22 - v_slice;
                        
                        sqrt_start <= '1';
                        sqrt_busy <= '1';
                        state <= WAIT_L22;

                    when WAIT_L22 =>
                        sqrt_start <= '0';
                        if sqrt_done = '1' then
                            l22 <= sqrt_result;
                            if sqrt_result <= 0 then
                                error_flag <= '1';
                                state <= FINISH;
                            else
                                state <= CALC_L32;
                            end if;
                            sqrt_busy <= '0';
                        end if;

                    when CALC_L32 =>
                        -- Step-by-step breakdown of: (a32 - l31*l21) / l22
                        v_mult := l31 * l21;
                        v_slice := v_mult(43 downto 12);
                        v_op1_64 := resize(a32, 64);
                        v_op2_64 := resize(v_slice, 64);
                        v_sub_64 := v_op1_64 - v_op2_64;
                        v_shift_64 := shift_left(v_sub_64, 12);
                        v_op1_64 := resize(l22, 64);
                        
                        v_div := v_shift_64 / v_op1_64;
                        l32 <= v_div(31 downto 0);

                        state <= CALC_L33;

                    when CALC_L33 =>
                        v_mult := l31 * l31;
                        v_mult2 := l32 * l32;
                        v_slice := v_mult(43 downto 12);
                        v_slice2 := v_mult2(43 downto 12);
                        
                        sqrt_x_in <= a33 - v_slice - v_slice2;

                        sqrt_start <= '1';
                        sqrt_busy <= '1';
                        state <= WAIT_L33;

                    when WAIT_L33 =>
                        sqrt_start <= '0';
                        if sqrt_done = '1' then
                            l33 <= sqrt_result;
                            if sqrt_result <= 0 then
                                error_flag <= '1';
                            end if;
                            sqrt_busy <= '0';
                            state <= FINISH;
                        end if;

                    when FINISH =>
                        output_valid <= '1';
                        done <= '1';
                        state <= IDLE;
                end case;
            end if;
        end if;
    end process;

    input_ready <= '1' when state = IDLE else '0';

    l11_out <= std_logic_vector(l11);
    l21_out <= std_logic_vector(l21);
    l22_out <= std_logic_vector(l22);
    l31_out <= std_logic_vector(l31);
    l32_out <= std_logic_vector(l32);
    l33_out <= std_logic_vector(l33);

end Behavioral;