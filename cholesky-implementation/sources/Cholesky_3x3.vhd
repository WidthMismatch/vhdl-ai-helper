library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- XSIM-compatible version with inline arithmetic
-- Does NOT use package functions (they cause SIGSEGV)

entity cholesky_3x3 is
    generic (
        DATA_WIDTH : integer := 32;
        FRAC_BITS  : integer := 12
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- Parallel input channels
        a11_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a21_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a22_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a31_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a32_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        a33_in      : in  std_logic_vector(DATA_WIDTH-1 downto 0);

        data_valid  : in  std_logic;
        input_ready : out std_logic;

        -- Output matrix L
        l11_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l21_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l22_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l31_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l32_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);
        l33_out     : out std_logic_vector(DATA_WIDTH-1 downto 0);

        -- Control signals
        output_valid : out std_logic;
        done        : out std_logic;
        error_flag  : out std_logic
    );
end cholesky_3x3;

architecture Behavioral of cholesky_3x3 is
    -- FSM states
    type state_type is (IDLE,
                       CALC_L11, WAIT_L11,
                       CALC_L21_L31,
                       CALC_L22, WAIT_L22,
                       CALC_L32,
                       CALC_L33, WAIT_L33,
                       FINISH);
    signal state : state_type := IDLE;

    -- Matrix storage registers
    signal a11, a21, a22, a31, a32, a33 : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal l11, l21, l22, l31, l32, l33 : signed(DATA_WIDTH-1 downto 0) := (others => '0');

    -- Square root interface
    signal sqrt_start    : std_logic := '0';
    signal sqrt_x_in     : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal sqrt_result   : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal sqrt_done     : std_logic := '0';
    signal sqrt_busy     : std_logic := '0';

begin
    -- XSIM-compatible sqrt_newton
    sqrt_inst: entity work.sqrt_newton
        port map (
            clk => clk,
            start_rt => sqrt_start,
            x_in => sqrt_x_in,
            x_out => sqrt_result,
            done => sqrt_done
        );

    -- Main FSM process with INLINE arithmetic (no package functions)
    process(clk)
        -- Variables for inline fixed-point arithmetic
        variable temp_mult : signed(63 downto 0);
        variable temp_div : signed(63 downto 0);
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
                -- Default outputs
                output_valid <= '0';
                done <= '0';
                sqrt_start <= '0';

                case state is
                    when IDLE =>
                        if data_valid = '1' then
                            -- Latch all inputs in parallel
                            a11 <= signed(a11_in);
                            a21 <= signed(a21_in);
                            a22 <= signed(a22_in);
                            a31 <= signed(a31_in);
                            a32 <= signed(a32_in);
                            a33 <= signed(a33_in);
                            state <= CALC_L11;
                        end if;

                    when CALC_L11 =>
                        -- Start L11 calculation (sqrt(a11))
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
                        -- Calculate L21 and L31: l21 = a21/l11, l31 = a31/l11
                        -- Inline fixed-point division: (a * 4096) / b
                        temp_div := a21 * 4096;  -- Scale up
                        temp_div := temp_div / l11;  -- Divide
                        l21 <= temp_div(31 downto 0);  -- Extract result

                        temp_div := a31 * 4096;
                        temp_div := temp_div / l11;
                        l31 <= temp_div(31 downto 0);

                        state <= CALC_L22;

                    when CALC_L22 =>
                        -- Calculate L22: sqrt(a22 - l21²)
                        -- Inline fixed-point multiply: (a * b) >> 12
                        temp_mult := l21 * l21;
                        sqrt_x_in <= a22 - temp_mult(43 downto 12);
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
                        -- Calculate L32: (a32 - l31*l21) / l22
                        temp_mult := l31 * l21;
                        temp_div := (a32 - temp_mult(43 downto 12)) * 4096;
                        temp_div := temp_div / l22;
                        l32 <= temp_div(31 downto 0);

                        state <= CALC_L33;

                    when CALC_L33 =>
                        -- Calculate L33: sqrt(a33 - l31² - l32²)
                        -- Fixed: Compute both squares and subtract in single expression
                        temp_mult := l31 * l31;
                        temp_div := l32 * l32;
                        sqrt_x_in <= a33 - temp_mult(43 downto 12) - temp_div(43 downto 12);

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

    -- Combinational outputs
    input_ready <= '1' when state = IDLE else '0';

    -- Output assignments
    l11_out <= std_logic_vector(l11);
    l21_out <= std_logic_vector(l21);
    l22_out <= std_logic_vector(l22);
    l31_out <= std_logic_vector(l31);
    l32_out <= std_logic_vector(l32);
    l33_out <= std_logic_vector(l33);
end Behavioral;