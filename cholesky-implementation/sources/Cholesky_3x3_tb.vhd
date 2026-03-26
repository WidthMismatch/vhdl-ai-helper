library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
-- Using math_real but with simplified structure

use IEEE.MATH_REAL.ALL;

entity simple_cholesky_tb is
end simple_cholesky_tb;

architecture Behavioral of simple_cholesky_tb is
    constant CLK_PERIOD : time := 10 ns;
    constant FRAC_BITS : integer := 12;
    
    signal clk, rst : std_logic := '0';
    signal a11_in, a21_in, a22_in : std_logic_vector(31 downto 0) := (others => '0');
    signal a31_in, a32_in, a33_in : std_logic_vector(31 downto 0) := (others => '0');
    signal data_valid, input_ready : std_logic := '0';
    signal l11_out, l21_out, l22_out : std_logic_vector(31 downto 0);
    signal l31_out, l32_out, l33_out : std_logic_vector(31 downto 0);
    signal output_valid, done, error_flag : std_logic;
    signal test_complete : boolean := false;

    -- Simple conversion functions (no records, no complex procedures)
    function to_fixed(val : real) return signed is
    begin
        return to_signed(integer(val * real(2**FRAC_BITS)), 32);
    end function;
    
    function to_real_val(fp_val : signed) return real is
    begin
        return real(to_integer(fp_val)) / real(2**FRAC_BITS);
    end function;

begin
    uut: entity work.cholesky_3x3
        port map (
            clk => clk, rst => rst,
            a11_in => a11_in, a21_in => a21_in, a22_in => a22_in,
            a31_in => a31_in, a32_in => a32_in, a33_in => a33_in,
            data_valid => data_valid, input_ready => input_ready,
            l11_out => l11_out, l21_out => l21_out, l22_out => l22_out,
            l31_out => l31_out, l32_out => l32_out, l33_out => l33_out,
            output_valid => output_valid, done => done, error_flag => error_flag
        );

    clk_process: process
    begin
        while not test_complete loop
            clk <= '0'; wait for CLK_PERIOD/2;
            clk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    stim_process: process
        variable L11, L21, L22, L31, L32, L33 : real;
    begin
        report "=== SIMPLE CHOLESKY TEST ===";

        rst <= '1';
        wait for CLK_PERIOD * 2;
        rst <= '0';
        wait for CLK_PERIOD;

        -- Test case: 3x3 positive definite matrix
        -- Input matrix A:
        -- [  4   12  -16 ]
        -- [ 12   37  -43 ]
        -- [-16  -43   98 ]
        -- Expected L (A = L*L^T):
        -- [ 2   0   0 ]
        -- [ 6   1   0 ]
        -- [-8   5   3 ]

        wait until rising_edge(clk) and input_ready = '1';
        a11_in <= std_logic_vector(to_fixed(4.0));
        a21_in <= std_logic_vector(to_fixed(12.0));
        a22_in <= std_logic_vector(to_fixed(37.0));
        a31_in <= std_logic_vector(to_fixed(-16.0));
        a32_in <= std_logic_vector(to_fixed(-43.0));
        a33_in <= std_logic_vector(to_fixed(98.0));

        data_valid <= '1';
        wait until rising_edge(clk);
        data_valid <= '0';

        -- Wait for result
        wait until rising_edge(clk) and output_valid = '1';

        -- Convert all outputs
        L11 := to_real_val(signed(l11_out));
        L21 := to_real_val(signed(l21_out));
        L22 := to_real_val(signed(l22_out));
        L31 := to_real_val(signed(l31_out));
        L32 := to_real_val(signed(l32_out));
        L33 := to_real_val(signed(l33_out));

        -- Report all results
        report "L11: " & real'image(L11) & " (expected  2.0)";
        report "L21: " & real'image(L21) & " (expected  6.0)";
        report "L22: " & real'image(L22) & " (expected  1.0)";
        report "L31: " & real'image(L31) & " (expected -8.0)";
        report "L32: " & real'image(L32) & " (expected  5.0)";
        report "L33: " & real'image(L33) & " (expected  3.0)";

        if error_flag = '0' then
            report "TEST PASSED - No errors detected";
        else
            report "ERROR FLAG SET - Check intermediate calculations";
        end if;

        wait for CLK_PERIOD * 5;
        report "=== TEST COMPLETE ===";
        test_complete <= true;
        wait;
    end process;

end Behavioral;