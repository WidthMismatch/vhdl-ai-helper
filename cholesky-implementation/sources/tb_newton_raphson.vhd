library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

entity tb_sqrt_newton is
end tb_sqrt_newton;

architecture TB of tb_sqrt_newton is
    -- DUT signals
    signal clk      : std_logic := '0';
    signal rst      : std_logic := '0';
    signal start_rt : std_logic := '0';
    signal x_in     : signed(31 downto 0) := (others => '0');
    signal x_out    : signed(31 downto 0);
    signal done     : std_logic;

    -- Test control
    constant clk_period : time := 10 ns;
    constant SCALE      : integer := 4096;  -- Q20.12 format (2^12)
    constant MAX_ERROR  : real := 0.5;      -- Maximum acceptable error percentage

    signal test_running : boolean := true;
    signal tests_passed : integer := 0;
    signal tests_failed : integer := 0;

    -- Helper functions
    function to_fixed(x : real) return signed is
        variable val : integer;
    begin
        val := integer(x * real(SCALE));
        return to_signed(val, 32);
    end function;

    function to_real(x : signed) return real is
    begin
        return real(to_integer(x)) / real(SCALE);
    end function;

    -- Test procedure
    procedure test_sqrt(
        constant input_val  : in real;
        constant test_name  : in string;
        signal clk          : in std_logic;
        signal start_rt     : out std_logic;
        signal x_in         : out signed;
        signal x_out        : in signed;
        signal done         : in std_logic;
        signal tests_passed : inout integer;
        signal tests_failed : inout integer
    ) is
        variable actual     : real;
        variable expected   : real;
        variable error_pct  : real;
    begin
        report "========================================";
        report "Test: " & test_name;
        report "Input: " & real'image(input_val);

        -- Apply stimulus
        x_in <= to_fixed(input_val);
        start_rt <= '1';
        wait for clk_period;
        start_rt <= '0';

        -- Wait for completion
        wait until done = '1';
        wait for clk_period;

        -- Check result
        actual := to_real(x_out);
        expected := sqrt(input_val);

        -- Calculate error percentage (handle divide-by-zero for zero inputs)
        if expected = 0.0 then
            -- For zero input, check if output is also zero
            if actual = 0.0 then
                error_pct := 0.0;  -- Perfect match
            else
                error_pct := 100.0;  -- Output non-zero when it should be zero
            end if;
        else
            error_pct := abs((actual - expected) / expected) * 100.0;
        end if;

        report "  Output:   " & real'image(actual);
        report "  Expected: " & real'image(expected);
        report "  Error:    " & real'image(error_pct) & "%";

        if error_pct <= MAX_ERROR then
            report "  PASS" severity note;
            tests_passed <= tests_passed + 1;
        else
            report "  FAIL - Error exceeds " & real'image(MAX_ERROR) & "%" severity error;
            tests_failed <= tests_failed + 1;
        end if;

        wait for 2 * clk_period;
    end procedure;

begin
    -- Instantiate DUT
    DUT: entity work.sqrt_newton
        port map (
            clk      => clk,
            rst      => rst,
            start_rt => start_rt,
            x_in     => x_in,
            x_out    => x_out,
            done     => done
        );

    -- Clock generation
    clk_process: process
    begin
        while test_running loop
            clk <= '0';
            wait for clk_period / 2;
            clk <= '1';
            wait for clk_period / 2;
        end loop;
        wait;
    end process;

    -- Main test stimulus
    stim_proc: process
        variable val_int : integer;
    begin
        -- Apply reset
        rst <= '1';
        wait for 20 ns;
        rst <= '0';
        wait for 30 ns;

        report "========================================";
        report "Starting Comprehensive Square Root Tests";
        report "========================================";

        -- Test Suite 1: Directed Tests (specific important cases)
        report "";
        report "TEST SUITE 1: Directed Tests";
        report "========================================";

        -- Test 1.1: Zero input
        test_sqrt(0.0, "Zero Input", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);

        -- Test 1.2: Perfect squares
        test_sqrt(1.0, "Perfect Square: 1", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(4.0, "Perfect Square: 4", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(9.0, "Perfect Square: 9", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(16.0, "Perfect Square: 16", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(25.0, "Perfect Square: 25", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(100.0, "Perfect Square: 100", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);

        -- Test 1.3: Decimal values
        test_sqrt(0.25, "Decimal: 0.25", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(0.5, "Decimal: 0.5", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(2.25, "Decimal: 2.25", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(3.5, "Decimal: 3.5", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);

        -- Test 1.4: Non-perfect squares
        test_sqrt(2.0, "Non-Perfect Square: 2", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(3.0, "Non-Perfect Square: 3", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(5.0, "Non-Perfect Square: 5", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(10.0, "Non-Perfect Square: 10", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(50.0, "Non-Perfect Square: 50", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);

        -- Test Suite 2: Parametric Sweep
        report "";
        report "TEST SUITE 2: Parametric Sweep (5 to 100, step 5)";
        report "========================================";

        for val_int in 1 to 20 loop
            test_sqrt(
                real(val_int * 5),
                "Sweep Test: " & integer'image(val_int * 5),
                clk, start_rt, x_in, x_out, done, tests_passed, tests_failed
            );
        end loop;

        -- Test Suite 3: Edge Cases
        report "";
        report "TEST SUITE 3: Edge Cases";
        report "========================================";

        test_sqrt(0.01, "Very Small: 0.01", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(0.1, "Small: 0.1", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(1000.0, "Large: 1000", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);
        test_sqrt(10000.0, "Very Large: 10000", clk, start_rt, x_in, x_out, done, tests_passed, tests_failed);

        -- Final Report
        wait for 100 ns;
        report "";
        report "========================================";
        report "TEST SUMMARY";
        report "========================================";
        report "Tests Passed: " & integer'image(tests_passed);
        report "Tests Failed: " & integer'image(tests_failed);
        report "Total Tests:  " & integer'image(tests_passed + tests_failed);

        if tests_failed = 0 then
            report "ALL TESTS PASSED!" severity note;
        else
            report "SOME TESTS FAILED!" severity error;
        end if;
        report "========================================";

        test_running <= false;
        wait;
    end process;

end TB;