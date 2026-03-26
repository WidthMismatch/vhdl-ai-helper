library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sqrt_newton is
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;  -- Synchronous reset (active high)
    start_rt : in  std_logic;
    x_in     : in  signed(31 downto 0);  -- Q20.12
    x_out    : out signed(31 downto 0);  -- Q20.12
    done     : out std_logic
  );
end entity;

architecture Behavioral of sqrt_newton is
  type state_type is (IDLE, INIT, ITERATE, ZERO, FINISH);
  signal state : state_type := IDLE;
  
  signal x_current : signed(31 downto 0);
  signal x_next    : signed(31 downto 0);
  signal iteration : integer range 0 to 12;
  signal x_input   : signed(31 downto 0);

  constant Q : integer := 12;
  constant ITERATIONS : integer := 12;  -- 12 iterations provide sufficient precision for Q20.12
  constant HALF : signed(31 downto 0) := to_signed(2048, 32); -- 0.5 in Q20.12
  constant TOLERANCE : signed(31 downto 0) := to_signed(4, 32); -- Convergence tolerance (0.1%)
  constant MIN_DIVISOR : signed(31 downto 0) := to_signed(4, 32); -- Minimum safe divisor value
  constant ONE_Q : signed(31 downto 0) := to_signed(4096, 32); -- 1.0 in Q20.12
  constant HUNDRED_Q : signed(31 downto 0) := to_signed(409600, 32); -- 100.0 in Q20.12
  
begin
  process(clk)
    variable temp_div : signed(63 downto 0);
    variable temp_sum : signed(31 downto 0);
    variable temp_mul : signed(63 downto 0);
    variable delta    : signed(31 downto 0);  -- For convergence check
    variable div_result : signed(31 downto 0); -- Saturated division result
    variable safe_divisor : signed(31 downto 0); -- Protected divisor
    variable x_next_var : signed(31 downto 0); -- Next iteration value (variable for immediate use)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        -- Synchronous reset
        state <= IDLE;
        done <= '0';
        x_out <= (others => '0');
        x_current <= (others => '0');
        x_next <= (others => '0');
        iteration <= 0;
        x_input <= (others => '0');
      else
        -- Default output assignments
        done <= '0';

        case state is
          when IDLE =>
            x_out <= (others => '0');
            if start_rt = '1' then
              x_input   <= x_in;
              iteration <= 0;
              -- If input is zero, skip directly to ZERO state
              if x_in = to_signed(0, 32) then
                state <= ZERO;
              else
                state <= INIT;
              end if;
            end if;

          when INIT =>
            -- Adaptive initial guess based on input magnitude
            -- For small inputs (< 1.0), use input itself
            -- For large inputs (> 100), use input / 4
            -- For normal range, use input / 2
            if x_input < ONE_Q then
              -- Input < 1.0: use input as initial guess
              x_current <= x_input;
            elsif x_input > HUNDRED_Q then
              -- Input > 100: use input / 4
              x_current <= shift_right(x_input, 2);
            else
              -- Normal range: use input / 2
              x_current <= shift_right(x_input, 1);
            end if;
            state <= ITERATE;
          
          when ITERATE =>
            -- Newton-Raphson iteration with overflow protection
            temp_div := shift_left(resize(x_input, 64), Q);

            -- Prevent division by very small numbers using variable
            safe_divisor := x_current;
            if abs(safe_divisor) < MIN_DIVISOR then
              safe_divisor := MIN_DIVISOR;
            end if;

            -- Perform division with protected divisor
            temp_div := temp_div / safe_divisor;

            -- Saturate division result to 32-bit range before adding
            if temp_div > to_signed(2147483647, 64) then
              div_result := to_signed(2147483647, 32);
            elsif temp_div < to_signed(-2147483648, 64) then
              div_result := to_signed(-2147483648, 32);
            else
              div_result := temp_div(31 downto 0);
            end if;

            -- Add current value to division result
            temp_sum := x_current + div_result;

            -- Multiply by 0.5 and shift back to Q20.12 format
            temp_mul := temp_sum * HALF;
            x_next_var := shift_right(temp_mul, Q)(31 downto 0);

            -- Check for convergence using variables (no signal delay issues)
            delta := abs(x_next_var - x_current);

            -- Update signals for next iteration
            x_current <= x_next_var;
            x_next <= x_next_var;
            iteration <= iteration + 1;

            -- Determine if we should finish on the NEXT cycle
            if delta < TOLERANCE or iteration >= ITERATIONS - 1 then
              -- Will finish after this iteration (convergence or max iterations reached)
              state <= FINISH;
            end if;

          when ZERO =>
            -- Handle x_in = 0 separately
            x_out <= (others => '0');
            done  <= '1';
            if start_rt = '0' then
              state <= IDLE;
            end if;

          when FINISH =>
            x_out <= x_current;
            done  <= '1';
            if start_rt = '0' then
              state <= IDLE;
            end if;

          when others =>
            state <= IDLE;
        end case;
      end if;
    end if;
  end process;
end Behavioral;