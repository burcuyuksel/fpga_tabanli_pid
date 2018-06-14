library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
USE ieee.std_logic_unsigned.all;
entity adc_serial_control is
generic(
  CLK_DIV               : integer := 100 ;  -- input clock divider to generate output serial clock; o_sclk frequency = i_clk/(CLK_DIV)
      sys_clk         : INTEGER := 50_000_000; --system clock frequency in Hz
      pwm_freq        : INTEGER := 15_000;    --PWM switching frequency in Hz
      bits_resolution : INTEGER := 12;          --bits of resolution setting the duty cycle
      phases          : INTEGER := 1);         --number of output pwms and phases
port (
  i_clk                       : in  std_logic;
  i_rstb                      : in  std_logic;
  i_conv_ena                  : in  std_logic;  -- enable ADC convesion
  i_adc_ch                    : in  std_logic_vector(2 downto 0);  -- ADC channel 0-7
  o_adc_data_valid            : out std_logic;  -- conversion valid pulse
  o_adc_ch                    : out std_logic_vector(2 downto 0);  -- ADC converted channel
  o_adc_data                  : inout std_logic_vector(11 downto 0); -- adc parallel data  
-- ADC serial interface
  o_sclk                      : out std_logic;
  o_ss                        : out std_logic;
  o_mosi                      : out std_logic;
  i_miso                      : in  std_logic;
  
   
      reset_n   : IN  STD_LOGIC;                                    --asynchronous reset
      ena       : IN  STD_LOGIC;                                    --latches in new duty cycle
      pwm_out   : OUT STD_LOGIC_VECTOR(phases-1 DOWNTO 0);          --pwm outputs
      pwm_n_out : OUT STD_LOGIC_VECTOR(phases-1 DOWNTO 0));         --pwm inverse outputs
end adc_serial_control;
architecture rtl of adc_serial_control is
constant C_N                     : integer := 16;
signal r_counter_clock            : integer range 0 to CLK_DIV;
signal r_sclk_rise                : std_logic;
signal r_sclk_fall                : std_logic;
signal r_counter_clock_ena        : std_logic;
signal r_counter_data             : integer range 0 to C_N-1;
signal r_tc_counter_data          : std_logic;
signal r_conversion_running       : std_logic;  -- enable serial data protocol 
signal r_miso                     : std_logic;
signal r_conv_ena                 : std_logic;  -- enable ADC convesion
signal r_adc_ch                   : std_logic_vector(2 downto 0);  -- ADC converted channel
signal r_adc_data                 : std_logic_vector(11 downto 0); -- adc parallel data 

 CONSTANT  period     :  INTEGER := sys_clk/pwm_freq;                      --number of clocks in one pwm period
  TYPE counters IS ARRAY (0 TO phases-1) OF INTEGER RANGE 0 TO period - 1;  --data type for array of period counters
  SIGNAL  count        :  counters := (OTHERS => 0);                        --array of period counters
  SIGNAL   half_duty_new  :  INTEGER RANGE 0 TO period/2 := 0;              --number of clocks in 1/2 duty cycle
  TYPE half_duties IS ARRAY (0 TO phases-1) OF INTEGER RANGE 0 TO period/2; --data type for array of half duty values
  SIGNAL  half_duty    :  half_duties := (OTHERS => 0);    
begin
--------------------------------------------------------------------
-- FSM
p_conversion_control : process(i_clk,i_rstb)
begin
  if(i_rstb='0') then
    r_conv_ena             <= '0';
    r_conversion_running   <= '0';
    r_counter_clock_ena    <= '0';
  elsif(rising_edge(i_clk)) then
    r_conv_ena             <= i_conv_ena;
    if(r_conv_ena='1') then
      r_conversion_running   <= '1';
    elsif(r_conv_ena='0') and (r_tc_counter_data='1') then -- terminate current conversion
      r_conversion_running   <= '0';
    end if;
    
    r_counter_clock_ena    <= r_conversion_running;  -- enable clock divider
  end if;
end process p_conversion_control;
p_counter_data : process(i_clk,i_rstb)
begin
  if(i_rstb='0') then
    r_counter_data       <= 0;
    r_tc_counter_data    <= '0';
  elsif(rising_edge(i_clk)) then
    if(r_counter_clock_ena='1') then
      if(r_sclk_rise='1') then  -- count data @ o_sclk rising edge
        if(r_counter_data<C_N-1) then
          r_counter_data     <= r_counter_data + 1;
          r_tc_counter_data  <= '0';
        else
          r_counter_data     <= 0;
          r_tc_counter_data  <= '1';
        end if;
      else
        r_tc_counter_data  <= '0';
      end if;
    else
      r_counter_data     <= 0;
      r_tc_counter_data  <= '0';
    end if;
  end if;
end process p_counter_data;
-- Serial Input Process
p_serial_input : process(i_clk,i_rstb)
begin
  if(i_rstb='0') then
    r_miso               <= '0';
    r_adc_ch             <= (others=>'0');
    r_adc_data           <= (others=>'0');
  elsif(rising_edge(i_clk)) then
    r_miso               <= i_miso;
    
    if(r_tc_counter_data='1') then
      r_adc_ch             <= i_adc_ch; -- strobe new
    end if;
    case r_counter_data is
      when  4  => r_adc_data(11)  <= r_miso;
      when  5  => r_adc_data(10)  <= r_miso;
      when  6  => r_adc_data( 9)  <= r_miso;
      when  7  => r_adc_data( 8)  <= r_miso;
      when  8  => r_adc_data( 7)  <= r_miso;
      when  9  => r_adc_data( 6)  <= r_miso;
      when 10  => r_adc_data( 5)  <= r_miso;
      when 11  => r_adc_data( 4)  <= r_miso;
      when 12  => r_adc_data( 3)  <= r_miso;
      when 13  => r_adc_data( 2)  <= r_miso;
      when 14  => r_adc_data( 1)  <= r_miso;
      when 15  => r_adc_data( 0)  <= r_miso;
      when others => NULL;
    end case;
  end if;
end process p_serial_input;
-- SERIAL Output process
p_serial_output : process(i_clk,i_rstb)
begin
  if(i_rstb='0') then
    o_ss                 <= '1';
    o_mosi               <= '1';
    o_sclk               <= '1';
    o_adc_data_valid     <= '0';
    o_adc_ch             <= (others=>'0');
    o_adc_data           <= (others=>'0');
  elsif(rising_edge(i_clk)) then
    o_ss                 <= not r_conversion_running;
    if(r_tc_counter_data='1') then
      o_adc_ch             <= r_adc_ch; -- update current conversion
      o_adc_data           <= r_adc_data;
    end if;
    o_adc_data_valid     <= r_tc_counter_data;
    if(r_counter_clock_ena='1') then  -- sclk = '1' by default 
      if(r_sclk_rise='1') then
        o_sclk   <= '1';
      elsif(r_sclk_fall='1') then
        o_sclk   <= '0';
      end if;
    else
      o_sclk   <= '1';
    end if;
  
    if(r_sclk_fall='1') then
      case r_counter_data is
        when  2  => o_mosi <= r_adc_ch(2);
        when  3  => o_mosi <= r_adc_ch(1);
        when  4  => o_mosi <= r_adc_ch(0);
        when others => NULL;
      end case;
    end if;
  end if;
end process p_serial_output;
-- CLOCK divider
p_counter_clock : process(i_clk,i_rstb)
begin
  if(i_rstb='0') then
    r_counter_clock            <= 0;
    r_sclk_rise                <= '0';
    r_sclk_fall                <= '0';
  elsif(rising_edge(i_clk)) then
    if(r_counter_clock_ena='1') then 
      if(r_counter_clock=(CLK_DIV/2)-1) then  -- firse edge = fall
        r_counter_clock            <= r_counter_clock + 1;
        r_sclk_rise                <= '0';
        r_sclk_fall                <= '1';
      elsif(r_counter_clock=(CLK_DIV-1)) then
        r_counter_clock            <= 0;
        r_sclk_rise                <= '1';
        r_sclk_fall                <= '0';
      else
        r_counter_clock            <= r_counter_clock + 1;
        r_sclk_rise                <= '0';
        r_sclk_fall                <= '0';
      end if;
    else
      r_counter_clock            <= 0;
      r_sclk_rise                <= '0';
      r_sclk_fall                <= '0';
    end if;
  end if;
end process p_counter_clock;

 pwm : PROCESS(i_clk, reset_n)
  BEGIN
    IF(reset_n = '0') THEN                                                 --asynchronous reset
      count <= (OTHERS => 0);                                                --clear counter
      pwm_out <= (OTHERS => '0');                                            --clear pwm outputs
      pwm_n_out <= (OTHERS => '0');                                          --clear pwm inverse outputs
    ELSIF(i_clk'EVENT AND i_clk = '1') THEN                                      --rising system clock edge
      IF(ena = '1') THEN                                                   --latch in new duty cycle
        half_duty_new <= conv_integer(o_adc_data)*period/(2**bits_resolution)/2;   --determine clocks in 1/2 duty cycle
      END IF;
      FOR i IN 0 to phases-1 LOOP                                            --create a counter for each phase
        IF(count(0) = period - 1 - i*period/phases) THEN                       --end of period reached
          count(i) <= 0;                                                         --reset counter
          half_duty(i) <= half_duty_new;                                         --set most recent duty cycle value
        ELSE                                                                   --end of period not reached
          count(i) <= count(i) + 1;                                              --increment counter
        END IF;
      END LOOP;
      FOR i IN 0 to phases-1 LOOP                                            --control outputs for each phase
        IF(count(i) = half_duty(i)) THEN                                       --phase's falling edge reached
          pwm_out(i) <= '0';                                                     --deassert the pwm output
          pwm_n_out(i) <= '1';                                                   --assert the pwm inverse output
        ELSIF(count(i) = period - half_duty(i)) THEN                           --phase's rising edge reached
          pwm_out(i) <= '1';                                                     --assert the pwm output
          pwm_n_out(i) <= '0';                                                   --deassert the pwm inverse output
        END IF;
      END LOOP;
    END IF;
  END PROCESS pwm;
  end rtl;