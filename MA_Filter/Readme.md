A VHDL description of a moving average filter that averages N samples, where N is a power of 2.
The word length of the incoming samples is specified using the "generic" value "data_width" throughout the vhdl code "df_I_filter.vhd." The number of samples over which the average needs to be taken IS ALWAYS A POWER OF 2. This number is defined by "constant taps" in this VHDL code.

For example, " data_width: integer := 8 " means that the word length of the incoming samples is 8-bits, and "constant taps: integer := 4; " means that number of samples over which the average needs to taken is 4.