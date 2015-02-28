
# .xise, .gise: configuration for ISE in XML format
# .cmd_log: ISE log file of all commands launched
# .lso: just the name of the work library??
# .stx: empty file?
# _envsettings.html: all about the machine and the environment where ISE was launched?

# .ngr: Who produces that??
# It should be for RTL viewers.
# Or it's generated when doing incremental synthesis. Several NGR files can be used.

# _guide.ncd: WHO produces that??
# Linked to PlanAhead?

# _map.ngm: The NCD and NGM files produced by map are for guiding.

SRC = \
	../Lib/pattern_generator.vhd \
	../Lib/spi_slave_gpio.vhd \
	remote_sources/_/Lib/clk_binary_divider.vhd \
	remote_sources/_/Lib/FIFO.vhd \
	remote_sources/_/Lib/time_sync.vhd \
	clk_mult_100mhz.vhd \
	gpio_protocol_data_decode.vhd \
	gpio_spi_protocol.vhd \
	output.vhd \
	Top.vhd

.PHONY: default xst ngdbuild map par drc trce bitgen promgen

default: bitgen

xst: Top.ngc
ngdbuild: Top.ngd
map: Top_map.ncd
par: Top.ncd
bitgen: Top.bit
promgen: GPIO_PROM.mcs



# Logic synthesis with xst
# .xst is parameter file
# .syr is log file
# _xst.xrpt is XML report file for ISE
# .prj is list of HDL files
# .vhd => .ngc
Top.ngc: $(SRC) Top.prj Top.xst
	mkdir -p xst/projnav.tmp
	xst -intstyle ise -ifn Top.xst -ofn Top.syr

# Translate with ngdbuild
# .bld is log file
# .ucf is user constraints file
# _ngdbuild.xrpt is XML report file for ISE
# .ngc => .ngd
Top.ngd: Top.ngc GPIO.ucf Cmod_S6_master.ucf
	ngdbuild -intstyle ise -dd _ngo -nt timestamp -uc GPIO.ucf -uc Cmod_S6_master.ucf -p xc6slx4-cpg196-2 Top.ngc Top.ngd

# Map operation
# _map.map is log file
# _map.mrp is report file
# _map.xrpt is XML report file for ISE
# .ngd => _map.ncd .pcf
Top_map.ncd: Top.ngd
	map -intstyle ise -p xc6slx4-cpg196-2 -w -logic_opt off -ol high -t 1 -xt 0 -register_duplication off -r 4 -global_opt off -mt off -ir off -pr off -lc off -power off -o Top_map.ncd Top.ngd Top.pcf

# Palce and route with par
# .par is log file
# .xpi is status file for ISE?
# .pad is pad info to be viewed with other tools (like CSV but with separator | ?)
# _pad.csv is pad info in CSV format
# _pad.txt is pad info in text format
# .unroutes is another info file for unrouted paths
# _par.xrpt is XML report file for ISE
# _map.ncd .pcf => .ncd
Top.ncd: Top_map.ncd Top.pcf
	par -w -intstyle ise -ol high -mt off Top_map.ncd Top.ncd Top.pcf
	$(MAKE) drc

# Design Rule Check
# .tdr is report file
# .drc is log file
drc: Top.ncd Top.pcf
	drc -z Top.ncd Top.pcf

# .twr is log file
# .ptwx .twx is XML report for ISE?
# .ncd .pcf .ucf => .twx
trce:
	trce -intstyle ise -v 3 -s 2 -n 3 -fastpaths -xml Top.twx Top.ncd -o Top.twr Top.pcf -ucf GPIO.ucf -ucf Cmod_S6_master.ucf

# Bitstream generation with bitgen
# _bitgen.xwbt is XML status file for ISE
# .bgn is log file
# .ut is parameter file
# .ncd .pcf => .bit
Top.bit: Top.ut Top.ncd
	bitgen -intstyle ise -f Top.ut Top.ncd

# .cfi is status file for ISE?
# .prm is log file
# .bit => .mcs
GPIO_PROM.mcs: Top.bit
	promgen -w -p mcs -c FF -o ./GPIO_PROM -s 4096 -u 0000 ./top.bit -spi


