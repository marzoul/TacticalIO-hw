
# PlanAhead Launch Script for Post PAR Floorplanning, created by Project Navigator

create_project -name AIO -dir "D:/git/opentekhnia/Tio/FPGA/AIO/planAhead_run_1" -part xc6slx4cpg196-2
set srcset [get_property srcset [current_run -impl]]
set_property design_mode GateLvl $srcset
set_property edif_top_file "D:/git/opentekhnia/Tio/FPGA/AIO/Top.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {D:/git/opentekhnia/Tio/FPGA/AIO} }
set_property target_constrs_file "D:/git/opentekhnia/Tio/FPGA/AIO/AIO.ucf" [current_fileset -constrset]
add_files [list {Cmod_S6_master.ucf}] -fileset [get_property constrset [current_run]]
add_files [list {AIO.ucf}] -fileset [get_property constrset [current_run]]
link_design
read_xdl -file "D:/git/opentekhnia/Tio/FPGA/AIO/Top.ncd"
if {[catch {read_twx -name results_1 -file "D:/git/opentekhnia/Tio/FPGA/AIO/Top.twx"} eInfo]} {
   puts "WARNING: there was a problem importing \"D:/git/opentekhnia/Tio/FPGA/AIO/Top.twx\": $eInfo"
}
