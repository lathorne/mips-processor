@echo off
set xv_path=C:\\Xilinx\\Vivado\\2016.4\\bin
call %xv_path%/xsim tb_generic_pipeline_behav -key {Behavioral:sim_1:Functional:tb_generic_pipeline} -tclbatch tb_generic_pipeline.tcl -view J:/Documents/323_lab9/tb_final_pipeline_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
