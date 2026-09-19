@echo off
rem Build the LR2021 driver sandbox + rtl_433 phase-3 decode with MSVC.
rem Run from the sim dir as:  MSYS_NO_PATHCONV=1 cmd.exe /c build_msvc2.bat
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" > build_env.log 2>&1
where cl >> build_env.log 2>&1

rem Vendored upstream rtl_433: /FI force-includes the static-arena shim
rem (mirror of the firmware build's -include).  /W0 for the vendored set
rem (upstream code, upstream style; the relaxed-warning list in the
rem Makefile has no MSVC equivalent that is worth maintaining) — the
rem glue + sim sources still compile with default warnings.
rem MSVC resolves a relative /FI path against the SOURCE file's dir, so
rem resolve to an absolute path first.
set RTL=..\..\zephcore\src\rtl433
for %%I in (%RTL%) do set RTLABS=%%~fI

cl /nologo /std:c17 /W0 /DSNF_RTL433_HOST /I%RTLABS% /FI%RTLABS%\rtl433_mem.h /c ^
   %RTLABS%\bitbuffer.c %RTLABS%\bit_util.c %RTLABS%\data.c %RTLABS%\abuf.c ^
   %RTLABS%\list.c %RTLABS%\decoder_util.c %RTLABS%\pulse_slicer.c ^
   %RTLABS%\devices\acurite.c %RTLABS%\devices\lacrosse_tx141x.c ^
   %RTLABS%\devices\oregon_scientific.c %RTLABS%\devices\oregon_scientific_v1.c ^
   %RTLABS%\devices\fineoffset.c %RTLABS%\devices\prologue.c ^
   %RTLABS%\devices\hideki.c > build_cl_vendor.log 2>&1
set VENDOR_EXIT=%ERRORLEVEL%

cl /nologo /std:c17 /W3 /DSNF_RTL433_HOST /I%RTLABS% /c ^
   stub_lr2021.c driver_under_test.c test_lr2021_driver.c ^
   tdm_wedge_model.c test_tdm_wedge.c test_rtl433.c %RTLABS%\sniffer_rtl433.c ^
   ..\..\zephcore\src\sniffer_wmbus_parse.c ^
   > build_cl.log 2>&1
set SIM_EXIT=%ERRORLEVEL%

cl /nologo /Fe:lr2021_sim_tests.exe *.obj > build_cl_link.log 2>&1
echo CL_EXIT=%ERRORLEVEL% VENDOR_EXIT=%VENDOR_EXIT% SIM_EXIT=%SIM_EXIT%
