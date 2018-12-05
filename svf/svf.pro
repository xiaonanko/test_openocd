!include( $$top_srcdir/common.pri ) {
  error( "Couldn't find the common.pri file!" )
}
!include( $$top_srcdir/src/src_common.pri ) {
  error( "Couldn't find the src_common.pri file!" )
}
TEMPLATE = lib
CONFIG += staticlib
unix {
  QMAKE_CXXFLAGS -= -Werror
}
win32 {
  QMAKE_CXXFLAGS -= /WX
}

SOURCES += svf.cc svf_info.cc openocd_config.cc jtag_svf_info.cc program_bin.cc

