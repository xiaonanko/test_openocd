/***************************************************************************
 *    Copyright (C) 2009 by Simon Qian                                     *
 *    SimonQian@SimonQian.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* The specification for SVF is available here:
 * http://www.asset-intertech.com/support/svf.pdf
 * Below, this document is refered to as the "SVF spec".
 *
 * The specification for XSVF is available here:
 * http://www.xilinx.com/support/documentation/application_notes/xapp503.pdf
 * Below, this document is refered to as the "XSVF spec".
 */

#include "config.h"
#include "jtag/jtag.h"
#include "jtag/interface.h"
#include "svf_info.h"
#include "svf.h"
#include "helper/time_support.h"
#include <time.h>

int g_svf_ret;
static bool g_svf_nil = false;
const int SvfCommand::kCheckTdoMaxSize_ = 1024;
const int SvfCommand::kMaxBufferSizeToCommit_ = 1024*1024;

const char *SvfCommand::svf_command_name[14] = {
  "ENDDR",
  "ENDIR",
  "FREQUENCY",
  "HDR",
  "HIR",
  "PIO",
  "PIOMAP",
  "RUNTEST",
  "SDR",
  "SIR",
  "STATE",
  "TDR",
  "TIR",
  "TRST"
};

const char * SvfCommand::svf_trst_mode_name[4] = {
  "ON",
  "OFF",
  "Z",
  "ABSENT"
};

/*
 * These paths are from the SVF specification for the STATE command, to be
 * used when the STATE command only includes the final state.  The first
 * element of the path is the "from" (current) state, and the last one is
 * the "to" (target) state.
 *
 * All specified paths are the shortest ones in the JTAG spec, and are thus
 * not (!!) exact matches for the paths used elsewhere in OpenOCD.  Note
 * that PAUSE-to-PAUSE transitions all go through UPDATE and then CAPTURE,
 * which has specific effects on the various registers; they are not NOPs.
 *
 * Paths to RESET are disabled here.  As elsewhere in OpenOCD, and in XSVF
 * and many SVF implementations, we don't want to risk missing that state.
 * To get to RESET, always we ignore the current state.
 */
const struct SvfCommand::svf_statemove SvfCommand::svf_statemoves[12] = {
  /* from      to        num_of_moves,  paths[8] */
/*  {TAP_RESET,    TAP_RESET,    1,        {TAP_RESET}}, */
  {TAP_RESET,    TAP_IDLE,    2,        {TAP_RESET, TAP_IDLE} },
  {TAP_RESET,    TAP_DRPAUSE,  6,        {TAP_RESET, TAP_IDLE, TAP_DRSELECT,
                            TAP_DRCAPTURE, TAP_DREXIT1, TAP_DRPAUSE} },
  {TAP_RESET,    TAP_IRPAUSE,  7,        {TAP_RESET, TAP_IDLE, TAP_DRSELECT,
                            TAP_IRSELECT, TAP_IRCAPTURE,
                            TAP_IREXIT1, TAP_IRPAUSE} },

/*  {TAP_IDLE,    TAP_RESET,    4,        {TAP_IDLE,
 * TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
  {TAP_IDLE,    TAP_IDLE,    1,        {TAP_IDLE} },
  {TAP_IDLE,    TAP_DRPAUSE,  5,        {TAP_IDLE, TAP_DRSELECT, TAP_DRCAPTURE,
                            TAP_DREXIT1, TAP_DRPAUSE} },
  {TAP_IDLE,    TAP_IRPAUSE,  6,        {TAP_IDLE, TAP_DRSELECT, TAP_IRSELECT,
                            TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} },

/*  {TAP_DRPAUSE,  TAP_RESET,    6,        {TAP_DRPAUSE,
 * TAP_DREXIT2, TAP_DRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
  {TAP_DRPAUSE,  TAP_IDLE,    4,        {TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
                            TAP_IDLE} },
  {TAP_DRPAUSE,  TAP_DRPAUSE,  7,        {TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
                            TAP_DRSELECT, TAP_DRCAPTURE,
                            TAP_DREXIT1, TAP_DRPAUSE} },
  {TAP_DRPAUSE,  TAP_IRPAUSE,  8,        {TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
                            TAP_DRSELECT, TAP_IRSELECT,
                            TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} },

/*  {TAP_IRPAUSE,  TAP_RESET,    6,        {TAP_IRPAUSE,
 * TAP_IREXIT2, TAP_IRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
  {TAP_IRPAUSE,  TAP_IDLE,    4,        {TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
                            TAP_IDLE} },
  {TAP_IRPAUSE,  TAP_DRPAUSE,  7,        {TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
                            TAP_DRSELECT, TAP_DRCAPTURE,
                            TAP_DREXIT1, TAP_DRPAUSE} },
  {TAP_IRPAUSE,  TAP_IRPAUSE,  8,        {TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
                            TAP_DRSELECT, TAP_IRSELECT,
                            TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} }
};

/*
 * macro is used to print the svf hex buffer at desired debug level
 * DEBUG, INFO, ERROR, USER
 */
#define SVF_BUF_LOG(_lvl, _buf, _nbits, _desc)              \
  svf_hexbuf_print(LOG_LVL_##_lvl ,  __FILE__, __LINE__, __FUNCTION__, _buf, _nbits, _desc)

void SvfCommand::svf_hexbuf_print(int dbg_lvl, const char *file, unsigned line,
               const char *function, const uint8_t *buf,
               int bit_len, const char *desc)
{
  int j, len = 0;
  int byte_len = DIV_ROUND_UP(bit_len, 8);
  int msbits = bit_len % 8;

  /* allocate 2 bytes per hex digit */
  char *prbuf = (char*)malloc((byte_len * 2) + 5);
  if (!prbuf)
    return;

  /* print correct number of bytes, mask excess bits where applicable */
  uint8_t msb = buf[byte_len - 1] & (msbits ? (1 << msbits) - 1 : 0xff);
  len = sprintf(prbuf, msbits <= 4 ? "0x%01x" : "0x%02x", msb);
  for (j = byte_len - 2; j >= 0; j--)
    len += sprintf(prbuf + len, "%02x", buf[j]);

  log_printf_lf(static_cast<log_levels>(dbg_lvl), file, line, function, "%8s = %s", desc ? desc : " ", prbuf);

  free(prbuf);
}

SvfCommand::SvfCommand() {
  /* init */
  svf_line_number = 0;
  svf_command_buffer_size = 0;

  procise_flow_ = false;

  check_tdo_para_index_ = 0;
  check_tdo_vec_.resize(kCheckTdoMaxSize_);
  svf_buffer_index = 0;
  /* parse command line */
  svf_quiet = 0;
  svf_progress_enabled = 0;
  ignore_error_ = false;
  svf_ignore_error = 0;

  svf_read_line=NULL;
  svf_read_line_size=0;
  svf_command_buffer=NULL;
  svf_tdi_buffer=NULL;
  svf_tdo_buffer=NULL;
  svf_mask_buffer=NULL;
  svf_buffer_size=0;

  /* Targetting particular tap */
  svf_tap_is_specified=0;

  /* Progress Indicator */
  svf_total_lines=0;
  svf_percentage=0;
  svf_last_printed_percentage = -1;
}

SvfCommand::~SvfCommand() {
  if (svf_command_buffer) {
    free(svf_command_buffer);
    svf_command_buffer = NULL;
    svf_command_buffer_size = 0;
  }
  if (svf_tdi_buffer) {
    free(svf_tdi_buffer);
    svf_tdi_buffer = NULL;
  }
  if (svf_tdo_buffer) {
    free(svf_tdo_buffer);
    svf_tdo_buffer = NULL;
  }
  if (svf_mask_buffer) {
    free(svf_mask_buffer);
    svf_mask_buffer = NULL;
  }
}

int SvfCommand::SvfXXRPara::copyHexStrToBinary(const std::string& str,
                                               std::vector<uint8_t>& bin,
                                               int bit_len) {
  int i, str_len = static_cast<int>(str.size()), str_hbyte_len = (bit_len + 3) >> 2;
  uint8_t ch = 0;
  /* fill from LSB (end of str) to MSB (beginning of str) */
  for (i = 0; i < str_hbyte_len; i++) {
    ch = 0;
    while (str_len > 0) {
      ch = str[--str_len];

      /* Skip whitespace.  The SVF specification (rev E) is
       * deficient in terms of basic lexical issues like
       * where whitespace is allowed.  Long bitstrings may
       * require line ends for correctness, since there is
       * a hard limit on line length.
       */
      if (!isspace(ch)) {
        if ((ch >= '0') && (ch <= '9')) {
          ch = ch - '0';
          break;
        } else if ((ch >= 'A') && (ch <= 'F')) {
          ch = ch - 'A' + 10;
          break;
        } else if ((ch >= 'a') && (ch <= 'f')) {
          ch = ch - 'a' + 10;
          break;
        } else {
          LOG_ERROR("invalid hex string");
          return ERROR_FAIL;
        }
      }
      ch = 0;
    }
    /* write bin */
    if (i % 2) {
      /* MSB */
      bin[i / 2] |= ch << 4;
    } else {
      /* LSB */
      bin[i / 2] = 0;
      bin[i / 2] |= ch;
    }
  }
  /* consume optional leading '0' MSBs or whitespace */
  while (str_len > 0 && ((str[str_len - 1] == '0')
        || isspace((int) str[str_len - 1])))
    str_len--;
  /* check validity: we must have consumed everything */
  if (str_len > 0) { //|| (ch & ~((2 << ((bit_len - 1) % 4)) - 1)) != 0) {
    LOG_ERROR("value execeeds length");
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::SvfXXRPara::parseDataToBin(const std::string& type, const std::string& hex_data) {
  std::vector<uint8_t>* pbuffer_tmp = NULL;
  if (!type.compare("TDI")) {
    pbuffer_tmp = &tdi;
  } else if (!type.compare("TDO")) {
    pbuffer_tmp = &tdo;
    set_tdo_used();
  } else if (!type.compare("MASK")) {
    pbuffer_tmp = &mask;
  } else if (!type.compare("SMASK")) {
    return ERROR_OK; // it's seem to be useless
  } else {
    LOG_ERROR("unknow parameter: %s", type.c_str());
    return ERROR_FAIL;
  }
  if (ERROR_OK !=
      copyHexStrToBinary(hex_data, *pbuffer_tmp, bits_len_)) {
    LOG_ERROR("fail to parse hex value");
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::SvfXXRPara::parseData(int len,
                                      const std::string& tdi,
                                      const std::string& tdo,
                                      const std::string& mask) {
  set_bits_len(len);
  CHECK(parseDataToBin("TDI", tdi));
  if (tdo.empty() == false)
    CHECK(parseDataToBin("TDO", tdo));
  if (mask.empty() == false)
    CHECK(parseDataToBin("MASK", mask));
  return ERROR_OK;
}

int SvfCommand::SvfXXRPara::parseData(const std::vector<std::string>& argus) {
  /* XXR length [TDI (tdi)] [TDO (tdo)][MASK (mask)] [SMASK (smask)] */
  const int num_of_argu = static_cast<int>(argus.size());
  if ((num_of_argu > 10) || (num_of_argu % 2)) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  set_bits_len(atoi(argus[1].c_str()));
  LOG_DEBUG("\tlength = %d", bits_len());
  for (int cnt = 2; cnt < num_of_argu; cnt += 2) {
    if ((argus[cnt + 1].size() < 3) ||
        (argus[cnt + 1][0] != '(') ||
        (argus[cnt + 1][argus[cnt + 1].size() - 1] != ')')) {
      LOG_ERROR("data section error");
      return ERROR_FAIL;
    }
    CHECK(parseDataToBin(argus[cnt], argus[cnt+1].substr(1, argus[cnt+1].size()-2)));
  }
  return ERROR_OK;
}

int SvfCommand::SvfXXRPara::processBitstream(int byte_len, uint32_t* tdi_data) {
  set_bits_len(byte_len*8);
  for (int word_cnt = 0; word_cnt < byte_len/4; ++word_cnt) {
    for (int i = 31; i >= 0; --i) {
      int byte_index = 3 - i/8;
      int bit_index = i%8;
      if (tdi_data[word_cnt] & (1 << i)) {
        tdi[word_cnt*4 + byte_index] |= 1 << (7-bit_index) ;
      } else {
        tdi[word_cnt*4 + byte_index] &= ~(1 << (7-bit_index));
      }
    }
  }
  return ERROR_OK;
}
 
void SvfCommand::SvfXXRPara::set_bits_len(int bits_len) {
  bits_len_ = bits_len;
  int byte_len = bits_len_/8 + (bits_len_%8 != 0 ? 1 : 0);
  if (byte_len > static_cast<int>(tdi.size())) {
    tdi.resize(byte_len, 0);
  }
  clear_tdo_flag();
}

void SvfCommand::SvfXXRPara::set_tdo_used() {
  tdo_flag_ = true;
  int byte_len = bits_len_/8 + (bits_len_%8 != 0 ? 1 : 0);
  if (byte_len > static_cast<int>(tdo.size())) {
    tdo.resize(byte_len, 0);
    mask.resize(byte_len, 0);
  }
}

void SvfCommand::SvfXXRPara::setDefaultTdo() {
  int byte_len = bits_len_/8 + (bits_len_%8 != 0 ? 1 : 0);
  if (byte_len > static_cast<int>(tdo.size())) {
    tdo.resize(byte_len, 0);
    mask.resize(byte_len, 0);
  }
  SvfCommand::svf_cmd()->bits_set_zeros(tdo, bits_len_);
  SvfCommand::svf_cmd()->bits_set_zeros(mask, bits_len_);
}

int SvfCommand::SvfPara::assembleDrTdiData(uint8_t* dst) {
  return assembleDrData(dst, "TDI");
}

int SvfCommand::SvfPara::assembleDrTdoData(uint8_t* dst) {
  return assembleDrData(dst, "TDO");
}

int SvfCommand::SvfPara::assembleDrMaskData(uint8_t* dst) {
  return assembleDrData(dst, "MASK");
}

int SvfCommand::SvfPara::assembleIrTdiData(uint8_t* dst) {
  return assembleIrData(dst, "TDI");
}

int SvfCommand::SvfPara::assembleIrTdoData(uint8_t* dst) {
  return assembleIrData(dst, "TDO");
}

int SvfCommand::SvfPara::assembleIrMaskData(uint8_t* dst) {
  return assembleIrData(dst, "MASK");
}

int SvfCommand::SvfPara::assembleDrData(uint8_t *dst, const char* type) {
  int i = 0;
  std::vector<uint8_t>* p_vec = NULL;
  p_vec = !strcmp(type, "TDI") ? &hdr_para.tdi :
          !strcmp(type, "TDO") ? &hdr_para.tdo :
          !strcmp(type, "MASK") ? &hdr_para.mask :
          !strcmp(type, "SMASK") ? &hdr_para.smask : NULL;
  if (p_vec == NULL)
    return ERROR_FAIL;
  vector_set_buf(*p_vec, 0, dst, i, hdr_para.bits_len());
  i += hdr_para.bits_len();
  p_vec = !strcmp(type, "TDI") ? &sdr_para.tdi :
          !strcmp(type, "TDO") ? &sdr_para.tdo :
          !strcmp(type, "MASK") ? &sdr_para.mask :
          !strcmp(type, "SMASK") ? &sdr_para.smask : NULL;
  vector_set_buf(*p_vec, 0, dst, i, sdr_para.bits_len());
  p_vec = !strcmp(type, "TDI") ? &tdr_para.tdi :
          !strcmp(type, "TDO") ? &tdr_para.tdo :
          !strcmp(type, "MASK") ? &tdr_para.mask :
          !strcmp(type, "SMASK") ? &tdr_para.smask : NULL;
  i += sdr_para.bits_len();
  vector_set_buf(*p_vec, 0, dst, i, tdr_para.bits_len());
  i += tdr_para.bits_len();
  return ERROR_OK;
}

int SvfCommand::SvfPara::assembleIrData(uint8_t *dst, const char* type) {
  int i = 0;
  std::vector<uint8_t>* p_vec = NULL;
  p_vec = !strcmp(type, "TDI") ? &hir_para.tdi :
          !strcmp(type, "TDO") ? &hir_para.tdo :
          !strcmp(type, "MASK") ? &hir_para.mask :
          !strcmp(type, "SMASK") ? &hir_para.smask : NULL;
  if (p_vec == NULL)
    return ERROR_FAIL;
  vector_set_buf(*p_vec, 0, dst, i, hir_para.bits_len());
  i += hir_para.bits_len();
  p_vec = !strcmp(type, "TDI") ? &sir_para.tdi :
          !strcmp(type, "TDO") ? &sir_para.tdo :
          !strcmp(type, "MASK") ? &sir_para.mask :
          !strcmp(type, "SMASK") ? &sir_para.smask : NULL;
  vector_set_buf(*p_vec, 0, dst, i, sir_para.bits_len());
  p_vec = !strcmp(type, "TDI") ? &tir_para.tdi :
          !strcmp(type, "TDO") ? &tir_para.tdo :
          !strcmp(type, "MASK") ? &tir_para.mask :
          !strcmp(type, "SMASK") ? &tir_para.smask : NULL;
  i += sir_para.bits_len();
  vector_set_buf(*p_vec, 0, dst, i, tir_para.bits_len());
  i += tir_para.bits_len();
  return ERROR_OK;
}

void SvfCommand::bits_set_ones(std::vector<uint8_t>& buf, unsigned bit_size) {
  for (unsigned i = 0; i < bit_size/8; ++i) {
    buf[i] = 0xff;
  }
  unsigned trailing_bits = bit_size % 8;
  if (trailing_bits)
    buf[bit_size/8] = (1 << trailing_bits) - 1;
}

void SvfCommand::bits_set_zeros(std::vector<uint8_t>& buf, unsigned bit_size) {
  for (unsigned i = 0; i < bit_size/8; ++i) {
    buf[i] = 0;
  }
  if (bit_size%8)
    buf[bit_size/8] = 0;
}

int SvfCommand::checkBufferSpace(int bits_len) {
  int byte_len = (bits_len+7)/8;
  if ((svf_buffer_size - svf_buffer_index) < byte_len) {
    /* reallocate buffer */
    if (svf_realloc_buffers(svf_buffer_index + byte_len) != ERROR_OK) {
      LOG_ERROR("not enough memory");
      return ERROR_FAIL;
    }
  }
  return ERROR_OK;
}

int SvfCommand::flushSvfTap() {
  if (g_svf_nil == false)
    CHECK(jtag_adapter_manager::jtag_execute_queue());
  CHECK(svf_check_tdo());
  svf_buffer_index = 0;
  return ERROR_OK;
}

int SvfCommand::executeSvfTap() {
  if (debug_level >= LOG_LVL_DEBUG) {
    /* for convenient debugging, execute tap if possible */
    if (svf_buffer_index > 0) {
      CHECK(flushSvfTap());
    }
  } else {
    if (procise_flow_ == true) {
      if (svf_buffer_index >= kMaxBufferSizeToCommit_ ||
          check_tdo_para_index_ >= 0)
        CHECK(flushSvfTap());
    } else {
      /* for fast executing, execute tap if necessary */
      /* half of the buffer is for the next command */
      if (svf_buffer_index >= kMaxBufferSizeToCommit_ ||
          check_tdo_para_index_ >= kCheckTdoMaxSize_/2)
        CHECK(flushSvfTap());
    }
  }
  return ERROR_OK;
}

int SvfCommand::cmdHDR(int bits_len) {
  svf_para_.hdr_para.set_bits_len(bits_len);
  bits_set_zeros(svf_para_.hdr_para.tdi, bits_len);
  svf_para_.hdr_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdHDR(const std::vector<std::string>& argus) {
  CHECK(svf_para_.hdr_para.parseData(argus));
  svf_para_.hdr_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdTDR(int bits_len) {
  svf_para_.tdr_para.set_bits_len(bits_len);
  bits_set_zeros(svf_para_.tdr_para.tdi, bits_len);
  svf_para_.tdr_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdTDR(const std::vector<std::string>& argus) {
  CHECK(svf_para_.tdr_para.parseData(argus));
  svf_para_.tdr_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdHIR(int bits_len) {
  svf_para_.hir_para.set_bits_len(bits_len);
  bits_set_ones(svf_para_.hir_para.tdi, bits_len);
  svf_para_.hir_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdHIR(const std::vector<std::string>& argus) {
  CHECK(svf_para_.hir_para.parseData(argus));
  svf_para_.hir_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdTIR(int bits_len) {
  svf_para_.tir_para.set_bits_len(bits_len);
  bits_set_ones(svf_para_.tir_para.tdi, bits_len);
  svf_para_.tir_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::cmdTIR(const std::vector<std::string>& argus) {
  CHECK(svf_para_.tir_para.parseData(argus));
  svf_para_.tir_para.setDefaultTdo();
  return ERROR_OK;
}

int SvfCommand::execSDRCmd() {
  /* check buffer size first, reallocate if necessary */
  int bits_len = svf_para_.dr_total_len();
  CHECK(checkBufferSpace(bits_len));
  CHECK(svf_para_.assembleDrTdiData(&svf_tdi_buffer[svf_buffer_index]));
  if (svf_para_.sdr_para.tdo_flag()) {
    CHECK(svf_para_.assembleDrMaskData(&svf_mask_buffer[svf_buffer_index]));
    CHECK(svf_para_.assembleDrTdoData(&svf_tdo_buffer[svf_buffer_index]));
    svf_add_check_para(1, svf_buffer_index, bits_len);
  }
  if (g_svf_nil == false) {
    /* NOTE:  doesn't use SVF-specified state paths */
    uint8_t* in_value = svf_para_.sdr_para.tdo_flag() ? &svf_tdi_buffer[svf_buffer_index] : NULL;
    jtag_adapter_manager::jtag_add_plain_dr_scan(bits_len,
        &svf_tdi_buffer[svf_buffer_index], in_value, svf_para_.dr_end_state);
  }
  svf_buffer_index += (bits_len+7)/8;
  CHECK(executeSvfTap());
  //SVF_BUF_LOG(DEBUG, svf_tdi_buffer, check_tdo_vec_[0].bit_len, "TDO read"); /* output debug info */
  // crash in windows  FIXME
  return ERROR_OK;
}

int SvfCommand::cmdSDR(int bits_len, const std::string& tdi,
                       const std::string& tdo, const std::string& mask) {
  svf_para_.sdr_para.parseData(bits_len, tdi, tdo, mask);
  if (bits_len == 1035)
    LOG_DEBUG("TEST FOR DEBUG\n");
  return execSDRCmd();
}

int SvfCommand::cmdSDR(const std::vector<std::string>& argus) {
  svf_para_.sdr_para.parseData(argus);
  return execSDRCmd();
}

int SvfCommand::cmdSDRBitstream(int byte_len, uint32_t* tdi_data) {
  svf_para_.sdr_para.processBitstream(byte_len, tdi_data);
  return execSDRCmd();
}

int SvfCommand::cmdSDRGetWaves(int bits_len) {
  svf_para_.sdr_para.set_bits_len(bits_len);
  svf_para_.sdr_para.tdi[0] = 0xa9;
  svf_para_.sdr_para.set_tdo_used();
  return execSDRCmd();
}
  
int SvfCommand::execSIRCmd() {
  /* check buffer size first, reallocate if necessary */
  int bits_len = svf_para_.ir_total_len();
  CHECK(checkBufferSpace(bits_len));
  CHECK(svf_para_.assembleIrTdiData(&svf_tdi_buffer[svf_buffer_index]));
  if (svf_para_.sir_para.tdo_flag()) {
    CHECK(svf_para_.assembleIrMaskData(&svf_mask_buffer[svf_buffer_index]));
    CHECK(svf_para_.assembleIrTdoData(&svf_tdo_buffer[svf_buffer_index]));
    svf_add_check_para(1, svf_buffer_index, bits_len);
  }
  if (g_svf_nil == false) {
    /* NOTE:  doesn't use SVF-specified state paths */
    uint8_t* in_value = svf_para_.sir_para.tdo_flag() ? &svf_tdi_buffer[svf_buffer_index] : NULL;
    jtag_adapter_manager::jtag_add_plain_ir_scan(bits_len,
        &svf_tdi_buffer[svf_buffer_index], in_value, svf_para_.ir_end_state);
  }
  svf_buffer_index += (bits_len+7)/8;
  CHECK(executeSvfTap());
  //SVF_BUF_LOG(DEBUG, svf_tdi_buffer, check_tdo_vec_[0].bit_len, "TDO read"); /* output debug info */
  // crash in windows  FIXME
  return ERROR_OK;
}

int SvfCommand::cmdSIR(int len, const std::string& tdi,
                        const std::string& tdo, const std::string& mask) {
  svf_para_.sir_para.parseData(len, tdi, tdo, mask);
  return execSIRCmd();
}

int SvfCommand::cmdSIR(const std::vector<std::string>& argus) {
  svf_para_.sir_para.parseData(argus);
  return execSIRCmd();
}

int SvfCommand::cmdENDDR(const std::string& state) {
  std::vector<std::string> temp = { "ENDDR" };
  temp.push_back(state);
  return cmdENDDR(temp);
}

int SvfCommand::cmdENDDR(const std::vector<std::string>& argus) {
  const int num_of_argu = static_cast<int>(argus.size());
  if (num_of_argu != 2) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  tap_state_t state = static_cast<tap_state_t>(tap_state_by_name(argus[1].c_str()));
  if (svf_tap_state_is_stable(state)) {
    svf_para_.dr_end_state = state;
    LOG_DEBUG("\tDR end_state = %s", tap_state_name(state));
  } else {
    LOG_ERROR("%s: %s is not a stable state", argus[0].c_str(), argus[1].c_str());
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::cmdFREQUENCY(const std::vector<std::string>& argus) {
  const int num_of_argu = static_cast<int>(argus.size());
  if ((num_of_argu != 1) && (num_of_argu != 3)) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  if (1 == num_of_argu) {
    /* TODO: set jtag speed to full speed */
    svf_para_.frequency = 0;
  } else {
    if (argus[2].compare("HZ")) {
      LOG_ERROR("HZ not found in FREQUENCY command");
      return ERROR_FAIL;
    }
    CHECK(flushSvfTap());
    svf_para_.frequency = atof(argus[1].c_str());
    /* TODO: set jtag speed to */
    if (svf_para_.frequency > 0) {
      command_run_linef("adapter_khz %d", (int)svf_para_.frequency / 1000);
      LOG_DEBUG("\tfrequency = %f", svf_para_.frequency);
    }
  }
  return ERROR_OK;
}
class InterfaceConfig;
class DetectPart;
int SvfCommand::cmdRUNTEST(const std::vector<std::string>& argus) {
  /* RUNTEST [run_state] run_count run_clk [min_time SEC [MAXIMUM max_time
   * SEC]] [ENDSTATE end_state] */
  /* RUNTEST [run_state] min_time SEC [MAXIMUM max_time SEC] [ENDSTATE end_state] */
  const int num_of_argu = static_cast<int>(argus.size());
  if ((num_of_argu < 3) && (num_of_argu > 11)) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  int run_count = 0, cnt = 1;
  float min_time = 0;
  /* run_state */
  tap_state_t tap_state = tap_state_by_name(argus[cnt].c_str());
  if (tap_state != TAP_INVALID) {
    if (svf_tap_state_is_stable(tap_state)) {
      svf_para_.runtest_run_state = tap_state;

      /* When a run_state is specified, the new
       * run_state becomes the default end_state. */
      svf_para_.runtest_end_state = tap_state;
      LOG_DEBUG("\trun_state = %s", tap_state_name(tap_state));
      cnt++;
    } else {
      LOG_ERROR("%s: %s is not a stable state", argus[0].c_str(), tap_state_name(tap_state));
      return ERROR_FAIL;
    }
  }

  /* run_count run_clk */
  if (((cnt + 2) <= num_of_argu) && argus[cnt + 1].compare("SEC")) {
    if (!argus[cnt + 1].compare("TCK")) {
      /* clock source is TCK */
      run_count = atoi(argus[cnt].c_str());
      LOG_DEBUG("\trun_count@TCK = %d", run_count);
    } else {
      LOG_ERROR("%s not supported for clock", argus[cnt + 1].c_str());
      return ERROR_FAIL;
    }
    cnt += 2;
  }
  //run_count adjust for jtag frequency
  if (DetectPart::interface_type() <= kUsbJtagSMT2Cable) {
    float per = InterfaceConfig::adapter_rate_run / 6000;
    if (per >= 1) {
      run_count = run_count * (per + 2);
    }
  }
  /* min_time SEC */
  if (((cnt + 2) <= num_of_argu) && !argus[cnt + 1].compare("SEC")) {
    min_time = atof(argus[cnt].c_str());
    LOG_DEBUG("\tmin_time = %fs", min_time);
    cnt += 2;
  }
  /* MAXIMUM max_time SEC */
  if (((cnt + 3) <= num_of_argu) &&
      !argus[cnt].compare("MAXIMUM") && !argus[cnt + 2].compare("SEC")) {
    float max_time = 0;
    max_time = atof(argus[cnt + 1].c_str());
    LOG_DEBUG("\tmax_time = %fs", max_time);
    cnt += 3;
  }
  /* ENDSTATE end_state */
  if (((cnt + 2) <= num_of_argu) && !argus[cnt].compare("ENDSTATE")) {
    tap_state = tap_state_by_name(argus[cnt + 1].c_str());

    if (svf_tap_state_is_stable(tap_state)) {
      svf_para_.runtest_end_state = tap_state;
      LOG_DEBUG("\tend_state = %s", tap_state_name(tap_state));
    } else {
      LOG_ERROR("%s: %s is not a stable state", argus[0].c_str(), tap_state_name(tap_state));
      return ERROR_FAIL;
    }
    cnt += 2;
  }

  /* all parameter should be parsed */
  if (cnt == num_of_argu) {
#if 1
    /* FIXME handle statemove failures */
    uint32_t min_usec = 1000000 * min_time;

    /* enter into run_state if necessary */
    if (cmd_queue_cur_state != svf_para_.runtest_run_state)
      svf_add_statemove(svf_para_.runtest_run_state);

    /* add clocks and/or min wait */
    if (run_count > 0) {
      if (g_svf_nil == false)
        jtag_adapter_manager::jtag_add_clocks(run_count);
    }

    if (min_usec > 0) {
      if (g_svf_nil == false)
        jtag_adapter_manager::jtag_add_sleep(min_usec);
    }

    /* move to end_state if necessary */
    if (svf_para_.runtest_end_state != svf_para_.runtest_run_state)
      svf_add_statemove(svf_para_.runtest_end_state);

#else
    if (svf_para_.runtest_run_state != TAP_IDLE) {
      LOG_ERROR("cannot runtest in %s state",
          tap_state_name(svf_para_.runtest_run_state));
      return ERROR_FAIL;
    }

    if (g_svf_nil == false)
      jtag_add_runtest(run_count, svf_para_.runtest_end_state);
#endif
  } else {
    LOG_ERROR("fail to parse parameter of RUNTEST, %d out of %d is parsed",
        cnt, num_of_argu);
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::cmdSTATE(const std::string& state) {
  std::vector<std::string> temp = { "STATE" };
  temp.push_back(state);
  return cmdSTATE(temp);
}

int SvfCommand::cmdSTATE(const std::vector<std::string>& argus) {
  /* STATE [pathstate1 [pathstate2 ...[pathstaten]]] stable_state */
  int num_of_argu = static_cast<int>(argus.size());
  if (num_of_argu < 2) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  if (num_of_argu > 2) {
    /* STATE pathstate1 ... stable_state */
    tap_state_t* path = (tap_state_t*) malloc((num_of_argu - 1) * sizeof(tap_state_t));
    if (NULL == path) {
      LOG_ERROR("not enough memory");
      return ERROR_FAIL;
    }
    num_of_argu--;  /* num of path */
    int i_tmp = 1;    /* path is from parameter 1 */
    for (int cnt = 0; cnt < num_of_argu; cnt++, i_tmp++) {
      path[cnt] = tap_state_by_name(argus[i_tmp].c_str());
      if (path[cnt] == TAP_INVALID) {
        LOG_ERROR("%s: %s is not a valid state", argus[0].c_str(), argus[i_tmp].c_str());
        free(path);
        return ERROR_FAIL;
      }
      /* OpenOCD refuses paths containing TAP_RESET */
      if (TAP_RESET == path[cnt]) {
        /* FIXME last state MUST be stable! */
        if (cnt > 0) {
          if (g_svf_nil == false)
            jtag_adapter_manager::jtag_add_pathmove(cnt, path);
        }
        if (g_svf_nil == false)
          jtag_adapter_manager::jtag_add_tlr();
        num_of_argu -= cnt + 1;
        cnt = -1;
      }
    }
    if (num_of_argu > 0) {
      /* execute last path if necessary */
      if (svf_tap_state_is_stable(path[num_of_argu - 1])) {
        /* last state MUST be stable state */
        if (g_svf_nil == false)
          jtag_adapter_manager::jtag_add_pathmove(num_of_argu, path);
        LOG_DEBUG("\tmove to %s by path_move",
            tap_state_name(path[num_of_argu - 1]));
      } else {
        LOG_ERROR("%s: %s is not a stable state",
            argus[0].c_str(),
            tap_state_name(path[num_of_argu - 1]));
        free(path);
        return ERROR_FAIL;
      }
    }

    free(path);
    path = NULL;
  } else {
    /* STATE stable_state */
    tap_state_t state = tap_state_by_name(argus[1].c_str());
    if (svf_tap_state_is_stable(state)) {
      LOG_DEBUG("\tmove to %s by svf_add_statemove", tap_state_name(state));
      /* FIXME handle statemove failures */
      svf_add_statemove(state);
    } else {
      LOG_ERROR("%s: %s is not a stable state", argus[0].c_str(), tap_state_name(state));
      return ERROR_FAIL;
    }
  }
  return ERROR_OK;
}

int SvfCommand::cmdTRST(const std::string& state) {
  std::vector<std::string> temp = { "TRST" };
  temp.push_back(state);
  return cmdTRST(temp);
}

int SvfCommand::cmdTRST(const std::vector<std::string>& argus) {
  if (argus.size() != 2) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  if (svf_para_.trst_mode != TRST_ABSENT) {
    CHECK(flushSvfTap());
    trst_mode mode = static_cast<trst_mode>(svf_find_string_in_array(argus[1].c_str(),
        (char **)svf_trst_mode_name,
        ARRAY_SIZE(svf_trst_mode_name)));
    switch (mode) {
      case TRST_ON:
        if (g_svf_nil == false)
          jtag_adapter_manager::jtag_add_reset(1, 0);
        break;
      case TRST_Z:
      case TRST_OFF:
        if (g_svf_nil == false)
          jtag_adapter_manager::jtag_add_reset(0, 0);
        break;
      case TRST_ABSENT:
        break;
      default:
        LOG_ERROR("unknown TRST mode: %s", argus[1].c_str());
        return ERROR_FAIL;
    }
    svf_para_.trst_mode = mode;
    LOG_DEBUG("\ttrst_mode = %s", svf_trst_mode_name[svf_para_.trst_mode]);
  } else {
    LOG_ERROR("can not accpet TRST command if trst_mode is ABSENT");
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::cmdENDIR(const std::string& state) {
  std::vector<std::string> temp = { "ENDIR" };
  temp.push_back(state);
  return cmdENDIR(temp);
}

int SvfCommand::cmdENDIR(const std::vector<std::string>& argus) {
  if (argus.size() != 2) {
    LOG_ERROR("invalid parameter of %s", argus[0].c_str());
    return ERROR_FAIL;
  }
  tap_state_t state = static_cast<tap_state_t>(tap_state_by_name(argus[1].c_str()));
  if (svf_tap_state_is_stable(state)) {
    svf_para_.ir_end_state = state;
    LOG_DEBUG("\tIR end_state = %s", tap_state_name(state));
  } else {
    LOG_ERROR("%s: %s is not a stable state", argus[0].c_str(), argus[1].c_str());
    return ERROR_FAIL;
  }
  return ERROR_OK;
}

int SvfCommand::svf_realloc_buffers(size_t len)
{
  void *ptr;

  CHECK(flushSvfTap());

  ptr = realloc(svf_tdi_buffer, len);
  if (!ptr)
    return ERROR_FAIL;
  svf_tdi_buffer = (uint8_t*)ptr;

  ptr = realloc(svf_tdo_buffer, len);
  if (!ptr)
    return ERROR_FAIL;
  svf_tdo_buffer = (uint8_t*)ptr;

  ptr = realloc(svf_mask_buffer, len);
  if (!ptr)
    return ERROR_FAIL;
  svf_mask_buffer = (uint8_t*) ptr;

  svf_buffer_size = len;

  return ERROR_OK;
}

/**
 * svf_add_statemove() moves from the current state to @a goal_state.
 *
 * @param goal_state The final TAP state.
 * @return ERROR_OK on success, or an error code on failure.
 *
 * The current and goal states must satisfy svf_tap_state_is_stable().
 * State transition paths used by this routine are those given in the
 * SVF specification for single-argument STATE commands (and also used
 * for various other state transitions).
 */
int SvfCommand::svf_add_statemove(tap_state_t state_to)
{
  tap_state_t state_from = cmd_queue_cur_state;
  unsigned index_var;

  /* when resetting, be paranoid and ignore current state */
  if (state_to == TAP_RESET) {
    if (g_svf_nil == true)
      return ERROR_OK;

    jtag_adapter_manager::jtag_add_tlr();
    return ERROR_OK;
  }

  for (index_var = 0; index_var < ARRAY_SIZE(svf_statemoves); index_var++) {
    if ((svf_statemoves[index_var].from == state_from)
        && (svf_statemoves[index_var].to == state_to)) {
      if (g_svf_nil == true)
        continue;
            /* recorded path includes current state ... avoid
             *extra TCKs! */
      if (svf_statemoves[index_var].num_of_moves > 1)
        jtag_adapter_manager::jtag_add_pathmove(svf_statemoves[index_var].num_of_moves - 1,
          svf_statemoves[index_var].paths + 1);
      else
        jtag_adapter_manager::jtag_add_pathmove(svf_statemoves[index_var].num_of_moves,
          svf_statemoves[index_var].paths);
      return ERROR_OK;
    }
  }
  LOG_ERROR("SVF: can not move to %s", tap_state_name(state_to));
  return ERROR_FAIL;
}

int SvfCommand::executeSvfFile(const std::string& svf_file, jtag_tap* tap) {
  FILE* svf_fd = fopen(svf_file.c_str(), "r");
  if (svf_fd == NULL) {
    int err = errno;
    command_print("open(\"%s\"): %s", svf_file.c_str(), strerror(err));
    /* no need to free anything now */
    return ERROR_COMMAND_SYNTAX_ERROR;
  } else {
    LOG_USER("svf processing file: \"%s\"", svf_file.c_str());
  }

  if (svf_fd == NULL)
    return ERROR_COMMAND_SYNTAX_ERROR;

  /* get time */
  int64_t time_measure_ms = timeval_ms();

  /* double the buffer size */
  /* in case current command cannot be committed, and next command is a bit scan command */
  /* here is 32K bits for this big scan command, it should be enough */
  /* buffer will be reallocated if buffer size is not enough */
  int ret = ERROR_OK;
  if (svf_realloc_buffers(2 * kMaxBufferSizeToCommit_) != ERROR_OK) {
    ret = ERROR_FAIL;
    //goto free_all;
  }

  if (g_svf_nil == false) {
    /* TAP_RESET */
    jtag_adapter_manager::jtag_add_tlr();
  }

  if (tap) {
    /* Tap is specified, set header/trailer paddings */
    int header_ir_len = 0, header_dr_len = 0, trailer_ir_len = 0, trailer_dr_len = 0;
    jtag_tap *check_tap;

    svf_tap_is_specified = 1;

    for (check_tap = jtag_tap::jtag_all_taps(); check_tap; check_tap = check_tap->next_tap) {
      if (check_tap->abs_chain_position < tap->abs_chain_position) {
        /* Header */
        header_ir_len += check_tap->ir_length;
        header_dr_len++;
      } else if (check_tap->abs_chain_position > tap->abs_chain_position) {
        /* Trailer */
        trailer_ir_len += check_tap->ir_length;
        trailer_dr_len++;
      }
    }

    svf_set_padding(&svf_para_.hdr_para, header_dr_len, 0); /* HDR %d TDI (0) */
    svf_set_padding(&svf_para_.hir_para, header_ir_len, 1); /* HIR %d TDI (0xFF) */
    svf_set_padding(&svf_para_.tdr_para, trailer_dr_len, 0); /* TDR %d TDI (0) */
    svf_set_padding(&svf_para_.tir_para, trailer_ir_len, 1); /* TIR %d TDI (0xFF) */
  }

  if (svf_progress_enabled) {
    /* Count total lines in file. */
    while (!feof(svf_fd)) {
      svf_getline(&svf_command_buffer, &svf_command_buffer_size, svf_fd);
      svf_total_lines++;
    }
    rewind(svf_fd);
  }

  int command_num = 0;

  while (ERROR_OK == svf_read_command_from_file(svf_fd)) {
    /* Log Output */
    if (svf_quiet) {
      if (svf_progress_enabled) {
        svf_percentage = ((svf_line_number * 20) / svf_total_lines) * 5;
        if (svf_last_printed_percentage != svf_percentage) {
          LOG_USER_N("\r%d%%    ", svf_percentage);
          svf_last_printed_percentage = svf_percentage;
        }
      }
    } else {
      if (svf_progress_enabled) {
        svf_percentage = ((svf_line_number * 20) / svf_total_lines) * 5;
        LOG_USER_N("%3d%%  %s", svf_percentage, svf_read_line);
      } else
        LOG_USER_N("%s", svf_read_line);
    }
    /* Run Command */
    if (ERROR_OK != svf_run_command(svf_command_buffer)) {
      LOG_ERROR("fail to run command at line %d", svf_line_number);
      ret = ERROR_FAIL;
      break;
    }
    command_num++;
  }

  ret = flushSvfTap();
  /* print time */
  time_measure_ms = timeval_ms() - time_measure_ms;
  int time_measure_s = time_measure_ms / 1000;
  time_measure_ms %= 1000;
  int time_measure_m = time_measure_s / 60;
  time_measure_s %= 60;
  if (time_measure_ms < 1000)
    command_print(
        "\r\nTime used: %dm%ds%lldms ",
        time_measure_m,
        time_measure_s,
        time_measure_ms);

free_all:

  fclose(svf_fd);
  svf_fd = 0;


  if (ERROR_OK == ret)
    command_print(
        "svf file programmed %s for %d commands with %d errors",
        (ignore_error_ == false && svf_ignore_error !=  0) ? "unsuccessfully" : "successfully",
        command_num, svf_ignore_error);
  else
    command_print("svf file programmed failed");

  svf_ignore_error = 0;
  return ret;
}

int handle_svf_command(struct command_invocation *cmd) {
#define SVF_MIN_NUM_OF_OPTIONS 1
#define SVF_MAX_NUM_OF_OPTIONS 5
  SvfCommand* svf_cmd = SvfCommand::svf_cmd();

  /* use NULL to indicate a "plain" svf file which accounts for
   * any additional devices in the scan chain, otherwise the device
   * that should be affected
  */
  jtag_tap *tap = NULL;

  if ((CMD_ARGC < SVF_MIN_NUM_OF_OPTIONS) || (CMD_ARGC > SVF_MAX_NUM_OF_OPTIONS))
    return ERROR_COMMAND_SYNTAX_ERROR;

  std::string svf_file;
  for (unsigned int i = 0; i < CMD_ARGC; i++) {
    if (strcmp(CMD_ARGV[i], "-tap") == 0) {
      tap = jtag_tap::jtag_tap_by_string(CMD_ARGV[i+1]);
      if (!tap) {
        command_print("Tap: %s unknown", CMD_ARGV[i+1]);
        return ERROR_FAIL;
      }
      i++;
    } else if ((strcmp(CMD_ARGV[i],
        "quiet") == 0) || (strcmp(CMD_ARGV[i], "-quiet") == 0))
      svf_cmd->svf_quiet = 1;
    else if ((strcmp(CMD_ARGV[i], "nil") == 0) || (strcmp(CMD_ARGV[i], "-nil") == 0))
      g_svf_nil = true;
    else if ((strcmp(CMD_ARGV[i],
          "progress") == 0) || (strcmp(CMD_ARGV[i], "-progress") == 0))
      svf_cmd->svf_progress_enabled = 1;
    else if ((strcmp(CMD_ARGV[i],
          "ignore_error") == 0) || (strcmp(CMD_ARGV[i], "-ignore_error") == 0))
      svf_cmd->ignore_error_ = true;
    else {
      svf_file = CMD_ARGV[i];
    }
  }
  return svf_cmd->executeSvfFile(svf_file, tap);
}

int SvfCommand::svf_getline(char **lineptr, size_t *n, FILE *stream)
{
#define MIN_CHUNK 10240//16  /* Buffer is increased by this size each time as required */
  size_t i = 0;

  if (*lineptr == NULL) {
    *n = MIN_CHUNK;
    *lineptr = (char*) malloc(*n);
    if (!*lineptr)
      return -1;
  }
  (*lineptr)[0] = fgetc(stream);
  while ((*lineptr)[i] != '\n') {
    (*lineptr)[++i] = fgetc(stream);
    if (feof(stream)) {
      (*lineptr)[0] = 0;
      return -1;
    }
    if ((i + 2) > *n) {
      *n += MIN_CHUNK;
      *lineptr = (char*) realloc(*lineptr, *n);
    }
  }
  (*lineptr)[++i] = 0;

  return sizeof(*lineptr);
}

#define SVFP_CMD_INC_CNT 1024
int SvfCommand::svf_read_command_from_file(FILE *svf_fd)
{
  unsigned char ch;
  int i = 0;
  size_t cmd_pos = 0;
  int cmd_ok = 0, slash = 0;
  if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
    return ERROR_FAIL;
  svf_line_number++;
  ch = svf_read_line[0];
  while (!cmd_ok && (ch != 0)) {
    switch (ch) {
      case '!':
        slash = 0;
        if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
          return ERROR_FAIL;
        svf_line_number++;
        i = -1;
        break;
      case '/':
        if (++slash == 2) {
          slash = 0;
          if (svf_getline(&svf_read_line, &svf_read_line_size,
            svf_fd) <= 0)
            return ERROR_FAIL;
          svf_line_number++;
          i = -1;
        }
        break;
      case ';':
        slash = 0;
        cmd_ok = 1;
        break;
      case '\n':
        svf_line_number++;
        if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
          return ERROR_FAIL;
        i = -1;
      case '\r':
        slash = 0;
        /* Don't save '\r' and '\n' if no data is parsed */
        if (!cmd_pos)
          break;
      default:
        /* The parsing code currently expects a space
         * before parentheses -- "TDI (123)".  Also a
         * space afterwards -- "TDI (123) TDO(456)".
         * But such spaces are optional... instead of
         * parser updates, cope with that by adding the
         * spaces as needed.
         *
         * Ensure there are 3 bytes available, for:
         *  - current character
         *  - added space.
         *  - terminating NUL ('\0')
         */
            if (cmd_pos + 3 > svf_command_buffer_size) {
          svf_command_buffer = (char*)realloc(svf_command_buffer, cmd_pos + 3);
          svf_command_buffer_size = cmd_pos + 3;
          if (svf_command_buffer == NULL) {
            LOG_ERROR("not enough memory");
            return ERROR_FAIL;
          }
        }

        /* insert a space before '(' */
        if ('(' == ch)
          svf_command_buffer[cmd_pos++] = ' ';

        svf_command_buffer[cmd_pos++] = (char)toupper(ch);

        /* insert a space after ')' */
        if (')' == ch)
          svf_command_buffer[cmd_pos++] = ' ';
        break; 
    }    
    ch = svf_read_line[++i];
  }
  if (cmd_ok) {
    svf_command_buffer[cmd_pos] = '\0';
    return ERROR_OK;
  } else
    return ERROR_FAIL;
}

int SvfCommand::svf_parse_cmd_string(char *str, std::vector<std::string>& argus) {
  int pos = 0, num = 0, space_found = 1, in_bracket = 0;
  int len = strlen(str);
  char* temp[256] = { 0 };
  while (pos < len) {
    switch (str[pos]) {
      case '!':
      case '/':
        LOG_ERROR("fail to parse svf command");
        return ERROR_FAIL;
      case '(':
        in_bracket = 1;
        goto parse_char;
      case ')':
        in_bracket = 0;
        goto parse_char;
      default:
parse_char:
        if (!in_bracket && isspace((int) str[pos])) {
          space_found = 1;
          str[pos] = '\0';
        } else if (space_found) {
          temp[num++] = &str[pos];
          space_found = 0;
        }
        break;
    }
    pos++;
  }
  for (int i = 0; i < 256; ++i) {
    if (temp[i] == NULL)
      break;
     argus.push_back(temp[i]);
  }
  return ERROR_OK;
}

/**
 * svf_tap_state_is_stable() returns true for stable non-SHIFT states
 *
 * @param state The TAP state in question
 * @return true iff the state is stable and not a SHIFT state.
 */
bool SvfCommand::svf_tap_state_is_stable(tap_state_t state) {
  return TAP_RESET == state || TAP_IDLE == state ||
         TAP_DRPAUSE == state || TAP_IRPAUSE == state;
}

int SvfCommand::svf_find_string_in_array(const char *str, char **strs, int num_of_element)
{
  int i;

  for (i = 0; i < num_of_element; i++) {
    if (!strcmp(str, strs[i]))
      return i;
  }
  return 0xFF;
}

void SvfCommand::svf_set_padding(SvfXXRPara *para, int len, int bit)
{
  para->set_bits_len(len);
  if (bit == 1) {
    bits_set_ones(para->tdi, len);
  } else {
    bits_set_zeros(para->tdi, len);
  }
  para->setDefaultTdo();
}

int SvfCommand::svf_check_tdo(void) {
  if (procise_flow_ == true) {
    if (check_tdo_para_index_ == 0)
      return ERROR_OK;
    int index_var = check_tdo_vec_[0].buffer_offset;
    int bit_len = check_tdo_vec_[0].bit_len;
    if (ExecuteSvf::process_tdo_type() == ExecuteSvf::kTdoNoCare) {
      // nothing need to do
    } else if (ExecuteSvf::process_tdo_type() == ExecuteSvf::kTdoGetFlag) {
      if (buf_cmp_mask(&svf_tdi_buffer[index_var], &svf_tdo_buffer[index_var],
                       &svf_mask_buffer[index_var], bit_len)) {
        ExecuteSvf::set_tdo_match_flag(false);
      } else {
        ExecuteSvf::set_tdo_match_flag(true);
      }
    } else if (ExecuteSvf::process_tdo_type() == ExecuteSvf::kTdoGetData) {
      ExecuteSvf::set_tdo_data(&svf_tdi_buffer[index_var], bit_len);
    } else {
      return ERROR_FAIL;
    }
  } else {
    for (int i = 0; i < check_tdo_para_index_; i++) {
      int index_var = check_tdo_vec_[i].buffer_offset;
      int bit_len = check_tdo_vec_[i].bit_len;
      if ((check_tdo_vec_[i].enabled) &&
          buf_cmp_mask(&svf_tdi_buffer[index_var], &svf_tdo_buffer[index_var],
                       &svf_mask_buffer[index_var], bit_len)) {
        LOG_ERROR("tdo check error at line %d", check_tdo_vec_[i].line_num);
        SVF_BUF_LOG(ERROR, &svf_tdi_buffer[index_var], bit_len, "READ");
        SVF_BUF_LOG(ERROR, &svf_tdo_buffer[index_var], bit_len, "WANT");
        SVF_BUF_LOG(ERROR, &svf_mask_buffer[index_var], bit_len, "MASK");

        if (ignore_error_ == false)
          return ERROR_FAIL;
        else
          svf_ignore_error++;
      }
    }
  }
  check_tdo_para_index_ = 0;

  return ERROR_OK;
}

int SvfCommand::svf_add_check_para(uint8_t enabled, int buffer_offset, int bit_len)
{
  if (check_tdo_para_index_ >= kCheckTdoMaxSize_) {
    LOG_ERROR("toooooo many operation undone");
    return ERROR_FAIL;
  }

  check_tdo_vec_[check_tdo_para_index_].line_num = svf_line_number;
  check_tdo_vec_[check_tdo_para_index_].bit_len = bit_len;
  check_tdo_vec_[check_tdo_para_index_].enabled = enabled;
  check_tdo_vec_[check_tdo_para_index_].buffer_offset = buffer_offset;
  check_tdo_para_index_++;

  return ERROR_OK;
}

int SvfCommand::svf_run_command(char *cmd_str) {
  std::vector<std::string> argus;
  if (ERROR_OK != svf_parse_cmd_string(cmd_str, argus))
    return ERROR_FAIL;
  
  svf_command_code command = static_cast<svf_command_code>(svf_find_string_in_array(argus[0].c_str(),
        (char **)svf_command_name, ARRAY_SIZE(svf_command_name)));
  switch (command) {
    case kENDDR: CHECK(cmdENDDR(argus)); break;
    case kENDIR: CHECK(cmdENDIR(argus)); break;
    case kFREQUENCY: CHECK(cmdFREQUENCY(argus)); break;
    case kSDR: CHECK(cmdSDR(argus)); break;
    case kSIR: CHECK(cmdSIR(argus)); break;
    case kRUNTEST: CHECK(cmdRUNTEST(argus)); break;
    case kSTATE: CHECK(cmdSTATE(argus)); break;
    case kTRST: CHECK(cmdTRST(argus)); break;
    case kHDR: case kHIR: case kTDR: case kTIR:
      if (svf_tap_is_specified) break;
      switch (command) {
        case kHDR: CHECK(cmdHDR(argus)); break;
        case kHIR: CHECK(cmdHIR(argus)); break;
        case kTDR: CHECK(cmdTDR(argus)); break;
        case kTIR: CHECK(cmdTIR(argus)); break;
      }
      break;
    case kPIO:
    case kPIOMAP:
      LOG_ERROR("PIO and PIOMAP are not supported"); return ERROR_FAIL;
    default:
      LOG_ERROR("invalid svf command: %s", argus[0].c_str());
      return ERROR_FAIL;
  }

  if (!svf_quiet) {
    if (svf_tap_is_specified)
      LOG_USER("(Above Padding command skipped, as per -tap argument)");
  }
  if ((command == kSTATE && argus.size() != 2) == false) {
    if (command != kSTATE && command != kRUNTEST) {
      CHECK(executeSvfTap());
      /* output debug info */
      if ((kSIR == command) || (kSDR == command)) {
        SVF_BUF_LOG(DEBUG, svf_tdi_buffer, check_tdo_vec_[0].bit_len, "TDO read");
      }
    }
  }
  return ERROR_OK;
}

static const struct command_registration svf_command_handlers[] = {
  {
    "svf",
    handle_svf_command,
    NULL,
    NULL,
    COMMAND_EXEC,
    "Runs a SVF file.",
    "svf [-tap device.tap] <file> [quiet] [nil] [progress] [ignore_error]",
    NULL
  },
  COMMAND_REGISTRATION_DONE
};

int svf_register_commands()
{
  return register_commands( NULL, svf_command_handlers);
}
