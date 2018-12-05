//============================================================================
// Copyright (c) 2016, All Right Reserved, FMSH
//
// file:      svf_info.cc
// author:    zhouwei
// purpose:   define the class for svf info
// revision history:
// 
//============================================================================
#include <memory.h>
#include <algorithm>
#include "svf/jtag_svf_info.h"

bool JtagSvfCmd::tdo_match_flag_ = false;
int JtagSvfCmd::bit_len_ = 0;
uint8_t* JtagSvfCmd::tdo_data_ = NULL;
JtagSvfCmd::ProcessTdoData JtagSvfCmd::process_tdo_data_ = kTdoNoCare;

unsigned int HexCharacterToDec(char ch) {
	// handle numeric hex digits
	if (ch >= '0' && ch <= '9') return ch - '0';
	// force lower case to upper case
	ch &= static_cast<char>(0xdf);
	// handle alphabetic hex digits
	if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
	// reject non-hex digits
	return -1;
}

char DecToHexCharacter(unsigned int num) {
	if (num <= 9) return static_cast<char>(num + '0');
	if (num >= 10 && num <= 15) return static_cast<char>(num - 10 + 'A');
	return -1;
}

unsigned int BinaryStringToInt(const std::string inString) {
	unsigned int result = 0;
	const int length = static_cast<int>(inString.length());
	int stop_bit = (inString[0] == '0' && inString[1] == 'b') ? 2 : 0;

	int shift_count = 0;
	while (length - shift_count > stop_bit) {
		if (inString[length - 1 - shift_count] == '1')
			result |= 0x0001 << shift_count;
		shift_count++;
	}

	return result;
}

unsigned int HexStringToInt(const std::string inString) {
	unsigned int result = 0;
	const int length = static_cast<int>(inString.length());
	int stop_bit = (inString[0] == '0' && inString[1] == 'x') ? 2 : 0;

	int shift_count = 0;
	while (length - shift_count > stop_bit) {
		char bit_ch = inString[length - 1 - shift_count];
		unsigned int bit_value = HexCharacterToDec(bit_ch);
		if (bit_value)
			result |= bit_value << (shift_count * 4);
		shift_count++;
	}

	return result;
}

std::string longToHexString(uint64_t num, int bit_width = 16) {
	char buf[512];
	buf[bit_width] = '\0';
	for (int i = 0; i < bit_width; ++i) {
		buf[bit_width - 1 - i] = DecToHexCharacter(static_cast<unsigned int>((num >> (i * 4)) & 0xf));
	}
	return std::string(buf);
}

// class JtagSvfCmd
void JtagSvfCmd::setupRegister() {
  Chain* chain = Chain::chain();
  part_ = chain->parts_[chain->active_part_];
  /* setup register SDR if not already existing */
  if (!(dr_ = part_->PartFindDataRegister("SDR"))) {
    char cmd_para0[] = "register";
    char cmd_para1[] = "SDR";
    char cmd_para2[] = "32";
    char *register_cmd[] = { cmd_para0, cmd_para1, cmd_para2, NULL };
    if (cmd_run(register_cmd) < 1)
      return;
    if (!(dr_ = part_->PartFindDataRegister("SDR"))) {
      return;
    }
  }
  /* setup Instruction SIR if not already existing */
  if (!(ir_ = part_->PartFindInstruction("SIR"))) {
    char *instruction_cmd[] = { "Instruction", "SIR", "", "SDR", NULL };
    char *instruction_string;
    int   len, result;
    len = part_->ir_length();
    if (len > 0) {
      if ((instruction_string = (char *)calloc(len + 1, sizeof(char))) != NULL) {
        memset(instruction_string, '1', len);
        instruction_string[len] = '\0';
        instruction_cmd[2] = instruction_string;
        result = cmd_run(instruction_cmd);
        free(instruction_string);
        if (result < 1)
          return;
      }
    }
    if (!(ir_ = part_->PartFindInstruction("SIR"))) {
      return;
    }
    part_->PartSetInstruction("SIR");
  }
}

void JtagSvfCmd::cmdTRST(int trst_mode) {
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  int  trst_cable = -1;
  char model_z[] = "Z";
  char model_absent[] = "ABSENT";
  char model_unknown[] = "UNKNOWN";
  char *unimplemented_mode;
  switch (trst_mode) {
    case ON: trst_cable = 0; break;
    case OFF:	trst_cable = 1;	break;
    case Z:	unimplemented_mode = model_z;	break; //"Z"
    case ABSENT:
            unimplemented_mode = model_absent; break; //"ABSENT";
    default: break;
  }
  cable->CableSetSignal(CS_TRST, trst_cable ? CS_TRST : 0);
}

void JtagSvfCmd::cmdTRST(const char* trst_mode) {
	cmdTRST(getStateValue(trst_mode));
}

void JtagSvfCmd::cmdGotoState(int new_state) {
  ParserPriv::SvfGotoState(new_state);
}

void JtagSvfCmd::cmdSTATE(int stable_state) {
  ParserPriv::SvfSTATE(stable_state);
}

void JtagSvfCmd::cmdSTATE(const char* stable_state) {
	cmdSTATE(getStateValue(stable_state));
}

void JtagSvfCmd::cmdENDIR(int state) {
  endir_ = ParserPriv::SvfMapState(state);
}

void JtagSvfCmd::cmdENDIR(const char* state) {
	cmdENDIR(getStateValue(state));
}

void JtagSvfCmd::cmdENDDR(int state) {
  enddr_ = ParserPriv::SvfMapState(state);
}

void JtagSvfCmd::cmdENDDR(const char* state) {
	cmdENDDR(getStateValue(state));
}

void JtagSvfCmd::cmdHIR(unsigned int bit_num) {
  TxrHxrParams* hxr = TxrHxrParams::instance();
  TapRegister& hir = hxr->hir_;
  hir.resize(bit_num);
  hir.datas_.assign(bit_num, 1);
};

void JtagSvfCmd::cmdHDR(unsigned int bit_num) {
  TxrHxrParams* hxr = TxrHxrParams::instance();
  TapRegister& hdr = hxr->hdr_;
  hdr.resize(bit_num);
  hdr.datas_.assign(bit_num, 0);
};

void JtagSvfCmd::cmdTIR(unsigned int bit_num) {
  TxrHxrParams* txr = TxrHxrParams::instance();
  TapRegister& tir = txr->tir_;
  tir.resize(bit_num);
  tir.datas_.assign(bit_num, 1);
};

void JtagSvfCmd::cmdTDR(unsigned int bit_num) {
  TxrHxrParams* txr = TxrHxrParams::instance();
  TapRegister& tdr = txr->tdr_;
  tdr.resize(bit_num);
  tdr.datas_.assign(bit_num, 0);
};

int JtagSvfCmd::cmdSIR(int len_sir, const std::string& str_tdi,
	const std::string& str_tdo, const std::string& str_mask) {
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  int len, result = 1;
  TxrHxrParams* txr_hxr = TxrHxrParams::instance();
  TapRegister& tir = txr_hxr->tir_;
  TapRegister& hir = txr_hxr->hir_;
  len = len_sir + tir.len() + hir.len();
  if (ir_->value_.len() != len ||
      (str_tdo.empty() == false && ir_->out_.len() != len)) {
    ir_->value_.resize(len);
    if (str_tdo.size()) {
      ir_->out_.resize(len);
    }
  }
  TapRegister& tr = ir_->value_;
  ParserPriv::SvfBuildBitString(str_tdi.c_str(), len_sir, tr.datas_);
  tr.datas_.insert(tr.datas_.begin(), hir.datas_.begin(), hir.datas_.end()); // insert hxr
  tr.datas_.insert(tr.datas_.end(), tir.datas_.begin(), tir.datas_.end());   // insert txr
#if 0 //for debug
  std::ofstream out("output_data2", std::ios::app | std::ios::binary);
  out << tr.RegisterGetString() << std::endl;
  out.close();
#endif
  /* shift selected Instruction/register */
  cmdGotoState(Shift_IR);
  if (str_tdo.size()) { //if have tdo_, we should flush the queue
    cable->CableFlush(COMPLETELY);
    Chain::merge_count = 0;
  }
  chain->ChainShiftInstructionsMode(str_tdo.size() ? 1 : 0, 0, EXITMODE_EXIT1);
  cmdGotoState(endir_);
  if (str_tdo.size()) {
    ir_->out_.RegisterShiftRight(hir.len());
    result = ParserPriv::SvfCompareTdo(str_tdo.c_str(), str_mask.c_str(), ir_->out_);
  }
  return result;
}

void JtagSvfCmd::svfBuildBitstream(const uint32_t *bitstream, int len, std::vector<char>& ret) {
  ret.resize(0);
  char a;
  for (int cnt = 0; cnt < len / 4; ++cnt) {
    for (int i = 0; i < 32; ++i) {
      char temp = static_cast<char>((bitstream[cnt] >> (31 - i)) & 0x01);
      ret.push_back(temp);
    }
  }
}

int JtagSvfCmd::cmdSDR(int len_sdr, const std::string& str_tdi, 
	const std::string& str_tdo, const std::string& str_mask) {
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  int len, result = 1;
  TxrHxrParams* txr_hxr = TxrHxrParams::instance();
  TapRegister& tdr = txr_hxr->tdr_;
  TapRegister& hdr = txr_hxr->hdr_;
  len = len_sdr + tdr.len() + hdr.len();
  if (dr_->in_.len() != len ||
      (str_tdo.empty() == false && dr_->out_.len() != len)) {
    dr_->in_.resize(len);
    if (str_tdo.size()) {
      dr_->out_.resize(len);
    }
  }
  TapRegister& tr = dr_->in_;
  ParserPriv::SvfBuildBitString(str_tdi.c_str(), len_sdr, tr.datas_);
  tr.datas_.insert(tr.datas_.begin(), hdr.datas_.begin(), hdr.datas_.end()); // insert hxr
  tr.datas_.insert(tr.datas_.end(), tdr.datas_.begin(), tdr.datas_.end());   // insert txr
#if 0 //for debug
  std::ofstream out("output_data2", std::ios::app | std::ios::binary);
  out << tr.RegisterGetString() << std::endl;
  out.close();
#endif
  cmdGotoState(Shift_DR);
  if (str_tdo.size()) { //if have tdo_, we should flush the queue
    cable->CableFlush(COMPLETELY);
    Chain::merge_count = 0;
  } else if (len > 10000) { //in order to free todo resource as soon as possible
    Chain::merge_count = MERGE_NUM - 1;
  }	else {
    Chain::merge_count++;
  }
  chain->ChainShiftDataRegisterMode(str_tdo.size() ? 1 : 0, 0, EXITMODE_EXIT1);
  if (Chain::merge_count%MERGE_NUM == MERGE_NUM - 1)
    Chain::merge_count = 0;
  cmdGotoState(enddr_);
  if (str_tdo.size()) {
    dr_->out_.RegisterShiftRight(hdr.len()); //get the lower bit first
    result = ParserPriv::SvfCompareTdo(str_tdo.c_str(), str_mask.c_str(), dr_->out_);
  }
  return result;
}


int JtagSvfCmd::cmdSdrRawData(int sdr_len, uint32_t*sdr_str) {
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  int len, result = 1;

  TxrHxrParams* txr_hxr = TxrHxrParams::instance();
  TapRegister& tdr = txr_hxr->tdr_;
  TapRegister& hdr = txr_hxr->hdr_;
  len = sdr_len + tdr.len() + hdr.len();
  TapRegister& tr = dr_->in_;
  svfBuildBitstream(sdr_str, sdr_len, tr.datas_);

  tr.datas_.insert(tr.datas_.begin(), hdr.datas_.begin(), hdr.datas_.end()); // insert hxr
  tr.datas_.insert(tr.datas_.end(), tdr.datas_.begin(), tdr.datas_.end());   // insert txr
#if 0 // for debug
  printf("hdr_size:%d  tdr_size:%d sdr_size:%d\n", hdr.datas_.size(), static_cast<int>(tdr.datas_.size()),
      static_cast<int>(tr.datas_.size()));

  std::ofstream out("output_bit_vec");
  for (size_t i = 0; i < tr.datas_.size(); ++i) {
    out << static_cast<char>(tr.datas_[i] + '0');
    if ((i + 1) % 32 == 0)
      out << "\n";
  }
  out.close();
#endif
#if 0 //for debug
  std::ofstream out("output_data2", std::ios::app | std::ios::binary);
  out << tr.RegisterGetString() << std::endl;
  out.close();
#endif
  cmdGotoState(Shift_DR);
  Chain::merge_count = MERGE_NUM - 1;
  chain->ChainShiftDataRegisterMode(0, 0, EXITMODE_EXIT1);
  Chain::merge_count = 0;
  cmdGotoState(enddr_);
  return result;
}

int JtagSvfCmd::cmdSdrGetWaves(int len_sdr) {
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  int len, result = 1;

  TxrHxrParams* txr_hxr = TxrHxrParams::instance();
  TapRegister& tdr = txr_hxr->tdr_;
  TapRegister& hdr = txr_hxr->hdr_;
  len = len_sdr + tdr.len() + hdr.len();
  dr_->in_.datas_.clear();
  dr_->in_.resize(len);
  char str_temp[] = "10101001"; // the last BYTE is "a9"
  for (int i = 0; i < 8; ++i) {
    dr_->in_.datas_[i] = str_temp[7 - i] - '0';
  }
  dr_->out_.resize(len);
  TapRegister& tr = dr_->in_;
  tr.datas_.insert(tr.datas_.begin(), hdr.datas_.begin(), hdr.datas_.end()); // insert hxr
  tr.datas_.insert(tr.datas_.end(), tdr.datas_.begin(), tdr.datas_.end());   // insert txr

  cmdGotoState(Shift_DR);
  cable->CableFlush(COMPLETELY);
  Chain::merge_count = 0;

  chain->ChainShiftDataRegisterMode(1, 0, EXITMODE_EXIT1);
  cmdGotoState(enddr_);

  dr_->out_.RegisterShiftRight(hdr.len());

  if (JtagSvfCmd::tdo_data_ != NULL) {
	  delete JtagSvfCmd::tdo_data_;
  }
  TapRegister& reg = dr_->out_;
  unsigned char temp = 0;
  unsigned int hex_cnt = 0;
  unsigned int hdr_len = txr_hxr->hdr_.len();
  unsigned int tdr_len = txr_hxr->tdr_.len();
  int tdo_len = static_cast<int>(reg.datas_.size() - tdr_len - hdr_len);
  int byte_len = (tdo_len + 7) / 8;
  uint8_t* tdo_data = new uint8_t[byte_len];
  for (int pos_cnt = 0, byte_cnt = 0; pos_cnt < tdo_len; ++pos_cnt) {
    temp += (reg.datas_[pos_cnt] & 0x01) << hex_cnt;
    if (++hex_cnt == 8) {
      tdo_data[byte_cnt++] = temp;
      temp = 0;
      hex_cnt = 0;
    }
  }
  JtagSvfCmd::bit_len_ = tdo_len;
  JtagSvfCmd::tdo_data_ = tdo_data;
  return result;
}

int JtagSvfCmd::cmdRUNTEST(uint32_t run_count_t)	{
  Chain*  chain = Chain::chain();
  Cable*  cable = Cable::cable();
  cmdGotoState(Run_Test_Idle);
  if (run_count_t > 10000) {  //flush the queue
    chain->ChainDeferClock(0, 0, run_count_t / RUNTEST_DIVIDE);
    cable->CableFlush(COMPLETELY);
    Chain::merge_count = 0;
  }	else {
    if (run_count_t < 1000)
      run_count_t *= 10;
    chain->ChainDeferClock(0, 0, run_count_t);
  }
  cmdGotoState(Run_Test_Idle);
  return 1;
}

void JtagSvfCmd::cmdCableFlush() {
  Cable*  cable = Cable::cable();
  cable->CableFlush(COMPLETELY);
}

int JtagSvfCmd::getStateValue(const std::string& state) const {
  if (!state.compare("IDLE")) {
    return IDLE;
  }	else if (!state.compare("OFF")) {
    return OFF;
  }	else if (!state.compare("RESET")) {
    return RESET;
  }	else if (!state.compare("IRPAUSE")) {
    return IRPAUSE;
  }
  return IDLE;
}

//class JtagExecuteSvf
void JtagSvfCmd::executeInitState() {
  setupRegister();
  cmdTRST(OFF);
  cmdENDIR(IDLE);
  cmdENDDR(IDLE);
  cmdSTATE(RESET);
  cmdSTATE(IDLE);
}
